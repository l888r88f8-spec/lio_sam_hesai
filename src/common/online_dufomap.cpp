#include "lio_sam/online_dufomap.h"

#include "common/dufomap_temporal_guard.h"
#include "common/keyframe_scan_cache.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace lio_sam_hesai {
namespace {

constexpr std::size_t kMaxQueuedFrames = 256;
constexpr std::size_t kMaxQueuedBytes = 512U * 1024U * 1024U;
// Convert, persist, and integrate a small number of scans per worker turn so a
// barrier or export never waits for a whole backlog to drain.
constexpr std::size_t kMaxIntegrationBatch = 8;
constexpr std::chrono::milliseconds kClassifyInterval{1000};
constexpr char kCacheMagic[8] = {'L', 'S', 'D', 'U', 'F', '0', '0', '1'};

bool ValidPose(const DufomapPose& pose) {
  for (double value : pose) {
    if (!std::isfinite(value)) return false;
  }
  return std::abs(pose[12]) < 1e-6 && std::abs(pose[13]) < 1e-6 &&
         std::abs(pose[14]) < 1e-6 && std::abs(pose[15] - 1.0) < 1e-6;
}

DufomapPose Multiply(const DufomapPose& lhs, const DufomapPose& rhs) {
  DufomapPose result{};
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      double value = 0.0;
      for (int k = 0; k < 4; ++k) {
        value += lhs[static_cast<std::size_t>(row * 4 + k)] *
                 rhs[static_cast<std::size_t>(k * 4 + col)];
      }
      result[static_cast<std::size_t>(row * 4 + col)] = value;
    }
  }
  return result;
}

DufomapPoint TransformPoint(const DufomapPoint& point,
                            const DufomapPose& pose) {
  return DufomapPoint{
      static_cast<float>(pose[0] * point.x + pose[1] * point.y +
                         pose[2] * point.z + pose[3]),
      static_cast<float>(pose[4] * point.x + pose[5] * point.y +
                         pose[6] * point.z + pose[7]),
      static_cast<float>(pose[8] * point.x + pose[9] * point.y +
                         pose[10] * point.z + pose[11]),
      point.intensity};
}

bool HasFloatField(const sensor_msgs::msg::PointCloud2& message,
                   const std::string& name) {
  for (const auto& field : message.fields) {
    if (field.name == name) {
      return field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
             static_cast<std::uint64_t>(field.offset) + sizeof(float) <=
                 message.point_step;
    }
  }
  return false;
}

bool ValidateCloud(const sensor_msgs::msg::PointCloud2& message,
                   std::string* error) {
  if (message.width == 0 || message.height == 0 ||
      message.point_step == 0 || message.data.empty()) {
    *error = "DUFOMap evidence PointCloud2 is empty";
    return false;
  }
  const std::uint64_t packed_row =
      static_cast<std::uint64_t>(message.point_step) * message.width;
  if (message.row_step < packed_row ||
      static_cast<std::uint64_t>(message.row_step) * message.height >
          message.data.size()) {
    *error = "DUFOMap evidence PointCloud2 has an inconsistent layout";
    return false;
  }
  if (!HasFloatField(message, "x") || !HasFloatField(message, "y") ||
      !HasFloatField(message, "z")) {
    *error = "DUFOMap evidence PointCloud2 needs float x/y/z fields";
    return false;
  }
  return true;
}

bool ValidateEvidence(const OnlineDufomapEvidence& evidence,
                      std::string* error) {
  if (!evidence.cloud) {
    *error = "DUFOMap evidence has no cloud";
    return false;
  }
  if (!ValidateCloud(*evidence.cloud, error)) return false;
  if (evidence.anchor_keyframe < -1) {
    *error = "DUFOMap evidence has an invalid anchor keyframe";
    return false;
  }
  if (evidence.anchor_keyframe >= 0) {
    if (!ValidPose(evidence.anchor_T_scan)) {
      *error = "DUFOMap evidence has an invalid anchor pose";
      return false;
    }
  } else if (!ValidPose(evidence.fallback_map_T_scan)) {
    *error = "DUFOMap evidence has an invalid fallback map pose";
    return false;
  }
  return true;
}

std::vector<DufomapPoint> CloudToDufomap(
    const sensor_msgs::msg::PointCloud2& message) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(message, cloud);
  std::vector<DufomapPoint> points;
  points.reserve(cloud.size());
  for (const auto& point : cloud.points) {
    points.push_back(DufomapPoint{point.x, point.y, point.z, point.intensity});
  }
  return points;
}

bool WriteCacheFile(const std::filesystem::path& path,
                    const std::vector<DufomapPoint>& points,
                    std::string* error) {
  std::error_code filesystem_error;
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path(), filesystem_error);
  }
  if (filesystem_error) {
    *error = "cannot create DUFOMap cache directory " +
             path.parent_path().string() + ": " + filesystem_error.message();
    return false;
  }
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  const std::uint64_t count = points.size();
  stream.write(kCacheMagic, sizeof(kCacheMagic));
  stream.write(reinterpret_cast<const char*>(&count), sizeof(count));
  stream.write(reinterpret_cast<const char*>(points.data()),
               static_cast<std::streamsize>(points.size() *
                                            sizeof(DufomapPoint)));
  if (!stream.good()) {
    *error = "cannot write DUFOMap evidence cache " + path.string();
    return false;
  }
  return true;
}

bool ReadCacheFile(const std::filesystem::path& path,
                   std::vector<DufomapPoint>* points, std::string* error) {
  std::ifstream stream(path, std::ios::binary);
  char magic[sizeof(kCacheMagic)]{};
  std::uint64_t count = 0;
  stream.read(magic, sizeof(magic));
  stream.read(reinterpret_cast<char*>(&count), sizeof(count));
  if (!stream.good() || std::memcmp(magic, kCacheMagic, sizeof(magic)) != 0 ||
      count > 100000000U) {
    *error = "cannot read DUFOMap evidence cache " + path.string();
    return false;
  }
  points->resize(static_cast<std::size_t>(count));
  stream.read(reinterpret_cast<char*>(points->data()),
              static_cast<std::streamsize>(points->size() *
                                           sizeof(DufomapPoint)));
  if (!stream.good()) {
    *error = "truncated DUFOMap evidence cache " + path.string();
    return false;
  }
  return true;
}

struct CachedEvidence {
  std::filesystem::path path;
  std::uint64_t sequence{0};
  int anchor_keyframe{-1};
  DufomapPose anchor_T_scan{};
  DufomapPose fallback_map_T_scan{};
};

struct QueuedEvidence {
  std::uint64_t sequence{0};
  std::size_t bytes{0};
  OnlineDufomapEvidence evidence;
};

// Point data is shared across pose generations; records are immutable once
// they enter the worker list.
struct KeyframeData {
  int index{-1};
  std::vector<DufomapPoint> corner;
  std::vector<DufomapPoint> surf;
};

struct KeyframeRecord {
  std::shared_ptr<const KeyframeData> data;
  DufomapPose pose{};
  std::string complete_scan_path;
};

using KeyframeList = std::vector<std::shared_ptr<const KeyframeRecord>>;

std::shared_ptr<const KeyframeRecord> MakeKeyframeRecord(
    std::shared_ptr<const KeyframeData> data, const DufomapPose& pose,
    std::string complete_scan_path = std::string()) {
  auto record = std::make_shared<KeyframeRecord>();
  record->data = std::move(data);
  record->pose = pose;
  record->complete_scan_path = std::move(complete_scan_path);
  return record;
}

struct ExportJob {
  std::uint64_t id{0};
  DufomapSnapshotRequest request;
  std::vector<int> keyframe_indices;
};

struct ExportOutcome {
  std::uint64_t id{0};
  std::shared_ptr<const KeyframeScanExport> value;
  std::string error;
};

}  // namespace

std::vector<DufomapPoint> ApplyDufomapLabels(
    const std::vector<DufomapPoint>& points,
    const std::vector<std::uint8_t>* labels) {
  if (labels == nullptr) return points;
  if (labels->size() != points.size()) {
    throw std::invalid_argument(
        "online DUFOMap label count does not match the point count");
  }
  std::vector<DufomapPoint> kept;
  kept.reserve(points.size());
  for (std::size_t index = 0; index < points.size(); ++index) {
    if ((*labels)[index] == 0U) kept.push_back(points[index]);
  }
  return kept;
}

class OnlineDufomap::Impl {
 public:
  Impl(const DufomapConfig& config, std::string cache_directory)
      : config_(config),
        cache_directory_(std::move(cache_directory)) {
    // Validate before starting the worker. A failed constructor must not leave
    // a joinable thread behind, otherwise unwinding terminates instead of
    // propagating std::invalid_argument to the caller.
    std::string guard_error;
    if (!ValidateTemporalGuardConfig(config_.temporal_guard, &guard_error)) {
      throw std::invalid_argument(guard_error);
    }
    if (config_.temporal_guard.enable &&
        config_.temporal_guard.coarse_resolution <= config_.resolution) {
      throw std::invalid_argument(
          "dufomap.temporal_guard.coarse_resolution must exceed "
          "dufomap.resolution so the auxiliary grid is actually coarser");
    }
    thread_ = std::thread(&Impl::WorkerMain, this);
  }

  ~Impl() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_.store(true);
    }
    condition_.notify_all();
    if (thread_.joinable()) thread_.join();
  }

  bool Enqueue(const OnlineDufomapEvidence& evidence) {
    std::string error;
    bool accepted = ValidateEvidence(evidence, &error);
    QueuedEvidence queued;
    if (accepted) {
      queued.evidence = evidence;
      queued.bytes = evidence.cloud->data.size();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stop_.load()) return false;
      if (accepted && evidence.anchor_keyframe >= 0 &&
          static_cast<std::size_t>(evidence.anchor_keyframe) >=
              keyframes_.size()) {
        error = "DUFOMap evidence anchor keyframe is not registered";
        accepted = false;
      }
      if (!accepted) {
        ++skipped_;
        if (last_error_.empty()) last_error_ = error;
        condition_.notify_all();
        return false;
      }
      if (config_.max_range > 0.0 && evidence.anchor_keyframe >= 0) {
        const auto& relative = evidence.anchor_T_scan;
        const double offset =
            std::hypot(relative[3], relative[7], relative[11]);
        if (offset > max_anchor_translation_) {
          max_anchor_translation_ = offset;
          std::vector<DufomapPose> poses;
          poses.reserve(keyframes_.size());
          for (const auto& keyframe : keyframes_) {
            poses.push_back(keyframe->pose);
          }
          // A farther scan origin increases the rotational lever arm. Keep
          // earlier micro revisions bounded against this enlarged radius.
          if (PoseRevisionNeedsRebuild(poses)) {
            ++model_epoch_;
            rebuild_reference_poses_ = std::move(poses);
            rebuild_requested_ = true;
          }
        }
      }
      queued.sequence = ++enqueued_sequence_;
      queue_.push_back(std::move(queued));
      queued_bytes_ += queue_.back().bytes;
      while (queue_.size() > 1U &&
             (queue_.size() > kMaxQueuedFrames ||
              queued_bytes_ > kMaxQueuedBytes)) {
        queued_bytes_ -= queue_.front().bytes;
        queue_.pop_front();
        ++dropped_;
      }
    }
    condition_.notify_one();
    return true;
  }

  void RegisterKeyframe(int index, std::vector<DufomapPoint> corner,
                        std::vector<DufomapPoint> surf,
                        const DufomapPose& pose,
                        std::string complete_scan_path) {
    if (!ValidPose(pose)) {
      SetError("online DUFOMap keyframe has an invalid map pose");
      return;
    }
    auto data = std::make_shared<KeyframeData>();
    data->index = index;
    data->corner = std::move(corner);
    data->surf = std::move(surf);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stop_.load()) return;
      if (index < 0 || static_cast<std::size_t>(index) > keyframes_.size()) {
        if (last_error_.empty()) {
          last_error_ =
              "online DUFOMap keyframe index is not sequential: expected " +
              std::to_string(keyframes_.size()) + ", got " +
              std::to_string(index);
        }
        condition_.notify_all();
        return;
      }
      KeyframeList next = keyframes_;
      auto record = MakeKeyframeRecord(std::move(data), pose,
                                       std::move(complete_scan_path));
      if (config_.temporal_guard.enable) {
        // Support is derived from the immutable keyframe list and complete
        // scans, not from continuous evidence. Any keyframe change schedules a
        // support refresh and invalidates a snapshot built from older support.
        ++support_revision_;
        snapshot_.reset();
      }
      if (static_cast<std::size_t>(index) == next.size()) {
        next.push_back(std::move(record));
        // Appending a keyframe extends the fixed motion reference without
        // moving any already-integrated evidence.
        if (rebuild_reference_poses_.size() != keyframes_.size()) {
          rebuild_reference_poses_.clear();
          for (const auto& keyframe : next) {
            rebuild_reference_poses_.push_back(keyframe->pose);
          }
        } else {
          rebuild_reference_poses_.push_back(pose);
        }
        classify_requested_ = true;
      } else {
        next[static_cast<std::size_t>(index)] = std::move(record);
        live_snapshot_.reset();
        // Replacing a registered keyframe moves the fixed reference itself, so
        // the native evidence model must be rebuilt even for a tiny change.
        ++keyframe_generation_;
        ++model_epoch_;
        rebuild_reference_poses_.clear();
        rebuild_reference_poses_.reserve(next.size());
        for (const auto& keyframe : next) {
          rebuild_reference_poses_.push_back(keyframe->pose);
        }
        rebuild_requested_ = true;
        classify_requested_ = true;
      }
      keyframes_ = std::move(next);
    }
    condition_.notify_one();
  }

  void SetKeyframeScanPath(int index, std::string complete_scan_path) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stop_.load()) return;
      if (index < 0 || static_cast<std::size_t>(index) >= keyframes_.size()) {
        // A path attached before its keyframe is registered is a request
        // problem, not a permanent worker failure.
        barrier_error_ = "online DUFOMap complete scan path keyframe index " +
                         std::to_string(index) + " is not registered";
        condition_.notify_all();
        return;
      }
      KeyframeList next = keyframes_;
      const auto& current = next[static_cast<std::size_t>(index)];
      // Replacing only the persisted path keeps the pose generation and the
      // immutable corner/surface target data untouched.
      next[static_cast<std::size_t>(index)] = MakeKeyframeRecord(
          current->data, current->pose, std::move(complete_scan_path));
      keyframes_ = std::move(next);
      if (config_.temporal_guard.enable) {
        // The native evidence epoch does not change, but support must be
        // rebuilt from the replacement scan before the next classification or
        // export. Clearing the snapshot prevents a barrier from returning the
        // previous support result.
        ++support_revision_;
        snapshot_.reset();
        if (live_snapshot_ != nullptr &&
            static_cast<std::size_t>(index) < live_snapshot_->keyframe_count) {
          live_snapshot_.reset();
        }
        classify_requested_ = true;
      }
    }
    condition_.notify_all();
  }

  void UpdateKeyframePoses(const std::vector<DufomapPose>& poses) {
    std::string error;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stop_.load()) return;
      if (poses.size() != keyframes_.size()) {
        error = "online DUFOMap pose generation count does not match keyframes";
      } else {
        for (const auto& pose : poses) {
          if (!ValidPose(pose)) {
            error = "online DUFOMap pose generation contains an invalid pose";
            break;
          }
        }
      }
      if (!error.empty()) {
        if (last_error_.empty()) last_error_ = error;
        condition_.notify_all();
        return;
      }
      bool changed = false;
      for (std::size_t index = 0; index < poses.size(); ++index) {
        if (poses[index] != keyframes_[index]->pose) {
          changed = true;
          break;
        }
      }
      // An exact no-op array changes nothing and must not invalidate a frozen
      // save or schedule more work.
      if (!changed) return;

      const bool rebuild = PoseRevisionNeedsRebuild(poses);
      KeyframeList next;
      next.reserve(keyframes_.size());
      for (std::size_t index = 0; index < keyframes_.size(); ++index) {
        next.push_back(MakeKeyframeRecord(
            keyframes_[index]->data, poses[index],
            keyframes_[index]->complete_scan_path));
      }
      keyframes_ = std::move(next);
      // Every pose revision advances the public pose generation so a frozen
      // save can never mix old transformations with newer map points.
      ++keyframe_generation_;
      if (rebuild) {
        // Only a correction beyond the bounded motion tolerance changes the
        // native history epoch. The fixed reference moves with this request,
        // not with each later tiny correction, so small errors cannot drift.
        ++model_epoch_;
        rebuild_reference_poses_ = poses;
        rebuild_requested_ = true;
      }
      classify_requested_ = true;
    }
    condition_.notify_all();
  }

  DufomapSnapshotRequest CaptureRequest(std::size_t keyframe_count) const {
    std::lock_guard<std::mutex> lock(mutex_);
    DufomapSnapshotRequest request;
    request.keyframe_count = keyframe_count;
    request.keyframe_generation = keyframe_generation_;
    request.evidence_sequence = enqueued_sequence_;
    return request;
  }

  std::optional<std::shared_ptr<const LabelSnapshot>> Barrier(
      const DufomapSnapshotRequest& request,
      std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto fail = [&](const std::string& error) {
      barrier_error_ = error;
      condition_.notify_all();
      return std::optional<std::shared_ptr<const LabelSnapshot>>();
    };
    barrier_error_.clear();
    if (stop_.load()) {
      return fail("online DUFOMap worker is stopped");
    }
    if (!last_error_.empty()) {
      return std::optional<std::shared_ptr<const LabelSnapshot>>();
    }
    if (request.keyframe_generation != keyframe_generation_) {
      return fail(
          "online DUFOMap keyframe generation advanced before the barrier; "
          "refusing to mix captured poses with new labels");
    }
    ++barrier_waiters_;
    condition_.notify_all();
    const bool ready = condition_.wait_for(lock, timeout, [&] {
      if (stop_.load() || !last_error_.empty()) return true;
      if (keyframe_generation_ != request.keyframe_generation) return true;
      return snapshot_ != nullptr &&
             snapshot_->keyframe_generation == request.keyframe_generation &&
             snapshot_->evidence_sequence >= request.evidence_sequence &&
             snapshot_->keyframe_count >= request.keyframe_count &&
             snapshot_->labels.size() >= request.keyframe_count;
    });
    --barrier_waiters_;
    if (stop_.load()) {
      return fail("online DUFOMap worker stopped during the barrier");
    }
    if (!last_error_.empty()) {
      return std::optional<std::shared_ptr<const LabelSnapshot>>();
    }
    if (keyframe_generation_ != request.keyframe_generation) {
      return fail(
          "online DUFOMap keyframe generation advanced during the barrier; "
          "refusing to return labels for the old generation");
    }
    if (!ready || snapshot_ == nullptr ||
        snapshot_->keyframe_generation != request.keyframe_generation ||
        snapshot_->evidence_sequence < request.evidence_sequence ||
        snapshot_->keyframe_count < request.keyframe_count ||
        snapshot_->labels.size() < request.keyframe_count) {
      return fail(
          "online DUFOMap save barrier timed out before the requested "
          "generation and evidence were fully classified");
    }
    return snapshot_;
  }

  std::optional<std::shared_ptr<const LabelSnapshot>> Barrier(
      std::size_t keyframe_count, std::chrono::milliseconds timeout) {
    return Barrier(CaptureRequest(keyframe_count), timeout);
  }

  std::optional<std::shared_ptr<const KeyframeScanExport>> ExportKeyframeScans(
      const DufomapSnapshotRequest& request,
      const std::vector<int>& keyframe_indices,
      std::chrono::milliseconds timeout) {
    // A save owns the only export query and keeps it serial with every other
    // save. The worker itself still owns all backend state.
    std::unique_lock<std::mutex> export_lock(export_mutex_, std::defer_lock);
    export_lock.lock();
    if (keyframe_indices.empty()) {
      SetBarrierError(
          "online DUFOMap complete scan export requested no keyframes");
      return std::nullopt;
    }
    for (std::size_t position = 0; position < keyframe_indices.size();
         ++position) {
      const int index = keyframe_indices[position];
      if (index < 0 ||
          static_cast<std::size_t>(index) >= request.keyframe_count) {
        SetBarrierError(
            "online DUFOMap complete scan export index is outside the "
            "captured keyframes");
        return std::nullopt;
      }
      if (position > 0U && index <= keyframe_indices[position - 1U]) {
        SetBarrierError(
            "online DUFOMap complete scan export indices are not strictly "
            "increasing");
        return std::nullopt;
      }
    }

    const auto deadline = Clock::now() + timeout;
    const auto snapshot = Barrier(request, timeout);
    if (!snapshot.has_value()) return std::nullopt;
    if (Clock::now() >= deadline) {
      SetBarrierError("online DUFOMap complete scan export timed out");
      return std::nullopt;
    }

    std::uint64_t id = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stop_.load()) {
        barrier_error_ = "online DUFOMap worker is stopped";
        return std::nullopt;
      }
      id = ++next_export_id_;
      ExportJob job;
      job.id = id;
      job.request = request;
      job.keyframe_indices = keyframe_indices;
      export_jobs_.push_back(std::move(job));
    }
    condition_.notify_all();

    std::unique_lock<std::mutex> lock(mutex_);
    const bool ready = condition_.wait_until(lock, deadline, [&] {
      return stop_.load() ||
             export_outcomes_.find(id) != export_outcomes_.end();
    });
    if (!ready || stop_.load()) {
      const bool stopped = stop_.load();
      // If the worker published while the deadline expired, consume that
      // outcome instead of leaving a cancellation marker that no worker will
      // ever erase. Otherwise the worker consumes the marker on its next
      // cancellation check and finishes the request without publishing.
      const auto outcome = export_outcomes_.find(id);
      if (outcome != export_outcomes_.end()) {
        export_outcomes_.erase(outcome);
      } else if (!stopped) {
        canceled_exports_.insert(id);
      }
      barrier_error_ = stopped
          ? "online DUFOMap worker stopped during complete scan export"
          : "online DUFOMap complete scan export timed out";
      return std::nullopt;
    }
    const ExportOutcome outcome = std::move(export_outcomes_[id]);
    export_outcomes_.erase(id);
    lock.unlock();
    if (!outcome.value) {
      SetBarrierError(outcome.error.empty()
                          ? "online DUFOMap complete scan export failed"
                          : outcome.error);
      return std::nullopt;
    }
    return outcome.value;
  }

  std::shared_ptr<const LabelSnapshot> LatestSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return live_snapshot_;
  }
  std::size_t dropped_evidence() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_;
  }
  std::size_t skipped_evidence() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return skipped_;
  }
  std::string last_error() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!last_error_.empty()) return last_error_;
    if (!barrier_error_.empty()) return barrier_error_;
    return support_error_;
  }
  std::uint64_t keyframe_generation() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return keyframe_generation_;
  }

 private:
  using Clock = std::chrono::steady_clock;

  // Call under mutex_. Appending beyond a frozen classification does not
  // invalidate its prefix; replacing a covered scan or pose does.
  bool PrefixStillCurrent(const KeyframeList& prefix) const {
    return prefix.size() <= keyframes_.size() &&
           std::equal(prefix.begin(), prefix.end(), keyframes_.begin());
  }

  void SetBarrierError(const std::string& error) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      barrier_error_ = error;
    }
    condition_.notify_all();
  }

  bool TakeCanceledExport(std::uint64_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    return canceled_exports_.erase(id) > 0U;
  }

  void PublishExportSuccess(
      std::uint64_t id, std::uint64_t generation,
      std::uint64_t support_revision,
      std::shared_ptr<const KeyframeScanExport> value) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (canceled_exports_.erase(id) > 0U) return;
      ExportOutcome outcome;
      outcome.id = id;
      // Publish and the final generation/support check share one lock, so a
      // loop correction or complete-scan path change that lands during the
      // last classification cannot return points built from older support.
      const bool generation_changed = keyframe_generation_ != generation;
      const bool support_changed =
          config_.temporal_guard.enable &&
          support_revision_ != support_revision;
      if (generation_changed || support_changed) {
        outcome.error = generation_changed
                            ? "online DUFOMap keyframe generation advanced "
                              "during complete scan export"
                            : "online DUFOMap temporal-guard support revision "
                              "advanced during complete scan export";
      } else {
        outcome.value = std::move(value);
      }
      export_outcomes_[id] = std::move(outcome);
    }
    condition_.notify_all();
  }

  void PublishExportOutcome(std::uint64_t id,
                            std::shared_ptr<const KeyframeScanExport> value,
                            std::string error) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (canceled_exports_.erase(id) > 0U) return;
      ExportOutcome outcome;
      outcome.id = id;
      outcome.value = std::move(value);
      outcome.error = std::move(error);
      export_outcomes_[id] = std::move(outcome);
    }
    condition_.notify_all();
  }

  // The single fine/coarse/support policy path used by live RViz labels, full
  // complete-scan export, and export fallback feature labels. When support is
  // not fully ready, coarse-only removal is conservatively disabled and only
  // the authoritative fine labels are returned.
  std::vector<std::uint8_t> ClassifyWithTemporalGuard(
      DufomapBackend* fine, DufomapBackend* coarse,
      const KeyframeSupportModel* support, bool support_ready,
      const std::vector<DufomapPoint>& points) const {
    std::vector<std::uint8_t> fine_labels = fine->Classify(points);
    if (!config_.temporal_guard.enable || !support_ready || coarse == nullptr ||
        support == nullptr) {
      return fine_labels;
    }
    const std::vector<std::uint8_t> coarse_labels = coarse->Classify(points);
    return ApplyTemporalGuard(fine_labels, coarse_labels, points, *support,
                              config_.temporal_guard.min_keyframe_support);
  }

  void SetSupportError(const std::string& error) {
    std::lock_guard<std::mutex> lock(mutex_);
    support_error_ = error;
  }

  void ClearSupportError() {
    std::lock_guard<std::mutex> lock(mutex_);
    support_error_.clear();
  }

  void RefreshSupportIfNeeded(const KeyframeList& keyframes,
                              std::uint64_t model_epoch,
                              std::uint64_t latest_generation,
                              std::uint64_t support_revision) {
    if (!config_.temporal_guard.enable) {
      support_ready_ = false;
      support_model_.reset();
      support_keyframes_.clear();
      return;
    }
    // Key failed attempts by the exact epoch/generation/revision they tried.
    // Re-entering the same state every batch must not re-read a missing or
    // corrupt complete scan; SetKeyframeScanPath bumps support_revision and
    // allows a retry. A successful append with a newer path revision also
    // passes through here because the revision changed.
    if (support_attempted_ &&
        attempted_support_model_epoch_ == model_epoch &&
        attempted_support_pose_generation_ == latest_generation &&
        attempted_support_revision_ == support_revision) {
      return;
    }
    support_attempted_ = true;
    attempted_support_model_epoch_ = model_epoch;
    attempted_support_pose_generation_ = latest_generation;
    attempted_support_revision_ = support_revision;

    // Append only when the cached prefix was built from the same model epoch
    // and the exact same pose generation. Any path replacement/reduction,
    // pose generation change, or model epoch change needs a full rebuild. A
    // newer revision with no new keyframe can mean an existing path was
    // rewritten in place, so it also requires a full rebuild.
    bool append = support_model_ != nullptr &&
                  built_support_model_epoch_ == model_epoch &&
                  built_support_pose_generation_ == latest_generation &&
                  support_keyframes_.size() < keyframes.size();
    if (append) {
      for (std::size_t index = 0; index < support_keyframes_.size(); ++index) {
        // Record identity also detects a scan replaced at the same path.
        if (keyframes[index] != support_keyframes_[index]) {
          append = false;
          break;
        }
      }
    }
    if (!append) {
      support_model_ = std::make_unique<KeyframeSupportModel>(
          config_.temporal_guard.coarse_resolution,
          config_.temporal_guard.min_keyframe_support);
      support_keyframes_.clear();
      support_ready_ = false;
      built_support_model_epoch_ = model_epoch;
      built_support_pose_generation_ = latest_generation;
    } else {
      // The retained prefix is not a complete support model while the newly
      // appended keyframes are being read.
      support_ready_ = false;
    }

    try {
      for (std::size_t index = support_keyframes_.size();
           index < keyframes.size(); ++index) {
        if (stop_.load() || CurrentModelEpoch() != model_epoch ||
            keyframe_generation() != latest_generation) {
          support_ready_ = false;
          return;
        }
        const auto& keyframe = keyframes[index];
        if (keyframe->complete_scan_path.empty()) {
          support_ready_ = false;
          SetSupportError(
              "online DUFOMap temporal-guard support is not ready: complete "
              "keyframe scan path is missing for keyframe " +
              std::to_string(index));
          return;
        }
        std::vector<DufomapPoint> local;
        std::string read_error;
        if (!ReadKeyframeScanCacheFile(keyframe->complete_scan_path, index,
                                       &local, &read_error)) {
          support_ready_ = false;
          SetSupportError(
              "online DUFOMap temporal-guard support is not ready: " +
              read_error);
          return;
        }
        std::vector<DufomapPoint> world;
        world.reserve(local.size());
        // latest_keyframes was captured together with latest_generation, so
        // this transform is the exact pose for the support revision we build.
        for (const auto& point : local) {
          world.push_back(TransformPoint(point, keyframe->pose));
        }
        if (!support_model_->AddKeyframe(index, world)) {
          support_ready_ = false;
          SetSupportError(
              "online DUFOMap temporal-guard support keyframe order is not "
              "strictly increasing at keyframe " +
              std::to_string(index));
          return;
        }
        support_keyframes_.push_back(keyframe);
      }
    } catch (const std::exception& exception) {
      support_ready_ = false;
      SetSupportError(
          std::string("online DUFOMap temporal-guard support failed: ") +
          exception.what());
      return;
    } catch (...) {
      support_ready_ = false;
      SetSupportError(
          "online DUFOMap temporal-guard support failed with an unknown error");
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stop_.load() || model_epoch_ != model_epoch ||
          keyframe_generation_ != latest_generation ||
          (support_revision_ != support_revision &&
           !PrefixStillCurrent(keyframes))) {
        // A later loop will rebuild against the newer immutable state.
        support_ready_ = false;
        return;
      }
    }
    support_ready_ = true;
    ClearSupportError();
  }

  void ProcessExportJob(const ExportJob& job, std::uint64_t model_epoch,
                        DufomapBackend* backend,
                        DufomapBackend* coarse_backend,
                        const KeyframeSupportModel* support,
                        bool support_ready,
                        std::uint64_t built_model_epoch,
                        std::uint64_t integrated_sequence,
                        std::uint64_t job_support_revision) {
    const auto fail = [&](const std::string& error) {
      PublishExportOutcome(job.id, nullptr, error);
    };
    if (TakeCanceledExport(job.id)) return;

    // A complete-scan path can be attached after the worker took this loop's
    // keyframe snapshot. Re-read the immutable list for the captured pose
    // generation so a just-completed cache write is not missed. The snapshot is
    // only reusable when it covers the same backend evidence the complete scans
    // are about to be classified against.
    KeyframeList export_keyframes;
    std::shared_ptr<const LabelSnapshot> reusable_labels;
    std::string snapshot_error;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stop_.load()) {
        snapshot_error =
            "online DUFOMap worker stopped during complete scan export";
      } else if (job.request.keyframe_generation != keyframe_generation_) {
        snapshot_error =
            "online DUFOMap complete scan export generation advanced; "
            "refusing to mix captured poses with a newer backend";
      } else if (model_epoch_ != model_epoch) {
        snapshot_error =
            "online DUFOMap complete scan export model epoch advanced; "
            "refusing to mix captured poses with newer evidence";
      } else if (config_.temporal_guard.enable &&
                 support_revision_ != job_support_revision) {
        snapshot_error =
            "online DUFOMap complete scan export temporal-guard support "
            "revision advanced; refusing to mix a newer complete scan path "
            "with older support";
      } else {
        export_keyframes = keyframes_;
        if (snapshot_ != nullptr &&
            snapshot_->keyframe_generation == job.request.keyframe_generation &&
            snapshot_->evidence_sequence == integrated_sequence &&
            snapshot_->keyframe_count >= job.request.keyframe_count &&
            snapshot_->labels.size() >= job.request.keyframe_count) {
          reusable_labels = snapshot_;
        }
      }
    }
    if (!snapshot_error.empty()) {
      fail(snapshot_error);
      return;
    }
    if (model_epoch != built_model_epoch || backend == nullptr) {
      fail(
          "online DUFOMap complete scan export model epoch advanced; refusing "
          "to mix captured poses with a newer backend");
      return;
    }
    if (integrated_sequence < job.request.evidence_sequence) {
      fail(
          "online DUFOMap complete scan export evidence is older than the "
          "captured request");
      return;
    }
    if (export_keyframes.size() < job.request.keyframe_count) {
      fail(
          "online DUFOMap complete scan export keyframes do not cover the "
          "captured request");
      return;
    }

    auto result = std::make_shared<KeyframeScanExport>();
    result->keyframe_generation = job.request.keyframe_generation;
    result->evidence_sequence = integrated_sequence;
    result->keyframe_count = job.request.keyframe_count;
    for (const int index : job.keyframe_indices) {
      if (stop_.load()) {
        fail("online DUFOMap worker stopped during complete scan export");
        return;
      }
      if (TakeCanceledExport(job.id)) return;
      if (!ExportPoseStillCurrent(job.request.keyframe_generation,
                                  model_epoch, job_support_revision)) {
        fail(
            "online DUFOMap keyframe generation advanced during complete "
            "scan export");
        return;
      }
      if (index < 0 ||
          static_cast<std::size_t>(index) >= export_keyframes.size()) {
        fail("online DUFOMap complete scan export keyframe is not registered");
        return;
      }
      const auto& keyframe = export_keyframes[static_cast<std::size_t>(index)];
      if (keyframe->complete_scan_path.empty()) {
        fail("complete keyframe scan cache path is not set for keyframe " +
             std::to_string(index));
        return;
      }
      try {
        std::vector<DufomapPoint> local_points;
        std::string read_error;
        if (!ReadKeyframeScanCacheFile(
                keyframe->complete_scan_path,
                static_cast<std::size_t>(index), &local_points, &read_error)) {
          fail(read_error);
          return;
        }
        std::vector<DufomapPoint> map_points;
        map_points.reserve(local_points.size());
        for (const auto& point : local_points)
          map_points.push_back(TransformPoint(point, keyframe->pose));
        const std::vector<std::uint8_t> labels =
            ClassifyWithTemporalGuard(backend, coarse_backend, support,
                                      support_ready, map_points);
        if (labels.size() != map_points.size()) {
          fail(
              "online DUFOMap complete scan classification returned a "
              "mismatched label count");
          return;
        }
        for (std::size_t point_index = 0; point_index < map_points.size();
             ++point_index) {
          if (labels[point_index] == 0U)
            result->points.push_back(map_points[point_index]);
        }
      } catch (const std::exception& exception) {
        // Export failures are request-scoped: they must not stop live
        // classification or poison the permanent worker error.
        fail(std::string("online DUFOMap complete scan export failed: ") +
             exception.what());
        return;
      } catch (...) {
        fail(
            "online DUFOMap complete scan export failed with an unknown "
            "error");
        return;
      }
    }

    std::shared_ptr<const LabelSnapshot> feature_labels = reusable_labels;
    if (!feature_labels) {
      auto captured_labels = std::make_shared<LabelSnapshot>();
      captured_labels->keyframe_generation = job.request.keyframe_generation;
      captured_labels->evidence_sequence = integrated_sequence;
      captured_labels->keyframe_count = job.request.keyframe_count;
      captured_labels->labels.resize(job.request.keyframe_count);
      try {
        for (std::size_t index = 0; index < job.request.keyframe_count;
             ++index) {
          if (stop_.load()) {
            fail("online DUFOMap worker stopped during complete scan export");
            return;
          }
          if (TakeCanceledExport(job.id)) return;
          if (!ExportPoseStillCurrent(job.request.keyframe_generation,
                                      model_epoch, job_support_revision)) {
            fail(
                "online DUFOMap keyframe generation advanced during complete "
                "scan export");
            return;
          }
          const auto& keyframe = export_keyframes[index];
          std::vector<DufomapPoint> corner;
          std::vector<DufomapPoint> surf;
          corner.reserve(keyframe->data->corner.size());
          surf.reserve(keyframe->data->surf.size());
          for (const auto& point : keyframe->data->corner)
            corner.push_back(TransformPoint(point, keyframe->pose));
          for (const auto& point : keyframe->data->surf)
            surf.push_back(TransformPoint(point, keyframe->pose));
          captured_labels->labels[index].corner =
              ClassifyWithTemporalGuard(backend, coarse_backend, support,
                                        support_ready, corner);
          captured_labels->labels[index].surf =
              ClassifyWithTemporalGuard(backend, coarse_backend, support,
                                        support_ready, surf);
        }
      } catch (const std::exception& exception) {
        fail(std::string("online DUFOMap feature classification failed: ") +
             exception.what());
        return;
      } catch (...) {
        fail(
            "online DUFOMap feature classification failed with an unknown "
            "error");
        return;
      }
      feature_labels = std::move(captured_labels);
    }
    result->feature_labels = std::move(feature_labels);
    PublishExportSuccess(job.id, job.request.keyframe_generation,
                         job_support_revision, std::move(result));
  }

  DufomapPose EvidencePose(const CachedEvidence& evidence,
                           const std::vector<DufomapPose>& poses) const {
    if (evidence.anchor_keyframe >= 0 &&
        static_cast<std::size_t>(evidence.anchor_keyframe) < poses.size()) {
      return Multiply(poses[static_cast<std::size_t>(evidence.anchor_keyframe)],
                      evidence.anchor_T_scan);
    }
    return evidence.fallback_map_T_scan;
  }

  void SetError(const std::string& error) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (last_error_.empty()) last_error_ = error;
    }
    condition_.notify_all();
  }

  double RebuildTolerance() const {
    if (config_.d_s <= 0.0) return 0.0;
    // Each retained/current pose stays within this reference budget. Their
    // combined error stays below half a voxel and half the hit inflation.
    return std::max(
        0.0, std::min(config_.resolution, config_.d_s) * 0.25);
  }

  // True when at least one pose has moved far enough from the fixed reference
  // that reusing the old native history could move an observed endpoint by more
  // than RebuildTolerance(). Include the scan-origin offset in the rotation
  // radius. Direct matrix differences also handle float-rounded rotations.
  bool PoseRevisionNeedsRebuild(const std::vector<DufomapPose>& poses) const {
    if (rebuild_reference_poses_.size() != poses.size()) return true;
    const double tolerance = RebuildTolerance();
    const bool rotation_unbounded = config_.max_range <= 0.0;
    for (std::size_t index = 0; index < poses.size(); ++index) {
      const DufomapPose& pose = poses[index];
      const DufomapPose& reference = rebuild_reference_poses_[index];
      const double dx = pose[3] - reference[3];
      const double dy = pose[7] - reference[7];
      const double dz = pose[11] - reference[11];
      const double translation = std::sqrt(dx * dx + dy * dy + dz * dz);

      double rotation_squared = 0.0;
      std::array<double, 3> row_sums{};
      std::array<double, 3> column_sums{};
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
          const std::size_t entry = static_cast<std::size_t>(row * 4 + col);
          const double delta = pose[entry] - reference[entry];
          rotation_squared += delta * delta;
          row_sums[row] += std::abs(delta);
          column_sums[col] += std::abs(delta);
        }
      }
      // Both expressions bound the matrix spectral norm. The row/column
      // bound is tighter for small yaw changes without assuming exact R*R^T.
      const double rotation = std::min(
          std::sqrt(rotation_squared),
          std::sqrt(*std::max_element(row_sums.begin(), row_sums.end()) *
                    *std::max_element(column_sums.begin(), column_sums.end())));
      if (translation > tolerance) return true;
      if (rotation_unbounded) {
        // max_range == 0 has no finite endpoint radius: any rotation change
        // can move an unbounded-far endpoint, so it must rebuild.
        if (rotation > 0.0) return true;
      } else if (translation +
                     (config_.max_range + max_anchor_translation_) * rotation >
                 tolerance) {
        return true;
      }
    }
    return false;
  }

  std::uint64_t CurrentModelEpoch() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return model_epoch_;
  }

  bool ExportPoseStillCurrent(std::uint64_t generation,
                              std::uint64_t model_epoch,
                              std::uint64_t support_revision) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return keyframe_generation_ == generation && model_epoch_ == model_epoch &&
           (!config_.temporal_guard.enable ||
            support_revision_ == support_revision);
  }

  bool WorkReady() const {
    if (stop_.load() || !queue_.empty() || !export_jobs_.empty() ||
        rebuild_requested_ || classify_requested_) {
      return true;
    }
    return pending_classification_ && last_error_.empty() &&
           (barrier_waiters_ > 0 || Clock::now() >= next_classification_);
  }

  bool StoreBatch(const std::deque<QueuedEvidence>& batch,
                  std::vector<CachedEvidence>* persisted) {
    persisted->reserve(batch.size());
    for (const auto& queued : batch) {
      if (stop_.load()) return false;
      std::vector<DufomapPoint> points;
      try {
        points = CloudToDufomap(*queued.evidence.cloud);
      } catch (const std::exception& exception) {
        SetError(std::string("DUFOMAP evidence conversion failed: ") +
                 exception.what());
        return false;
      }
      if (points.empty()) {
        SetError("DUFOMap evidence conversion produced no points");
        return false;
      }
      CachedEvidence cached;
      cached.path = cache_directory_ / "scans" /
                    ("scan_" + std::to_string(queued.sequence) + ".bin");
      std::string error;
      if (!WriteCacheFile(cached.path, points, &error)) {
        SetError(error);
        return false;
      }
      cached.sequence = queued.sequence;
      cached.anchor_keyframe = queued.evidence.anchor_keyframe;
      cached.anchor_T_scan = queued.evidence.anchor_T_scan;
      cached.fallback_map_T_scan = queued.evidence.fallback_map_T_scan;
      persisted->push_back(std::move(cached));
    }
    return true;
  }

  void WorkerMain() {
    std::unique_ptr<DufomapBackend> backend;
    std::unique_ptr<DufomapBackend> coarse_backend;
    std::uint64_t built_model_epoch = 0;
    std::uint64_t integrated_sequence = 0;

    while (true) {
      std::deque<QueuedEvidence> batch;
      std::vector<CachedEvidence> cached_before;
      KeyframeList keyframes;
      std::uint64_t model_epoch = 0;
      std::uint64_t support_revision = 0;
      bool rebuild = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!WorkReady()) {
          if (pending_classification_ && last_error_.empty() &&
              barrier_waiters_ == 0) {
            condition_.wait_until(lock, next_classification_);
          } else {
            condition_.wait(lock);
          }
          continue;
        }
        if (stop_.load()) break;
        while (!queue_.empty() && batch.size() < kMaxIntegrationBatch) {
          QueuedEvidence queued = std::move(queue_.front());
          queue_.pop_front();
          queued_bytes_ -= queued.bytes;
          batch.push_back(std::move(queued));
        }
        const bool requested_rebuild = rebuild_requested_;
        rebuild_requested_ = false;
        const bool classify_requested = classify_requested_;
        classify_requested_ = false;
        if (!batch.empty() || classify_requested) {
          pending_classification_ = true;
        }
        model_epoch = model_epoch_;
        support_revision = support_revision_;
        keyframes = keyframes_;
        rebuild = requested_rebuild || !backend ||
                  model_epoch != built_model_epoch;
        if (rebuild) {
          cached_before = cached_;
          pending_classification_ = true;
        }
      }

      std::vector<CachedEvidence> persisted;
      if (!batch.empty()) {
        if (!StoreBatch(batch, &persisted)) {
          batch.clear();
          condition_.notify_all();
          continue;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          cached_.insert(cached_.end(), persisted.begin(), persisted.end());
        }
        batch.clear();
      }

      std::vector<DufomapPose> poses;
      poses.reserve(keyframes.size());
      for (const auto& keyframe : keyframes) poses.push_back(keyframe->pose);

      bool model_changed = false;
      try {
        if (rebuild) {
          // The online worker must not inherit the bounded export timeout:
          // live mapping has no fixed session lifetime. Save keeps its own
          // barrier timeout.
          DufomapConfig backend_config = config_;
          backend_config.timeout_seconds = 0.0;
          auto candidate = std::make_unique<DufomapBackend>(backend_config);
          std::unique_ptr<DufomapBackend> coarse_candidate;
          if (config_.temporal_guard.enable) {
            DufomapConfig coarse_config = backend_config;
            coarse_config.resolution =
                config_.temporal_guard.coarse_resolution;
            coarse_candidate =
                std::make_unique<DufomapBackend>(coarse_config);
          }
          std::vector<CachedEvidence> replay = cached_before;
          replay.insert(replay.end(), persisted.begin(), persisted.end());
          for (const auto& cached : replay) {
            if (stop_.load()) return;
            // Tiny pose revisions only move the classification transform; the
            // native history is still within its bounded motion tolerance.
            if (CurrentModelEpoch() != model_epoch) {
              model_changed = true;
              break;
            }
            std::vector<DufomapPoint> points;
            std::string error;
            if (!ReadCacheFile(cached.path, &points, &error)) {
              throw std::runtime_error(error);
            }
            const DufomapPose evidence_pose = EvidencePose(cached, poses);
            candidate->Integrate(points, evidence_pose);
            if (coarse_candidate) {
              coarse_candidate->Integrate(points, evidence_pose);
            }
            integrated_sequence =
                std::max(integrated_sequence, cached.sequence);
          }
          if (!model_changed) {
            candidate->Propagate();
            if (coarse_candidate) coarse_candidate->Propagate();
            backend = std::move(candidate);
            coarse_backend = std::move(coarse_candidate);
            support_model_.reset();
            support_keyframes_.clear();
            support_ready_ = false;
            support_attempted_ = false;
            built_model_epoch = model_epoch;
          }
        } else if (!persisted.empty()) {
          if (!backend) {
            throw std::runtime_error("online DUFOMap backend is unavailable");
          }
          for (const auto& cached : persisted) {
            if (stop_.load()) return;
            if (CurrentModelEpoch() != model_epoch) {
              model_changed = true;
              break;
            }
            std::vector<DufomapPoint> points;
            std::string error;
            if (!ReadCacheFile(cached.path, &points, &error)) {
              throw std::runtime_error(error);
            }
            const DufomapPose evidence_pose = EvidencePose(cached, poses);
            backend->Integrate(points, evidence_pose);
            if (coarse_backend) {
              coarse_backend->Integrate(points, evidence_pose);
            }
            integrated_sequence =
                std::max(integrated_sequence, cached.sequence);
          }
          if (!model_changed) {
            backend->Propagate();
            if (coarse_backend) coarse_backend->Propagate();
          }
        }
      } catch (const std::exception& exception) {
        SetError(std::string("online DUFOMap worker failed: ") +
                 exception.what());
        condition_.notify_all();
        continue;
      }
      if (model_changed) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          rebuild_requested_ = true;
        }
        condition_.notify_all();
        continue;
      }

      // Integration deliberately used the poses captured at the start of this
      // bounded batch. Once it finishes, switch to the newest immutable poses
      // for classification and export as long as the model epoch is unchanged.
      KeyframeList latest_keyframes;
      std::uint64_t latest_generation = 0;
      std::uint64_t latest_support_revision = 0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_.load()) break;
        if (model_epoch_ != model_epoch) {
          rebuild_requested_ = true;
          condition_.notify_all();
          continue;
        }
        latest_keyframes = keyframes_;
        latest_generation = keyframe_generation_;
        latest_support_revision = support_revision_;
      }

      // Support must be current before any export or live classification. It
      // comes only from complete keyframe scans, never from continuous anchor
      // evidence. A missing/corrupt path disables coarse-only removal while the
      // fine decision continues unchanged.
      RefreshSupportIfNeeded(latest_keyframes, model_epoch, latest_generation,
                             latest_support_revision);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_.load()) break;
        if (model_epoch_ != model_epoch) {
          rebuild_requested_ = true;
          condition_.notify_all();
          continue;
        }
        if (support_revision_ != latest_support_revision &&
            !PrefixStillCurrent(latest_keyframes)) {
          // A path or keyframe changed while support was being built. Do not
          // use or export the older snapshot; loop and rebuild support first.
          classify_requested_ = true;
          pending_classification_ = true;
          condition_.notify_all();
          continue;
        }
      }

      // Export jobs run on the worker thread after the captured barrier, so
      // the complete scans are classified against the exact pose generation
      // the save froze. A failure here stays request-scoped.
      std::deque<ExportJob> export_jobs;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        export_jobs.swap(export_jobs_);
      }
      for (const auto& job : export_jobs) {
        if (stop_.load()) break;
        try {
          ProcessExportJob(job, model_epoch, backend.get(),
                           coarse_backend.get(), support_model_.get(),
                           support_ready_, built_model_epoch,
                           integrated_sequence, latest_support_revision);
        } catch (const std::exception& exception) {
          // A bad cache read, conversion, or classification must fail only
          // this request and keep the worker alive for live mapping.
          PublishExportOutcome(
              job.id, nullptr,
              std::string("online DUFOMap complete scan export failed: ") +
                  exception.what());
        } catch (...) {
          PublishExportOutcome(
              job.id, nullptr,
              "online DUFOMap complete scan export failed with an unknown "
              "error");
        }
      }
      if (stop_.load()) break;

      bool classify = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_.load()) break;
        if (model_epoch_ != model_epoch) {
          rebuild_requested_ = true;
          condition_.notify_all();
          continue;
        }
        const bool due =
            rebuild || Clock::now() >= next_classification_;
        classify = pending_classification_ &&
                   (due || barrier_waiters_ > 0);
      }
      if (!classify) {
        condition_.notify_all();
        continue;
      }

      std::shared_ptr<LabelSnapshot> labels;
      bool pose_changed = false;
      bool model_changed_during_classification = false;
      try {
        labels = std::make_shared<LabelSnapshot>();
        labels->keyframe_generation = latest_generation;
        labels->keyframe_count = latest_keyframes.size();
        labels->labels.resize(latest_keyframes.size());
        for (std::size_t index = 0; index < latest_keyframes.size(); ++index) {
          if (stop_.load()) return;
          std::uint64_t current_generation = 0;
          std::uint64_t current_model_epoch = 0;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            current_generation = keyframe_generation_;
            current_model_epoch = model_epoch_;
          }
          if (current_model_epoch != model_epoch) {
            model_changed_during_classification = true;
            break;
          }
          if (current_generation != latest_generation) {
            // A tiny pose revision does not need a native replay; retry the
            // classification with the newer immutable poses.
            pose_changed = true;
            break;
          }
          const auto& keyframe = latest_keyframes[index];
          std::vector<DufomapPoint> corner;
          std::vector<DufomapPoint> surf;
          corner.reserve(keyframe->data->corner.size());
          surf.reserve(keyframe->data->surf.size());
          for (const auto& point : keyframe->data->corner) {
            corner.push_back(TransformPoint(point, keyframe->pose));
          }
          for (const auto& point : keyframe->data->surf) {
            surf.push_back(TransformPoint(point, keyframe->pose));
          }
          labels->labels[index].corner =
              ClassifyWithTemporalGuard(backend.get(), coarse_backend.get(),
                                        support_model_.get(), support_ready_,
                                        corner);
          labels->labels[index].surf =
              ClassifyWithTemporalGuard(backend.get(), coarse_backend.get(),
                                        support_model_.get(), support_ready_,
                                        surf);
        }
      } catch (const std::exception& exception) {
        SetError(std::string("online DUFOMap classification failed: ") +
                 exception.what());
        condition_.notify_all();
        continue;
      }
      if (model_changed_during_classification) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          rebuild_requested_ = true;
        }
        condition_.notify_all();
        continue;
      }
      if (pose_changed) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          classify_requested_ = true;
          pending_classification_ = true;
        }
        condition_.notify_all();
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_.load()) break;
        if (!last_error_.empty()) {
          condition_.notify_all();
          continue;
        }
        if (model_epoch_ != model_epoch) {
          rebuild_requested_ = true;
          condition_.notify_all();
          continue;
        }
        if (latest_generation != keyframe_generation_) {
          // Publication must match the exact pose generation in the snapshot;
          // classify again instead of rebuilding native history.
          classify_requested_ = true;
          pending_classification_ = true;
          condition_.notify_all();
          continue;
        }
        if (config_.temporal_guard.enable &&
            latest_support_revision != support_revision_ &&
            !PrefixStillCurrent(latest_keyframes)) {
          // A complete scan path changed while classification was running.
          // Rebuild support and classify again rather than publish a snapshot
          // whose guard decision used older support.
          classify_requested_ = true;
          pending_classification_ = true;
          condition_.notify_all();
          continue;
        }
        labels->evidence_sequence = integrated_sequence;
        labels->keyframe_count = latest_keyframes.size();
        live_snapshot_ = labels;
        const bool support_current = !config_.temporal_guard.enable ||
                                     latest_support_revision ==
                                         support_revision_;
        // Live may display this internally consistent historical prefix.
        // Barrier/export only receive the complete current support revision.
        if (support_current) snapshot_ = std::move(labels);
        pending_classification_ = !support_current;
        if (!support_current) classify_requested_ = true;
        next_classification_ = Clock::now() + kClassifyInterval;
      }
      condition_.notify_all();
    }
  }

  const DufomapConfig config_;
  const std::filesystem::path cache_directory_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<QueuedEvidence> queue_;
  std::deque<ExportJob> export_jobs_;
  std::unordered_map<std::uint64_t, ExportOutcome> export_outcomes_;
  std::set<std::uint64_t> canceled_exports_;
  std::vector<CachedEvidence> cached_;
  KeyframeList keyframes_;
  // Fixed poses of the last requested meaningful rebuild. Tiny revisions are
  // measured cumulatively against this reference, not against each other.
  std::vector<DufomapPose> rebuild_reference_poses_;
  std::shared_ptr<const LabelSnapshot> snapshot_;
  std::shared_ptr<const LabelSnapshot> live_snapshot_;
  std::uint64_t enqueued_sequence_{0};
  std::uint64_t keyframe_generation_{0};
  std::uint64_t model_epoch_{0};
  // Incremented whenever the complete keyframe scan set or keyframe list
  // changes. Unlike model_epoch, a path attach does not replay native history.
  std::uint64_t support_revision_{0};
  // Worker-owned coarse support. Access is serialized on the worker thread;
  // support_error_ is the only field read under mutex_ from other threads.
  std::unique_ptr<KeyframeSupportModel> support_model_;
  // Complete scan paths already folded into support_model_, in keyframe order.
  // They form the retained prefix used by incremental appends.
  KeyframeList support_keyframes_;
  bool support_ready_{false};
  // Identity of the retained support prefix. A pose generation or model epoch
  // change invalidates the prefix and forces an in-order rebuild.
  std::uint64_t built_support_model_epoch_{0};
  std::uint64_t built_support_pose_generation_{0};
  // Exact last attempted state, successful or failed, so a missing/corrupt
  // complete scan is not re-read on every worker batch.
  bool support_attempted_{false};
  std::uint64_t attempted_support_model_epoch_{0};
  std::uint64_t attempted_support_pose_generation_{0};
  std::uint64_t attempted_support_revision_{0};
  double max_anchor_translation_{0.0};
  std::uint64_t next_export_id_{0};
  std::size_t queued_bytes_{0};
  std::size_t dropped_{0};
  std::size_t skipped_{0};
  std::size_t barrier_waiters_{0};
  bool pending_classification_{false};
  std::chrono::steady_clock::time_point next_classification_{};
  bool rebuild_requested_{false};
  bool classify_requested_{false};
  std::string last_error_;
  // A rejected save request does not invalidate the background map.
  std::string barrier_error_;
  // Support cache diagnostics stay separate from last_error_ so a missing or
  // corrupt complete scan disables coarse-only removal without poisoning the
  // worker. A later SetKeyframeScanPath retries and clears this message.
  std::string support_error_;
  // Saves query the worker serially; a timed-out request never satisfies a
  // later save because outcomes are keyed by a monotonically increasing id.
  std::mutex export_mutex_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

OnlineDufomap::OnlineDufomap(const DufomapConfig& config,
                             std::string cache_directory)
    : impl_(std::make_unique<Impl>(config, std::move(cache_directory))) {}

OnlineDufomap::~OnlineDufomap() = default;

bool OnlineDufomap::Enqueue(const OnlineDufomapEvidence& evidence) {
  return impl_->Enqueue(evidence);
}
void OnlineDufomap::RegisterKeyframe(
    int index, std::vector<DufomapPoint> corner, std::vector<DufomapPoint> surf,
    const DufomapPose& map_T_scan, std::string complete_scan_path) {
  impl_->RegisterKeyframe(index, std::move(corner), std::move(surf),
                          map_T_scan, std::move(complete_scan_path));
}
void OnlineDufomap::SetKeyframeScanPath(int index,
                                        std::string complete_scan_path) {
  impl_->SetKeyframeScanPath(index, std::move(complete_scan_path));
}
void OnlineDufomap::UpdateKeyframePoses(const std::vector<DufomapPose>& poses) {
  impl_->UpdateKeyframePoses(poses);
}
DufomapSnapshotRequest OnlineDufomap::CaptureRequest(
    std::size_t keyframe_count) const {
  return impl_->CaptureRequest(keyframe_count);
}
std::optional<std::shared_ptr<const LabelSnapshot>> OnlineDufomap::Barrier(
    const DufomapSnapshotRequest& request, std::chrono::milliseconds timeout) {
  return impl_->Barrier(request, timeout);
}
std::optional<std::shared_ptr<const LabelSnapshot>> OnlineDufomap::Barrier(
    std::size_t keyframe_count, std::chrono::milliseconds timeout) {
  return impl_->Barrier(keyframe_count, timeout);
}
std::optional<std::shared_ptr<const KeyframeScanExport>>
OnlineDufomap::ExportKeyframeScans(
    const DufomapSnapshotRequest& request,
    const std::vector<int>& keyframe_indices,
    std::chrono::milliseconds timeout) {
  return impl_->ExportKeyframeScans(request, keyframe_indices, timeout);
}
std::shared_ptr<const LabelSnapshot> OnlineDufomap::LatestSnapshot() const {
  return impl_->LatestSnapshot();
}
std::size_t OnlineDufomap::dropped_evidence() const {
  return impl_->dropped_evidence();
}
std::size_t OnlineDufomap::skipped_evidence() const {
  return impl_->skipped_evidence();
}
std::string OnlineDufomap::last_error() const { return impl_->last_error(); }
std::uint64_t OnlineDufomap::keyframe_generation() const {
  return impl_->keyframe_generation();
}

}  // namespace lio_sam_hesai
