#ifndef LIO_SAM_HESAI_LIO_SAM_ONLINE_DUFOMAP_H
#define LIO_SAM_HESAI_LIO_SAM_ONLINE_DUFOMAP_H

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common/dufomap_config.h"
#include "lio_sam/dufomap_backend.h"

namespace lio_sam_hesai {

// Map/lidar pose in the same row-major homogeneous matrix form used by the
// backend. Translation is in elements 3, 7, 11.
using DufomapPose = std::array<double, 16>;

struct KeyframeLabels {
  std::vector<std::uint8_t> corner;
  std::vector<std::uint8_t> surf;
};

// A barrier request captured atomically with the caller's keyframe poses.
// evidence_sequence covers every scan accepted before the capture, including
// a batch already in flight in the worker.
struct DufomapSnapshotRequest {
  std::size_t keyframe_count{0};
  std::uint64_t keyframe_generation{0};
  std::uint64_t evidence_sequence{0};
};

// Immutable classification result. 1 means seen-free/dynamic, 0 means keep.
// evidence_sequence is the last integrated, propagated, and classified scan.
struct LabelSnapshot {
  std::uint64_t keyframe_generation{0};
  std::uint64_t evidence_sequence{0};
  std::size_t keyframe_count{0};
  std::vector<KeyframeLabels> labels;
};

// Result of one complete-keyframe-scan export. points are in the final map
// frame and already contain only points DUFOMap kept (label 0).
// evidence_sequence is the worker's actual integrated sequence at export, so
// it is never older than the captured request. feature_labels covers
// request.keyframe_count keyframes and was produced against the same backend
// evidence and pose generation as points. The worker reuses its current
// snapshot only when the evidence sequence, generation, and keyframe coverage
// match exactly; otherwise it classifies the captured corner/surface clouds
// itself.
struct KeyframeScanExport {
  std::uint64_t keyframe_generation{0};
  std::uint64_t evidence_sequence{0};
  std::size_t keyframe_count{0};
  std::shared_ptr<const LabelSnapshot> feature_labels;
  std::vector<DufomapPoint> points;
};

// Keeps points whose label is 0. Input and output order is preserved, so the
// caller can use the same helper for RViz and save. A missing label snapshot
// means "no online DUFOMap": return the original cloud unchanged. A label
// vector whose length differs from the point vector is rejected.
std::vector<DufomapPoint> ApplyDufomapLabels(
    const std::vector<DufomapPoint>& points,
    const std::vector<std::uint8_t>* labels);

// One complete deskewed scan. The world pose is rebuilt as
// keyframe_pose(anchor_keyframe) * anchor_T_scan, so a loop correction that
// moves the anchor also moves all evidence attached to it. If there is no
// reliable keyframe anchor, fallback_map_T_scan is used instead.
struct OnlineDufomapEvidence {
  std::shared_ptr<const sensor_msgs::msg::PointCloud2> cloud;
  int anchor_keyframe{-1};
  DufomapPose anchor_T_scan{};
  DufomapPose fallback_map_T_scan{};
};

class OnlineDufomap {
 public:
  OnlineDufomap(const DufomapConfig& config, std::string cache_directory);
  ~OnlineDufomap();

  OnlineDufomap(const OnlineDufomap&) = delete;
  OnlineDufomap& operator=(const OnlineDufomap&) = delete;

  // Never blocks. A full queue drops its oldest scan and increments the
  // dropped counter. Returns false for malformed input.
  bool Enqueue(const OnlineDufomapEvidence& evidence);

  // Immutable target clouds are copied; original keyframe clouds are never
  // modified. A non-sequential index or invalid pose is rejected through
  // last_error().
  void RegisterKeyframe(int index, std::vector<DufomapPoint> corner,
                        std::vector<DufomapPoint> surf,
                        const DufomapPose& map_T_scan,
                        std::string complete_scan_path = std::string());

  // Attaches a persisted complete keyframe scan to an already registered
  // keyframe. This changes no pose and does not advance the generation.
  void SetKeyframeScanPath(int index, std::string complete_scan_path);

  // Publishes a new pose generation and requests fresh classification. Native
  // history is replayed only when a pose moved beyond the bounded motion
  // tolerance since the last meaningful rebuild; smaller numeric corrections
  // reuse history and only reclassify. The pose count and all poses must be
  // valid; an identical pose array does not advance the generation.
  void UpdateKeyframePoses(const std::vector<DufomapPose>& poses);

  // Atomically captures the current keyframe generation and the last accepted
  // evidence sequence, including evidence already handed to the worker.
  DufomapSnapshotRequest CaptureRequest(std::size_t keyframe_count) const;

  // Waits for exactly the captured generation, evidence at least as new as the
  // request, and labels covering the requested keyframes. Fails explicitly on
  // a worker error or if the generation advances while waiting.
  std::optional<std::shared_ptr<const LabelSnapshot>> Barrier(
      const DufomapSnapshotRequest& request, std::chrono::milliseconds timeout);

  // Convenience overload: captures a request immediately, then waits.
  std::optional<std::shared_ptr<const LabelSnapshot>> Barrier(
      std::size_t keyframe_count, std::chrono::milliseconds timeout);

  // Waits for the captured barrier, then runs one worker-owned batch export of
  // complete keyframe scans. Indices must be strictly increasing and covered by
  // request.keyframe_count. Classification uses the exact generation pose.
  // A generation change, missing/corrupt cache, timeout, or shutdown fails only
  // this request; live classification keeps running.
  std::optional<std::shared_ptr<const KeyframeScanExport>> ExportKeyframeScans(
      const DufomapSnapshotRequest& request,
      const std::vector<int>& keyframe_indices,
      std::chrono::milliseconds timeout);

  std::shared_ptr<const LabelSnapshot> LatestSnapshot() const;
  std::size_t dropped_evidence() const;
  std::size_t skipped_evidence() const;
  std::string last_error() const;
  std::uint64_t keyframe_generation() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lio_sam_hesai

#endif  // LIO_SAM_HESAI_LIO_SAM_ONLINE_DUFOMAP_H
