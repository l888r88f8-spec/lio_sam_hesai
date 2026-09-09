#include "localization_map/tiled_map_provider.h"

#include <pcl/common/point_tests.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <linux/openat2.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "localization_map/tile_manifest.h"

namespace localization_map {
namespace {

namespace fs = std::filesystem;

std::string TileDescription(const TileIndex& index,
                            const fs::path& relative_path,
                            const char* error_class) {
  std::string result = std::string(error_class) + " tile [" +
                       std::to_string(index.x) + "," +
                       std::to_string(index.y) + "]";
  if (!relative_path.empty()) {
    result += " path=" + relative_path.generic_string();
  }
  return result;
}

enum class Priority : std::size_t {
  kPrefetch = 0U,
  kActive = 1U,
  kCandidate = 2U,
};

enum class OutcomeState { kWaiting, kReady, kNotPresent, kFailed, kBudget };

struct TileOutcome {
  OutcomeState state{OutcomeState::kWaiting};
  bool ground_present{false};
  bool ground_ok{false};
  std::string error;
  std::shared_ptr<const TilePayload> payload;
};

struct Request {
  std::uint64_t id{0U};
  TileIndex center;
  Priority priority{Priority::kPrefetch};
  std::vector<TileIndex> indices;
  std::map<TileIndex, TileOutcome> outcomes;
  std::set<TileIndex> pinned_indices;
  MapRequestResult result;
  bool assembly_queued{false};
  bool building{false};
  bool completed{false};
};

struct InflightLoad {
  Priority priority{Priority::kPrefetch};
  bool loading{false};
  std::set<std::uint64_t> waiters;
};

bool IsTerminal(const Request& request) {
  return std::all_of(
      request.outcomes.begin(), request.outcomes.end(),
      [](const auto& entry) { return entry.second.state != OutcomeState::kWaiting; });
}

bool HasField(const pcl::PCLPointCloud2& cloud, const std::string& name) {
  return std::any_of(cloud.fields.begin(), cloud.fields.end(),
                     [&](const pcl::PCLPointField& field) {
                       return field.name == name;
                     });
}

PCLPointXYZI MakePoint(float x, float y, float z, float intensity) {
  PCLPointXYZI point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  return point;
}

PCLPointCloudXYZI::Ptr LoadPcdAsXYZI(const fs::path& path) {
  pcl::PCLPointCloud2 blob;
  if (pcl::io::loadPCDFile(path.string(), blob) != 0 ||
      !HasField(blob, "x") || !HasField(blob, "y") || !HasField(blob, "z")) {
    return nullptr;
  }

  PCLPointCloudXYZI::Ptr loaded(new PCLPointCloudXYZI);
  if (HasField(blob, "intensity")) {
    pcl::fromPCLPointCloud2(blob, *loaded);
  } else {
    PCLPointCloudXYZ xyz;
    pcl::fromPCLPointCloud2(blob, xyz);
    loaded->reserve(xyz.size());
    for (const auto& point : xyz) {
      loaded->push_back(MakePoint(point.x, point.y, point.z, 0.0F));
    }
    loaded->header = xyz.header;
  }

  PCLPointCloudXYZI::Ptr finite(new PCLPointCloudXYZI);
  finite->header = loaded->header;
  finite->reserve(loaded->size());
  for (const auto& point : *loaded) {
    if (pcl::isFinite(point)) {
      finite->push_back(point);
    }
  }
  finite->width = static_cast<std::uint32_t>(finite->size());
  finite->height = 1U;
  finite->is_dense = true;
  return finite;
}

class UniqueFileDescriptor {
 public:
  UniqueFileDescriptor() = default;
  explicit UniqueFileDescriptor(int descriptor) : descriptor_(descriptor) {}
  ~UniqueFileDescriptor() {
    if (descriptor_ >= 0) {
      ::close(descriptor_);
    }
  }

  UniqueFileDescriptor(const UniqueFileDescriptor&) = delete;
  UniqueFileDescriptor& operator=(const UniqueFileDescriptor&) = delete;
  UniqueFileDescriptor(UniqueFileDescriptor&& other) noexcept
      : descriptor_(std::exchange(other.descriptor_, -1)) {}
  UniqueFileDescriptor& operator=(UniqueFileDescriptor&& other) noexcept {
    if (this != &other) {
      if (descriptor_ >= 0) {
        ::close(descriptor_);
      }
      descriptor_ = std::exchange(other.descriptor_, -1);
    }
    return *this;
  }

  explicit operator bool() const { return descriptor_ >= 0; }
  std::string ProcPath() const {
    return "/proc/self/fd/" + std::to_string(descriptor_);
  }

 private:
  int descriptor_{-1};
};

int OpenRootDirectory(const fs::path& root) {
  std::error_code error;
  const fs::path canonical_root = fs::weakly_canonical(root, error);
  if (error) {
    return -1;
  }
  return ::open(canonical_root.c_str(),
                O_RDONLY | O_CLOEXEC | O_DIRECTORY | O_NOFOLLOW);
}

UniqueFileDescriptor OpenValidatedFile(
    int root_descriptor, const fs::path& relative_path,
    std::optional<std::uintmax_t> expected_bytes) {
  if (root_descriptor < 0 || relative_path.empty() ||
      relative_path.is_absolute()) {
    return {};
  }
  const std::string relative = relative_path.generic_string();
  struct open_how how {};
  how.flags = O_RDONLY | O_CLOEXEC | O_NOFOLLOW;
  how.resolve = RESOLVE_BENEATH | RESOLVE_NO_MAGICLINKS | RESOLVE_NO_SYMLINKS;
  const int descriptor = static_cast<int>(
      ::syscall(SYS_openat2, root_descriptor, relative.c_str(), &how,
                sizeof(how)));
  if (descriptor < 0) {
    return {};
  }
  UniqueFileDescriptor result(descriptor);
  struct stat status {};
  if (::fstat(descriptor, &status) != 0 || !S_ISREG(status.st_mode) ||
      status.st_size < 0) {
    return {};
  }
  if (expected_bytes.has_value() &&
      static_cast<std::uintmax_t>(status.st_size) != *expected_bytes) {
    return {};
  }
  return result;
}

bool ValidVelocity(const std::optional<Vec2d>& velocity) {
  return velocity.has_value() && velocity->allFinite() &&
         velocity->squaredNorm() > std::numeric_limits<double>::epsilon();
}

MapRequestResult Unavailable(const std::string& error,
                             bool previous_covers = false) {
  return {MapRequestState::kUnavailable, nullptr, previous_covers, error};
}

}  // namespace

class TiledMapProvider::Impl {
 public:
  Impl(TiledMapProviderConfig provider_config,
       std::function<TileLoadResult(TileIndex)> injected_loader)
      : config(std::move(provider_config)),
        loader(std::move(injected_loader)),
        cache(TileCacheConfig{config.max_cached_tiles,
                              config.max_cache_bytes}) {
    diagnostics.mode = MapMode::kTiled;
  }

  ~Impl() {
    if (root_descriptor >= 0) {
      ::close(root_descriptor);
    }
  }

  bool ValidateConfig(std::string* error) const {
    const auto fail = [&](const std::string& message) {
      if (error != nullptr) {
        *error = message;
      }
      return false;
    };
    if (config.root.empty()) {
      return fail("tiled map root must be nonempty");
    }
    if (config.active_radius < 0 ||
        config.prefetch_radius < config.active_radius) {
      return fail("tiled map radii are invalid");
    }
    const std::uint64_t width =
        static_cast<std::uint64_t>(2LL * config.active_radius + 1LL);
    if (width > std::numeric_limits<std::uint64_t>::max() / width) {
      return fail("active tile count is not representable");
    }
    const std::uint64_t count = width * width;
    if (count > config.max_cached_tiles || config.max_cache_bytes == 0U) {
      return fail("tile cache cannot hold the active neighborhood");
    }
    if (!std::isfinite(config.registration_voxel_size) ||
        config.registration_voxel_size <= 0.0F) {
      return fail("registration voxel size must be finite and positive");
    }
    return true;
  }

  TileLoadResult LoadProductionTile(TileIndex index) const {
    TileLoadResult result;
    result.index = index;
    const auto global_entry = manifest.global_tiles.find(index);
    result.global_present = global_entry != manifest.global_tiles.end();
    if (!result.global_present) {
      return result;
    }

    UniqueFileDescriptor global_file = OpenValidatedFile(
        root_descriptor, global_entry->second.path, global_entry->second.bytes);
    if (!global_file) {
      result.error = TileDescription(index, global_entry->second.path,
                                     "global file validation failed for");
      return result;
    }
    PCLPointCloudXYZI::Ptr global = LoadPcdAsXYZI(global_file.ProcPath());
    if (!global || global->size() != global_entry->second.point_count) {
      result.error = TileDescription(index, global_entry->second.path,
                                     "global point count mismatch for");
      return result;
    }
    PCLPointCloudXYZI::Ptr filtered(new PCLPointCloudXYZI);
    pcl::VoxelGrid<PCLPointXYZI> voxel_grid;
    voxel_grid.setInputCloud(global);
    voxel_grid.setLeafSize(config.registration_voxel_size,
                           config.registration_voxel_size,
                           config.registration_voxel_size);
    voxel_grid.filter(*filtered);
    if (filtered->empty()) {
      result.error = TileDescription(index, global_entry->second.path,
                                     "global empty after filtering for");
      return result;
    }
    result.payload.global = filtered;
    result.global_ok = true;

    const auto ground_entry = manifest.ground_tiles.find(index);
    result.ground_present = ground_entry != manifest.ground_tiles.end();
    if (result.ground_present) {
      UniqueFileDescriptor ground_file = OpenValidatedFile(
          root_descriptor, ground_entry->second.path,
          ground_entry->second.bytes);
      if (ground_file) {
        PCLPointCloudXYZI::Ptr ground =
            LoadPcdAsXYZI(ground_file.ProcPath());
        if (ground && ground->size() == ground_entry->second.point_count &&
            !ground->empty()) {
          result.payload.ground = ground;
          result.ground_ok = true;
        } else {
          result.error = TileDescription(index, ground_entry->second.path,
                                         "ground point count mismatch for");
        }
      } else {
        result.error = TileDescription(index, ground_entry->second.path,
                                       "ground file validation failed for");
      }
    }

    const std::size_t global_bytes =
        result.payload.global->size() * sizeof(PCLPointXYZI);
    const std::size_t ground_bytes = result.payload.ground
                                         ? result.payload.ground->size() *
                                               sizeof(PCLPointXYZI)
                                         : 0U;
    if (global_bytes > std::numeric_limits<std::size_t>::max() - ground_bytes) {
      result.global_ok = false;
      result.payload = {};
      result.error = TileDescription(index, global_entry->second.path,
                                     "memory size overflow for");
      return result;
    }
    result.payload.bytes = global_bytes + ground_bytes;
    return result;
  }

  std::optional<TileIndex> CenterForPose(const Mat4d& pose,
                                         std::string* error) const {
    if (!pose.allFinite()) {
      if (error != nullptr) {
        *error = "map pose must be finite";
      }
      return std::nullopt;
    }
    try {
      return TileIndexForPosition(pose(0, 3), pose(1, 3),
                                  manifest.tile_size_m, manifest.origin[0],
                                  manifest.origin[1]);
    } catch (const std::exception&) {
      if (error != nullptr) {
        *error = "map pose is outside tiled map index range";
      }
      return std::nullopt;
    }
  }

  bool HasQueuedWorkLocked() const {
    return std::any_of(assembly_queues.begin(), assembly_queues.end(),
                       [](const auto& queue) { return !queue.empty(); }) ||
           std::any_of(queues.begin(), queues.end(),
                       [](const auto& queue) { return !queue.empty(); });
  }

  bool HasRequiredWorkLocked() const {
    if (!queues[static_cast<std::size_t>(Priority::kCandidate)].empty() ||
        !queues[static_cast<std::size_t>(Priority::kActive)].empty()) {
      return true;
    }
    return !assembly_queues[static_cast<std::size_t>(Priority::kCandidate)]
                .empty() ||
           !assembly_queues[static_cast<std::size_t>(Priority::kActive)]
                .empty();
  }

  bool HasOutstandingWorkLocked() const {
    return !loads.empty() ||
           std::any_of(assembly_queues.begin(), assembly_queues.end(),
                       [](const auto& queue) { return !queue.empty(); }) ||
           std::any_of(requests.begin(), requests.end(), [](const auto& entry) {
             return entry.second->building;
           });
  }

  void QueueAssemblyLocked(const std::shared_ptr<Request>& request) {
    if (!request->completed && !request->building &&
        !request->assembly_queued && IsTerminal(*request)) {
      request->assembly_queued = true;
      assembly_queues[static_cast<std::size_t>(request->priority)].push_back(
          request);
      condition.notify_all();
    }
  }

  void QueueIndexLocked(const TileIndex& index, Priority priority,
                        const std::shared_ptr<Request>& request) {
    if (const auto cached = cache.Find(index); cached) {
      if (request) {
        request->outcomes[index] = {
            OutcomeState::kReady,
            !cached->ground,
            static_cast<bool>(cached->ground),
            {},
            cached};
        if (request->priority == Priority::kActive) {
          if (cache.Pin(index)) {
            request->pinned_indices.insert(index);
          } else {
            request->outcomes[index].state = OutcomeState::kBudget;
            request->outcomes[index].error = TileDescription(
                index, {}, "cache could not pin active");
          }
        }
      }
      return;
    }

    auto inflight = loads.find(index);
    if (inflight == loads.end()) {
      InflightLoad state;
      state.priority = priority;
      if (request) {
        state.waiters.insert(request->id);
      }
      loads.emplace(index, std::move(state));
      queues[static_cast<std::size_t>(priority)].push_back(index);
    } else {
      if (request) {
        inflight->second.waiters.insert(request->id);
      }
      if (!inflight->second.loading &&
          static_cast<std::size_t>(priority) >
              static_cast<std::size_t>(inflight->second.priority)) {
        EraseQueuedIndexLocked(index);
        inflight->second.priority = priority;
        queues[static_cast<std::size_t>(priority)].push_back(index);
      }
    }
    condition.notify_all();
  }

  std::shared_ptr<Request> CreateRequestLocked(
      const TileIndex& center, Priority priority) {
    auto request = std::make_shared<Request>();
    request->id = next_request_id++;
    request->center = center;
    request->priority = priority;
    request->indices = SquareNeighborhood(center, config.active_radius);
    for (const TileIndex& index : request->indices) {
      request->outcomes.emplace(index, TileOutcome{});
    }
    requests.emplace(request->id, request);
    for (const TileIndex& index : request->indices) {
      QueueIndexLocked(index, priority, request);
    }
    QueueAssemblyLocked(request);
    return request;
  }

  void EraseQueuedIndexLocked(const TileIndex& index) {
    for (auto& queue : queues) {
      queue.erase(std::remove(queue.begin(), queue.end(), index), queue.end());
    }
  }

  void ReleaseRequestPinsLocked(const std::shared_ptr<Request>& request) {
    for (const TileIndex& index : request->pinned_indices) {
      cache.Unpin(index);
    }
    request->pinned_indices.clear();
  }

  void CancelRequestLocked(std::uint64_t request_id) {
    const auto request = requests.find(request_id);
    if (request == requests.end()) {
      return;
    }
    for (const TileIndex& index : request->second->indices) {
      const auto load = loads.find(index);
      if (load == loads.end()) {
        continue;
      }
      load->second.waiters.erase(request_id);
      if (load->second.waiters.empty() && !load->second.loading) {
        loads.erase(load);
        EraseQueuedIndexLocked(index);
      }
    }
    ReleaseRequestPinsLocked(request->second);
    for (auto& queue : assembly_queues) {
      queue.erase(
          std::remove_if(queue.begin(), queue.end(),
                         [&](const std::shared_ptr<Request>& queued) {
                           return queued->id == request_id;
                         }),
          queue.end());
    }
    requests.erase(request);
  }

  void CancelSupersededTrackingLocked(const TileIndex& retained_center) {
    for (auto tracking = tracking_requests.begin();
         tracking != tracking_requests.end();) {
      if (tracking->first == retained_center) {
        ++tracking;
        continue;
      }
      const std::uint64_t request_id = tracking->second;
      CancelRequestLocked(request_id);
      tracking = tracking_requests.erase(tracking);
    }
  }

  void CancelQueuedPrefetchLocked() {
    for (auto load = loads.begin(); load != loads.end();) {
      if (!load->second.loading && load->second.waiters.empty() &&
          load->second.priority == Priority::kPrefetch) {
        const TileIndex index = load->first;
        load = loads.erase(load);
        EraseQueuedIndexLocked(index);
      } else {
        ++load;
      }
    }
  }

  bool PopLoadLocked(Priority priority, TileIndex* index) {
    auto& queue = queues[static_cast<std::size_t>(priority)];
    while (!queue.empty()) {
      const TileIndex candidate = queue.front();
      queue.pop_front();
      const auto load = loads.find(candidate);
      if (load == loads.end() || load->second.loading ||
          load->second.priority != priority) {
        continue;
      }
      load->second.loading = true;
      *index = candidate;
      return true;
    }
    return false;
  }

  std::shared_ptr<Request> PopAssemblyLocked(Priority priority) {
    auto& queue = assembly_queues[static_cast<std::size_t>(priority)];
    while (!queue.empty()) {
      auto request = queue.front();
      queue.pop_front();
      const auto active = requests.find(request->id);
      if (active == requests.end() || active->second.get() != request.get() ||
          request->completed || request->building || !IsTerminal(*request)) {
        continue;
      }
      request->assembly_queued = false;
      request->building = true;
      return request;
    }
    return nullptr;
  }

  bool PopNextWorkLocked(std::shared_ptr<Request>* assembly,
                         TileIndex* index) {
    for (std::size_t priority_index =
             static_cast<std::size_t>(Priority::kCandidate) + 1U;
         priority_index > 0U; --priority_index) {
      const auto priority = static_cast<Priority>(priority_index - 1U);
      *assembly = PopAssemblyLocked(priority);
      if (*assembly || PopLoadLocked(priority, index)) {
        return true;
      }
    }
    return false;
  }

  void AbortBudgetRequestLocked(const std::shared_ptr<Request>& request) {
    for (auto& entry : request->outcomes) {
      if (entry.second.state != OutcomeState::kWaiting) {
        continue;
      }
      entry.second.state = OutcomeState::kBudget;
      entry.second.error = TileDescription(
          entry.first, {}, "cache budget rejected active");
      const auto load = loads.find(entry.first);
      if (load == loads.end()) {
        continue;
      }
      load->second.waiters.erase(request->id);
      if (load->second.waiters.empty() && !load->second.loading) {
        loads.erase(load);
        EraseQueuedIndexLocked(entry.first);
      }
    }
    QueueAssemblyLocked(request);
  }

  void WorkerLoop() {
    while (true) {
      TileIndex index;
      std::shared_ptr<Request> assembly;
      bool has_load = false;
      {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [&] {
          return stopping || HasQueuedWorkLocked();
        });
        if (stopping && !HasRequiredWorkLocked()) {
          return;
        }
        has_load = PopNextWorkLocked(&assembly, &index) && !assembly;
        if (!assembly && !has_load) {
          continue;
        }
        diagnostics.load_state = "loading";
      }

      if (assembly) {
        MapRequestResult built;
        try {
          built = BuildResult(*assembly);
        } catch (...) {
          built = Unavailable("tile snapshot assembly failed");
        }
        std::lock_guard<std::mutex> lock(mutex);
        const auto active = requests.find(assembly->id);
        if (active != requests.end() &&
            active->second.get() == assembly.get()) {
          assembly->result = std::move(built);
          assembly->building = false;
          assembly->completed = true;
        }
        diagnostics.load_state =
            HasOutstandingWorkLocked() ? "loading" : "ready";
        condition.notify_all();
        continue;
      }

      const auto started = std::chrono::steady_clock::now();
      TileLoadResult result;
      try {
        result = loader(index);
        result.index = index;
      } catch (...) {
        result.index = index;
        result.global_present = true;
        result.error = TileDescription(index, {}, "loader exception for");
      }
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - started)
                                    .count();

      std::lock_guard<std::mutex> lock(mutex);
      const auto load = loads.find(index);
      if (load == loads.end()) {
        continue;
      }

      TileOutcome outcome;
      outcome.ground_present = result.ground_present;
      outcome.ground_ok = result.ground_ok;
      outcome.error = result.error;
      if (!result.global_present) {
        outcome.state = OutcomeState::kNotPresent;
      } else if (!result.global_ok || !result.payload.global ||
                 result.payload.global->empty()) {
        outcome.state = OutcomeState::kFailed;
        ++diagnostics.load_failures;
      } else {
        if (!result.ground_present) {
          result.ground_ok = true;
          result.payload.ground = result.payload.global;
        } else if (!result.ground_ok || !result.payload.ground) {
          result.ground_ok = false;
          result.payload.ground.reset();
        }
        outcome.ground_ok = result.ground_ok;
        bool inserted = false;
        try {
          inserted = cache.Insert(index, std::move(result.payload));
        } catch (...) {
          outcome.state = OutcomeState::kFailed;
          outcome.error = TileDescription(index, {}, "cache insertion failed for");
          ++diagnostics.load_failures;
        }
        if (!inserted && outcome.state != OutcomeState::kFailed) {
          outcome.state = OutcomeState::kBudget;
          outcome.error = TileDescription(index, {}, "cache budget rejected loaded");
          ++diagnostics.load_failures;
        } else if (inserted) {
          outcome.state = OutcomeState::kReady;
          outcome.payload = cache.Find(index);
        }
        if (result.ground_present && !result.ground_ok) {
          ++diagnostics.load_failures;
        }
      }

      std::vector<std::shared_ptr<Request>> affected;
      std::vector<std::shared_ptr<Request>> budget_failures;
      for (const std::uint64_t request_id : load->second.waiters) {
        const auto request = requests.find(request_id);
        if (request != requests.end()) {
          request->second->outcomes[index] = outcome;
          if (request->second->priority == Priority::kActive &&
              outcome.state == OutcomeState::kReady) {
            if (cache.Pin(index)) {
              request->second->pinned_indices.insert(index);
            } else {
              request->second->outcomes[index].state = OutcomeState::kBudget;
              request->second->outcomes[index].error = TileDescription(
                  index, {}, "cache could not pin active");
            }
          }
          affected.push_back(request->second);
          if (request->second->priority == Priority::kActive &&
              request->second->outcomes[index].state ==
                  OutcomeState::kBudget) {
            budget_failures.push_back(request->second);
          }
        }
      }
      loads.erase(load);
      for (const auto& request : budget_failures) {
        AbortBudgetRequestLocked(request);
      }
      for (const auto& request : affected) {
        QueueAssemblyLocked(request);
      }
      diagnostics.last_load_ms = elapsed_ms;
      diagnostics.load_state = HasOutstandingWorkLocked() ? "loading" : "ready";
      if (!outcome.error.empty()) {
        diagnostics.last_error = outcome.error;
      }
      condition.notify_all();
    }
  }

  MapRequestResult BuildResult(const Request& request) const {
    const auto center_outcome = request.outcomes.find(request.center);
    if (center_outcome != request.outcomes.end() &&
        center_outcome->second.state == OutcomeState::kBudget) {
      return Unavailable(TileDescription(
          request.center, {}, "cache budget rejected active"));
    }
    if (center_outcome == request.outcomes.end() ||
        center_outcome->second.state != OutcomeState::kReady) {
      return Unavailable("center global tile [" +
                         std::to_string(request.center.x) + "," +
                         std::to_string(request.center.y) + "] is unavailable");
    }

    PCLPointCloudXYZI::Ptr registration(new PCLPointCloudXYZI);
    PCLPointCloudXYZI::Ptr ground(new PCLPointCloudXYZI);
    auto snapshot = std::make_shared<MapSnapshot>();
    for (const TileIndex& index : request.indices) {
      const TileOutcome& outcome = request.outcomes.at(index);
      if (outcome.state == OutcomeState::kBudget) {
        return Unavailable(TileDescription(
            index, {}, "cache budget rejected active"));
      }
      if (outcome.state != OutcomeState::kReady) {
        snapshot->degraded = true;
        snapshot->missing_indices.push_back(index);
        continue;
      }
      const auto& payload = outcome.payload;
      if (!payload || !payload->global) {
        return Unavailable(TileDescription(
            index, {}, "evicted before snapshot commit"));
      }
      *registration += *payload->global;
      snapshot->indices.push_back(index);
      if (payload->ground) {
        *ground += *payload->ground;
      } else {
        *ground += *payload->global;
        if (outcome.ground_present && !outcome.ground_ok) {
          snapshot->degraded = true;
        }
      }
    }
    snapshot->registration_cloud = registration;
    snapshot->ground_cloud = ground;
    return {MapRequestState::kReady, std::move(snapshot), true, {}};
  }

  bool ActiveContainsLocked(const TileIndex& center) const {
    return current_snapshot &&
           std::binary_search(current_snapshot->indices.begin(),
                              current_snapshot->indices.end(), center);
  }

  bool RestoreCurrentActiveLocked() {
    if (current_pins_valid) {
      return true;
    }
    cache.ClearPins();
    bool restored = true;
    for (const auto& entry : current_payloads) {
      if (!cache.Find(entry.first)) {
        try {
          restored = cache.Insert(entry.first, *entry.second) && restored;
        } catch (...) {
          restored = false;
        }
      }
      restored = cache.Pin(entry.first) && restored;
    }
    if (!restored) {
      diagnostics.last_error = "previous active tile cache restore failed";
    }
    current_pins_valid = restored;
    return restored;
  }

  void EnqueuePrefetchLocked(const TileIndex& center, const Vec2d& position,
                             const std::optional<Vec2d>& velocity) {
    std::vector<TileIndex> prefetch;
    std::vector<TileIndex> active;
    try {
      prefetch = SquareNeighborhood(center, config.prefetch_radius);
      active = SquareNeighborhood(center, config.active_radius);
    } catch (const std::exception&) {
      diagnostics.last_error = "prefetch neighborhood is outside tile range";
      return;
    }
    prefetch.erase(std::remove_if(prefetch.begin(), prefetch.end(),
                                  [&](const TileIndex& index) {
                                    return std::binary_search(
                                        active.begin(), active.end(), index);
                                  }),
                   prefetch.end());
    if (ValidVelocity(velocity)) {
      const Vec2d direction = velocity.value().normalized();
      std::stable_sort(prefetch.begin(), prefetch.end(),
                       [&](const TileIndex& lhs, const TileIndex& rhs) {
        const auto score = [&](const TileIndex& index) {
          const Vec2d tile_center{
              manifest.origin[0] +
                  (static_cast<double>(index.x) + 0.5) * manifest.tile_size_m,
              manifest.origin[1] +
                  (static_cast<double>(index.y) + 0.5) * manifest.tile_size_m};
          return (tile_center - position).dot(direction);
        };
        const double lhs_score = score(lhs);
        const double rhs_score = score(rhs);
        if (lhs_score != rhs_score) {
          return lhs_score > rhs_score;
        }
        return lhs < rhs;
      });
    }
    for (const TileIndex& index : prefetch) {
      QueueIndexLocked(index, Priority::kPrefetch, nullptr);
    }
  }

  const TiledMapProviderConfig config;
  std::function<TileLoadResult(TileIndex)> loader;
  mutable std::mutex mutex;
  std::mutex shutdown_mutex;
  std::condition_variable condition;
  TileManifest manifest;
  TileCache cache;
  std::array<std::deque<TileIndex>, 3> queues;
  std::array<std::deque<std::shared_ptr<Request>>, 3> assembly_queues;
  std::map<TileIndex, InflightLoad> loads;
  std::map<std::uint64_t, std::shared_ptr<Request>> requests;
  std::map<TileIndex, std::uint64_t> tracking_requests;
  std::uint64_t next_request_id{1U};
  std::shared_ptr<const MapSnapshot> current_snapshot;
  std::map<TileIndex, std::shared_ptr<const TilePayload>> current_payloads;
  bool current_pins_valid{false};
  std::optional<TileIndex> current_center;
  PCLPointCloudXYZI::ConstPtr overview;
  MapDiagnostics diagnostics;
  std::thread worker;
  int root_descriptor{-1};
  bool prepared{false};
  bool stopping{false};
};

TiledMapProvider::TiledMapProvider(TiledMapProviderConfig config)
    : impl_(std::make_unique<Impl>(std::move(config), nullptr)) {}

TiledMapProvider::TiledMapProvider(
    TiledMapProviderConfig config,
    std::function<TileLoadResult(TileIndex)> loader)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(loader))) {}

TiledMapProvider::~TiledMapProvider() { Shutdown(); }

bool TiledMapProvider::Prepare(std::string* error) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->prepared) {
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }
  const auto fail = [&](const std::string& message) {
    impl_->diagnostics.load_state = "error";
    impl_->diagnostics.last_error = message;
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };

  std::string config_error;
  if (!impl_->ValidateConfig(&config_error)) {
    return fail(config_error);
  }
  try {
    impl_->manifest = TileManifest::Load(impl_->config.root);
  } catch (const std::exception&) {
    return fail("tiled map manifest is invalid or unavailable");
  }
  impl_->root_descriptor = OpenRootDirectory(impl_->config.root);
  if (impl_->root_descriptor < 0) {
    return fail("tiled map root is unavailable");
  }

  impl_->diagnostics.manifest_version = impl_->manifest.schema_version;
  impl_->diagnostics.load_state = "ready";
  impl_->diagnostics.last_error.clear();
  if (impl_->config.load_overview) {
    if (!impl_->manifest.overview.has_value()) {
      impl_->diagnostics.last_error = "tiled map overview is unavailable";
      impl_->diagnostics.nonfatal_fault_active = true;
    } else {
      try {
        UniqueFileDescriptor overview_file = OpenValidatedFile(
            impl_->root_descriptor, impl_->manifest.overview->path,
            std::nullopt);
        if (overview_file) {
          impl_->overview = LoadPcdAsXYZI(overview_file.ProcPath());
        }
      } catch (...) {
        impl_->overview.reset();
      }
      if (!impl_->overview || impl_->overview->empty()) {
        impl_->overview.reset();
        impl_->diagnostics.last_error = "tiled map overview is unavailable";
        impl_->diagnostics.nonfatal_fault_active = true;
      }
    }
  }
  if (!impl_->loader) {
    impl_->loader = [state = impl_.get()](TileIndex index) {
      return state->LoadProductionTile(index);
    };
  }
  impl_->stopping = false;
  try {
    std::thread worker([state = impl_.get()] { state->WorkerLoop(); });
    impl_->worker = std::move(worker);
  } catch (const std::exception&) {
    if (impl_->root_descriptor >= 0) {
      ::close(impl_->root_descriptor);
      impl_->root_descriptor = -1;
    }
    impl_->stopping = false;
    return fail("tiled map worker could not be started");
  }
  impl_->prepared = true;
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

MapRequestResult TiledMapProvider::LoadCandidate(
    const Mat4d& map_lidar_pose, std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(impl_->mutex);
  if (!impl_->prepared || impl_->stopping) {
    const std::string error = impl_->diagnostics.last_error.empty()
                                  ? "tiled map is not prepared"
                                  : impl_->diagnostics.last_error;
    return Unavailable(error);
  }
  std::string pose_error;
  const auto center = impl_->CenterForPose(map_lidar_pose, &pose_error);
  if (!center.has_value()) {
    return Unavailable(pose_error);
  }

  std::shared_ptr<Request> request;
  try {
    request = impl_->CreateRequestLocked(*center, Priority::kCandidate);
  } catch (const std::exception&) {
    return Unavailable("active tile neighborhood is outside tile range");
  }
  if (!impl_->condition.wait_for(lock, timeout,
                                 [&] { return request->completed; })) {
    impl_->CancelRequestLocked(request->id);
    return Unavailable("candidate tile load timed out");
  }
  MapRequestResult result = request->result;
  impl_->requests.erase(request->id);
  return result;
}

MapRequestResult TiledMapProvider::RequestTracking(
    const Mat4d& map_lidar_pose, std::optional<Vec2d> velocity_xy,
    bool force_rebuild) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->prepared || impl_->stopping) {
    const std::string error = impl_->diagnostics.last_error.empty()
                                  ? "tiled map is not prepared"
                                  : impl_->diagnostics.last_error;
    return Unavailable(error);
  }
  std::string pose_error;
  const auto center = impl_->CenterForPose(map_lidar_pose, &pose_error);
  if (!center.has_value()) {
    return Unavailable(pose_error);
  }
  const bool previous_covers = impl_->ActiveContainsLocked(*center);

  const auto existing = impl_->tracking_requests.find(*center);
  if (existing != impl_->tracking_requests.end()) {
    const auto request = impl_->requests.find(existing->second);
    if (request != impl_->requests.end() && request->second->completed) {
      MapRequestResult result = request->second->result;
      auto completed_request = request->second;
      impl_->requests.erase(request);
      impl_->tracking_requests.erase(existing);
      if (result.state == MapRequestState::kReady) {
        const bool all_pinned = std::all_of(
            result.snapshot->indices.begin(), result.snapshot->indices.end(),
            [&](const TileIndex& index) {
              return completed_request->pinned_indices.count(index) != 0U;
            });
        if (!all_pinned) {
          impl_->ReleaseRequestPinsLocked(completed_request);
          impl_->RestoreCurrentActiveLocked();
          return Unavailable(TileDescription(
              existing->first, {}, "cache could not retain active"));
        }
        std::map<TileIndex, std::shared_ptr<const TilePayload>> new_payloads;
        for (const TileIndex& index : result.snapshot->indices) {
          new_payloads.emplace(
              index, completed_request->outcomes.at(index).payload);
        }
        // Transfer the request-scoped pins to the newly active snapshot.
        completed_request->pinned_indices.clear();
        impl_->current_snapshot = result.snapshot;
        impl_->current_payloads = std::move(new_payloads);
        impl_->current_pins_valid = true;
        impl_->current_center = center;
        impl_->diagnostics.active_tiles = result.snapshot->indices.size();
        impl_->EnqueuePrefetchLocked(
            *center, map_lidar_pose.block<2, 1>(0, 3), velocity_xy);
      } else {
        impl_->ReleaseRequestPinsLocked(completed_request);
        impl_->RestoreCurrentActiveLocked();
      }
      return result;
    }
    return {MapRequestState::kPending, nullptr, previous_covers, {}};
  }

  impl_->CancelSupersededTrackingLocked(*center);
  if (!force_rebuild && impl_->current_center == center &&
      impl_->current_snapshot) {
    impl_->RestoreCurrentActiveLocked();
    return {MapRequestState::kUnchanged, impl_->current_snapshot, true, {}};
  }

  // The immutable merged snapshot retains the old map while its cache entries
  // become evictable, allowing a minimum-capacity cache to load a replacement.
  impl_->CancelQueuedPrefetchLocked();
  impl_->cache.ClearPins();
  impl_->current_pins_valid = false;
  std::shared_ptr<Request> request;
  try {
    request = impl_->CreateRequestLocked(*center, Priority::kActive);
  } catch (const std::exception&) {
    impl_->RestoreCurrentActiveLocked();
    return Unavailable("active tile neighborhood is outside tile range",
                       previous_covers);
  }
  impl_->tracking_requests[*center] = request->id;
  return {MapRequestState::kPending, nullptr, previous_covers, {}};
}

PCLPointCloudXYZI::ConstPtr TiledMapProvider::Overview() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->overview;
}

MapDiagnostics TiledMapProvider::Diagnostics() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  MapDiagnostics result = impl_->diagnostics;
  const TileCacheSnapshot cache = impl_->cache.Snapshot();
  result.cached_tiles = cache.cached_tiles;
  result.cache_mib =
      static_cast<double>(cache.bytes) / (1024.0 * 1024.0);
  result.cache_hits = cache.hits;
  result.cache_misses = cache.misses;
  return result;
}

void TiledMapProvider::Shutdown() {
  std::thread worker;
  {
    std::lock_guard<std::mutex> shutdown_lock(impl_->shutdown_mutex);
    {
      std::lock_guard<std::mutex> lock(impl_->mutex);
      if (!impl_->worker.joinable()) {
        return;
      }
      impl_->stopping = true;
      auto& prefetch =
          impl_->queues[static_cast<std::size_t>(Priority::kPrefetch)];
      prefetch.clear();
      for (auto load = impl_->loads.begin(); load != impl_->loads.end();) {
        if (!load->second.loading &&
            load->second.priority == Priority::kPrefetch) {
          load = impl_->loads.erase(load);
        } else {
          ++load;
        }
      }
      impl_->condition.notify_all();
    }
    worker = std::move(impl_->worker);
  }
  worker.join();
}

}  // namespace localization_map
