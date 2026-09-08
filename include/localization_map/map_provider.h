#ifndef LOCALIZATION_MAP__MAP_PROVIDER_H_
#define LOCALIZATION_MAP__MAP_PROVIDER_H_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "common/data_type.h"
#include "localization_map/tile_index.h"

namespace localization_map {

enum class MapMode { kGlobal, kTiled };

enum class MapRequestState { kReady, kUnchanged, kPending, kUnavailable };

struct MapSnapshot {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PCLPointCloudXYZI::ConstPtr registration_cloud;
  PCLPointCloudXYZI::ConstPtr ground_cloud;
  std::vector<TileIndex> indices;
  std::vector<TileIndex> missing_indices;
  bool degraded{false};
};

struct MapRequestResult {
  MapRequestState state{MapRequestState::kUnavailable};
  std::shared_ptr<const MapSnapshot> snapshot;
  bool previous_snapshot_covers_center{false};
  std::string error;
};

struct MapDiagnostics {
  MapMode mode{MapMode::kGlobal};
  int manifest_version{0};
  std::size_t active_tiles{0};
  std::size_t cached_tiles{0};
  double cache_mib{0.0};
  std::uint64_t cache_hits{0};
  std::uint64_t cache_misses{0};
  std::uint64_t load_failures{0};
  double last_load_ms{0.0};
  std::string load_state{"idle"};
  std::string last_error;
};

class MapProvider {
 public:
  virtual ~MapProvider() = default;

  virtual bool Prepare(std::string* error) = 0;
  virtual MapRequestResult LoadCandidate(
      const Mat4d& map_lidar_pose, std::chrono::milliseconds timeout) = 0;
  virtual MapRequestResult RequestTracking(
      const Mat4d& map_lidar_pose, std::optional<Vec2d> velocity_xy,
      bool force_rebuild) = 0;
  virtual PCLPointCloudXYZI::ConstPtr Overview() const = 0;
  virtual MapDiagnostics Diagnostics() const = 0;
  virtual void Shutdown() = 0;
};

}  // namespace localization_map

#endif  // LOCALIZATION_MAP__MAP_PROVIDER_H_
