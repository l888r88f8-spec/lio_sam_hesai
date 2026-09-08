#ifndef LOCALIZATION_MAP__TILED_MAP_PROVIDER_H_
#define LOCALIZATION_MAP__TILED_MAP_PROVIDER_H_

#include <cstddef>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>

#include "localization_map/map_provider.h"
#include "localization_map/tile_cache.h"

namespace localization_map {

struct TiledMapProviderConfig {
  std::filesystem::path root;
  int active_radius{1};
  int prefetch_radius{2};
  std::size_t max_cached_tiles{25};
  std::size_t max_cache_bytes{1024U * 1024U * 1024U};
  bool load_overview{false};
  float registration_voxel_size{0.25F};
};

struct TileLoadResult {
  TileIndex index;
  TilePayload payload;
  bool global_present{false};
  bool global_ok{false};
  bool ground_present{false};
  bool ground_ok{false};
  std::string error;
};

class TiledMapProvider final : public MapProvider {
 public:
  explicit TiledMapProvider(TiledMapProviderConfig config);
  TiledMapProvider(TiledMapProviderConfig config,
                   std::function<TileLoadResult(TileIndex)> loader);
  ~TiledMapProvider() override;

  bool Prepare(std::string* error) override;
  MapRequestResult LoadCandidate(
      const Mat4d& map_lidar_pose,
      std::chrono::milliseconds timeout) override;
  MapRequestResult RequestTracking(
      const Mat4d& map_lidar_pose, std::optional<Vec2d> velocity_xy,
      bool force_rebuild) override;
  PCLPointCloudXYZI::ConstPtr Overview() const override;
  MapDiagnostics Diagnostics() const override;
  void Shutdown() override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace localization_map

#endif  // LOCALIZATION_MAP__TILED_MAP_PROVIDER_H_
