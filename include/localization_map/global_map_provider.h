#ifndef LOCALIZATION_MAP__GLOBAL_MAP_PROVIDER_H_
#define LOCALIZATION_MAP__GLOBAL_MAP_PROVIDER_H_

#include <filesystem>
#include <memory>
#include <optional>

#include "localization_map/map_provider.h"

namespace localization_map {

struct GlobalMapProviderConfig {
  std::filesystem::path global_map_path;
  std::optional<std::filesystem::path> ground_map_path;
  float global_voxel_size{0.3F};
};

class GlobalMapProvider final : public MapProvider {
 public:
  explicit GlobalMapProvider(GlobalMapProviderConfig config);
  ~GlobalMapProvider() override;

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

#endif  // LOCALIZATION_MAP__GLOBAL_MAP_PROVIDER_H_
