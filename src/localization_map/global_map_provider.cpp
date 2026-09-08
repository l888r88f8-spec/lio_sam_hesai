#include "localization_map/global_map_provider.h"

#include <pcl/conversions.h>
#include <pcl/common/point_tests.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace localization_map {
namespace {

constexpr double kCandidateHalfExtent = 50.0;
constexpr double kTrackingHalfExtent = 100.0;
constexpr double kTrackingRefreshDistance = 50.0;

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

PCLPointCloudXYZI::Ptr LoadPcdAsXYZI(const std::filesystem::path& path) {
  if (path.empty()) {
    return nullptr;
  }

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
    for (const auto& source : xyz) {
      loaded->push_back(MakePoint(source.x, source.y, source.z, 0.0F));
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

PCLPointCloudXYZI::Ptr Flatten(const PCLPointCloudXYZI::ConstPtr& source) {
  PCLPointCloudXYZI::Ptr flattened(new PCLPointCloudXYZI(*source));
  for (auto& point : *flattened) {
    point.z = 0.0F;
  }
  return flattened;
}

pcl::KdTreeFLANN<PCLPointXYZI>::Ptr BuildTree(
    const PCLPointCloudXYZI::Ptr& flattened) {
  pcl::KdTreeFLANN<PCLPointXYZI>::Ptr tree(
      new pcl::KdTreeFLANN<PCLPointXYZI>());
  tree->setInputCloud(flattened);
  return tree;
}

PCLPointCloudXYZI::ConstPtr CandidateCrop(
    const PCLPointCloudXYZI::ConstPtr& source,
    const pcl::KdTreeFLANN<PCLPointXYZI>::Ptr& xy_tree,
    const Vec3d& center) {
  PCLPointCloudXYZI::Ptr cropped(new PCLPointCloudXYZI);
  PCLPointXYZI query;
  query.x = static_cast<float>(center.x());
  query.y = static_cast<float>(center.y());
  query.z = 0.0F;
  query.intensity = 0.0F;

  std::vector<int> indices;
  std::vector<float> squared_distances;
  const double radius = std::sqrt(2.0) * kCandidateHalfExtent;
  const double radius_tolerance =
      radius * static_cast<double>(std::numeric_limits<float>::epsilon());
  xy_tree->radiusSearch(query, radius + radius_tolerance, indices,
                        squared_distances);
  cropped->reserve(indices.size());
  for (const int index : indices) {
    if (index < 0 || index >= static_cast<int>(source->size())) {
      continue;
    }
    const auto& point = source->points[static_cast<std::size_t>(index)];
    if (std::fabs(static_cast<double>(point.x) - center.x()) <=
            kCandidateHalfExtent &&
        std::fabs(static_cast<double>(point.y) - center.y()) <=
            kCandidateHalfExtent &&
        std::fabs(static_cast<double>(point.z) - center.z()) <=
            kCandidateHalfExtent) {
      cropped->push_back(point);
    }
  }
  return cropped;
}

PCLPointCloudXYZI::ConstPtr TrackingCrop(
    const PCLPointCloudXYZI::ConstPtr& source, const Vec3d& center) {
  PCLPointCloudXYZI::Ptr cropped(new PCLPointCloudXYZI);
  pcl::CropBox<PCLPointXYZI> crop;
  crop.setInputCloud(source);
  crop.setMin(Vec4d(center.x() - kTrackingHalfExtent,
                    center.y() - kTrackingHalfExtent,
                    center.z() - kTrackingHalfExtent, 1.0)
                  .cast<float>());
  crop.setMax(Vec4d(center.x() + kTrackingHalfExtent,
                    center.y() + kTrackingHalfExtent,
                    center.z() + kTrackingHalfExtent, 1.0)
                  .cast<float>());
  crop.filter(*cropped);
  return cropped;
}

bool Covers(const std::optional<Vec3d>& snapshot_center, const Vec3d& center) {
  return snapshot_center.has_value() &&
         (snapshot_center.value() - center).cwiseAbs().maxCoeff() <=
             kTrackingHalfExtent;
}

bool TranslationSupportsFloatMapOperations(const Mat4d& pose) {
  const double float_lowest =
      static_cast<double>(std::numeric_limits<float>::lowest());
  const double float_max =
      static_cast<double>(std::numeric_limits<float>::max());
  const double safe_lowest = std::nextafter(float_lowest, 0.0);
  const double safe_max = std::nextafter(float_max, 0.0);
  for (int axis = 0; axis < 3; ++axis) {
    const double center = pose(axis, 3);
    if (center < safe_lowest || center > safe_max) {
      return false;
    }
    const double minimum = center - kTrackingHalfExtent;
    const double maximum = center + kTrackingHalfExtent;
    if (!std::isfinite(minimum) || !std::isfinite(maximum) ||
        minimum < float_lowest || maximum > float_max) {
      return false;
    }
  }
  return true;
}

MapRequestResult Unavailable(const std::string& error) {
  return {MapRequestState::kUnavailable, nullptr, false, error};
}

}  // namespace

class GlobalMapProvider::Impl {
 public:
  explicit Impl(GlobalMapProviderConfig provider_config)
      : config(std::move(provider_config)) {}

  void ResetMapState() {
    prepared = false;
    global.reset();
    ground.reset();
    global_xy.reset();
    ground_xy.reset();
    global_tree.reset();
    ground_tree.reset();
    current_snapshot.reset();
    tracking_center.reset();
  }

  void Fail(const std::string& error) {
    ResetMapState();
    diagnostics.load_state = "error";
    diagnostics.last_error = error;
  }

  const GlobalMapProviderConfig config;
  mutable std::mutex mutex;
  bool prepared{false};
  PCLPointCloudXYZI::ConstPtr global;
  PCLPointCloudXYZI::ConstPtr ground;
  PCLPointCloudXYZI::Ptr global_xy;
  PCLPointCloudXYZI::Ptr ground_xy;
  pcl::KdTreeFLANN<PCLPointXYZI>::Ptr global_tree;
  pcl::KdTreeFLANN<PCLPointXYZI>::Ptr ground_tree;
  std::shared_ptr<const MapSnapshot> current_snapshot;
  std::optional<Vec3d> tracking_center;
  MapDiagnostics diagnostics;
};

GlobalMapProvider::GlobalMapProvider(GlobalMapProviderConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

GlobalMapProvider::~GlobalMapProvider() = default;

bool GlobalMapProvider::Prepare(std::string* error) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->prepared) {
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  const auto fail = [&](const std::string& message) {
    impl_->Fail(message);
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };

  if (!std::isfinite(impl_->config.global_voxel_size) ||
      impl_->config.global_voxel_size <= 0.0F) {
    return fail("global voxel size must be finite and positive");
  }

  PCLPointCloudXYZI::Ptr loaded_global;
  try {
    loaded_global = LoadPcdAsXYZI(impl_->config.global_map_path);
  } catch (const std::exception& exception) {
    return fail("failed to load global map: " + std::string(exception.what()));
  }
  if (!loaded_global || loaded_global->empty()) {
    return fail("global map is missing, invalid, or empty: " +
                impl_->config.global_map_path.string());
  }

  PCLPointCloudXYZI::Ptr voxelized(new PCLPointCloudXYZI);
  pcl::VoxelGrid<PCLPointXYZI> voxel_grid;
  voxel_grid.setInputCloud(loaded_global);
  voxel_grid.setLeafSize(impl_->config.global_voxel_size,
                         impl_->config.global_voxel_size,
                         impl_->config.global_voxel_size);
  voxel_grid.filter(*voxelized);
  if (voxelized->empty()) {
    return fail("global map is empty after voxel filtering: " +
                impl_->config.global_map_path.string());
  }

  PCLPointCloudXYZI::Ptr loaded_ground;
  if (impl_->config.ground_map_path.has_value()) {
    try {
      loaded_ground = LoadPcdAsXYZI(*impl_->config.ground_map_path);
    } catch (const std::exception&) {
      loaded_ground.reset();
    }
  }
  if (!loaded_ground || loaded_ground->empty()) {
    loaded_ground = voxelized;
  }

  impl_->global = voxelized;
  impl_->ground = loaded_ground;
  impl_->global_xy = Flatten(impl_->global);
  impl_->ground_xy = Flatten(impl_->ground);
  impl_->global_tree = BuildTree(impl_->global_xy);
  impl_->ground_tree = BuildTree(impl_->ground_xy);
  impl_->current_snapshot.reset();
  impl_->tracking_center.reset();
  impl_->prepared = true;
  impl_->diagnostics = MapDiagnostics{};
  impl_->diagnostics.mode = MapMode::kGlobal;
  impl_->diagnostics.load_state = "ready";
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

MapRequestResult GlobalMapProvider::LoadCandidate(
    const Mat4d& map_lidar_pose, std::chrono::milliseconds timeout) {
  static_cast<void>(timeout);
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->prepared) {
    const std::string error = impl_->diagnostics.last_error.empty()
                                  ? "global map is not prepared"
                                  : impl_->diagnostics.last_error;
    return Unavailable(error);
  }
  if (!map_lidar_pose.allFinite()) {
    return Unavailable("map pose must be finite");
  }
  if (!TranslationSupportsFloatMapOperations(map_lidar_pose)) {
    return Unavailable("map pose translation exceeds supported float range");
  }

  const Vec3d center = map_lidar_pose.block<3, 1>(0, 3);
  auto snapshot = std::make_shared<MapSnapshot>();
  snapshot->registration_cloud =
      CandidateCrop(impl_->global, impl_->global_tree, center);
  snapshot->ground_cloud =
      CandidateCrop(impl_->ground, impl_->ground_tree, center);
  return {MapRequestState::kReady, std::move(snapshot), false, {}};
}

MapRequestResult GlobalMapProvider::RequestTracking(
    const Mat4d& map_lidar_pose, std::optional<Vec2d> velocity_xy,
    bool force_rebuild) {
  static_cast<void>(velocity_xy);
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->prepared) {
    const std::string error = impl_->diagnostics.last_error.empty()
                                  ? "global map is not prepared"
                                  : impl_->diagnostics.last_error;
    return Unavailable(error);
  }
  if (!map_lidar_pose.allFinite()) {
    return Unavailable("map pose must be finite");
  }
  if (!TranslationSupportsFloatMapOperations(map_lidar_pose)) {
    return Unavailable("map pose translation exceeds supported float range");
  }

  const Vec3d center = map_lidar_pose.block<3, 1>(0, 3);
  const bool previous_covers = Covers(impl_->tracking_center, center);
  if (!force_rebuild && impl_->current_snapshot &&
      impl_->tracking_center.has_value() &&
      (impl_->tracking_center.value() - center).cwiseAbs().maxCoeff() <
          kTrackingRefreshDistance) {
    return {MapRequestState::kUnchanged, impl_->current_snapshot,
            previous_covers, {}};
  }

  auto snapshot = std::make_shared<MapSnapshot>();
  snapshot->registration_cloud = TrackingCrop(impl_->global, center);
  snapshot->ground_cloud = TrackingCrop(impl_->ground, center);
  impl_->current_snapshot = snapshot;
  impl_->tracking_center = center;
  return {MapRequestState::kReady, std::move(snapshot), previous_covers, {}};
}

PCLPointCloudXYZI::ConstPtr GlobalMapProvider::Overview() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->global;
}

MapDiagnostics GlobalMapProvider::Diagnostics() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->diagnostics;
}

void GlobalMapProvider::Shutdown() {}

}  // namespace localization_map
