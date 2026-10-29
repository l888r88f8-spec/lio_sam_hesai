// End-to-end save-map contract tests for the online DUFOMap integration.
//
// The node class is defined in src/mapOptimization.cpp and has no header, so
// this test includes that translation unit directly; its main() is renamed so
// it cannot collide with the gtest entry point.
//
// Ordinary export fixtures explicitly disable dufomap.enable. Online fixtures
// construct one real worker snapshot by registering the seeded keyframe
// targets, enqueueing evidence, and waiting on CaptureRequest + Barrier. No
// test passes just because a source string exists.

#define main lio_samMapOptimizationNodeMain
#include "../src/mapOptimization.cpp"
#undef main

#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace {

namespace fs = std::filesystem;

constexpr std::size_t kSeededGroundPoints = 16;

fs::path UniqueTempRoot() {
  static std::atomic<unsigned> counter{0};
  return fs::temp_directory_path() /
         ("lio_sam_map_optimization_save_export_" +
          std::to_string(::getpid()) + "_" +
          std::to_string(counter.fetch_add(1)));
}

class TempDirectory {
 public:
  TempDirectory() : path_(UniqueTempRoot()) {
    std::error_code error;
    fs::remove_all(path_, error);
    fs::create_directories(path_);
  }

  ~TempDirectory() {
    std::error_code error;
    fs::remove_all(path_, error);
  }

  TempDirectory(const TempDirectory&) = delete;
  TempDirectory& operator=(const TempDirectory&) = delete;

  [[nodiscard]] const fs::path& path() const { return path_; }

 private:
  fs::path path_;
};

void EnsureRosInitialized() {
  static const bool initialized = []() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    return true;
  }();
  (void)initialized;
}

using Cloud = pcl::PointCloud<PointType>;
using CloudPtr = Cloud::Ptr;

lio_sam_hesai::DufomapPose IdentityPose() {
  return {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
}

CloudPtr MakePlaneCloud(float z) {
  CloudPtr cloud(new Cloud());
  for (int x = 0; x < 4; ++x) {
    for (int y = 0; y < 4; ++y) {
      PointType point;
      point.x = 0.5F * static_cast<float>(x);
      point.y = 0.5F * static_cast<float>(y);
      point.z = z;
      point.intensity = 0.0F;
      cloud->push_back(point);
    }
  }
  return cloud;
}

CloudPtr MakePointCloud(const std::vector<std::array<float, 3>>& coordinates) {
  CloudPtr cloud(new Cloud());
  for (const auto& coordinate : coordinates) {
    PointType point;
    point.x = coordinate[0];
    point.y = coordinate[1];
    point.z = coordinate[2];
    point.intensity = 1.0F;
    cloud->push_back(point);
  }
  return cloud;
}

// A dense wall whose rays pass through the y/z origin at x=10.
CloudPtr DenseWall(float x) {
  CloudPtr cloud(new Cloud());
  for (float y = -1.0F; y <= 1.0F; y += 0.1F) {
    for (float z = -0.5F; z <= 0.5F; z += 0.1F) {
      PointType point;
      point.x = x;
      point.y = y;
      point.z = z;
      point.intensity = 1.0F;
      cloud->push_back(point);
    }
  }
  return cloud;
}

CloudPtr ShiftY(const CloudPtr& cloud, float offset) {
  CloudPtr shifted(new Cloud(*cloud));
  for (auto& point : shifted->points) point.y += offset;
  return shifted;
}

void SeedSingleKeyFrame(mapOptimization& node) {
  PointType pose3d;
  pose3d.x = 0.0F;
  pose3d.y = 0.0F;
  pose3d.z = 0.0F;
  pose3d.intensity = 0.0F;
  node.cloudKeyPoses3D->push_back(pose3d);

  PointTypePose pose6d = PointTypePose();
  pose6d.x = 0.0F;
  pose6d.y = 0.0F;
  pose6d.z = 0.0F;
  pose6d.roll = 0.0F;
  pose6d.pitch = 0.0F;
  pose6d.yaw = 0.0F;
  pose6d.time = 0.0;
  node.cloudKeyPoses6D->push_back(pose6d);

  node.cornerCloudKeyFrames.push_back(MakePlaneCloud(1.0F));
  node.surfCloudKeyFrames.push_back(MakePlaneCloud(0.5F));
  node.groundCloudKeyFrames.push_back(MakePlaneCloud(0.0F));
}

struct ExportNodeSpec {
  bool dufomap_enable{false};
  int dufomap_dp{2};
  bool legacy_filter{false};
  bool grid_patches{false};
  bool ground_grid{false};
  bool ground_mesh{false};
  std::optional<int> ground_grid_max_hole_cells;
  std::optional<double> ground_resolution;
  bool flatten{false};
  bool align_ground_to_zero{false};
  double ground_z_offset{0.68};
  std::string current_mode{"off"};
  std::string patchwork_mode{"off"};
  int scan_lines{64};
  int horizon_scan{512};
};

std::unique_ptr<mapOptimization> MakeExportNode(const fs::path& cache,
                                                const ExportNodeSpec& spec) {
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);
  std::vector<rclcpp::Parameter> overrides = {
      rclcpp::Parameter("dufomap.enable", spec.dufomap_enable),
      rclcpp::Parameter("dufomap.d_p", spec.dufomap_dp),
      rclcpp::Parameter("dufomap.cache_directory", cache.string()),
      rclcpp::Parameter("map_save.generate_scan_context_db", false),
      rclcpp::Parameter("map_save.align_ground_to_zero", spec.align_ground_to_zero),
      rclcpp::Parameter("map_save.ground_z_offset", spec.ground_z_offset),
      rclcpp::Parameter("map_save.ground_grid.enable", spec.ground_grid),
      rclcpp::Parameter("map_save.ground_mesh.enable", spec.ground_mesh),
      rclcpp::Parameter("lidar.scan_lines", spec.scan_lines),
      rclcpp::Parameter("lidar.horizon_scan", spec.horizon_scan),
      rclcpp::Parameter("ring_plane.mode", spec.current_mode),
      rclcpp::Parameter("ring_plane.scan_start_index", 0),
      rclcpp::Parameter("ring_plane.scan_end_index", 1),
      rclcpp::Parameter("ring_plane.plane_min_points", 3),
      rclcpp::Parameter("ring_plane.flatten_z", spec.flatten),
      rclcpp::Parameter("ring_plane.flatten_value", 0.1),
      rclcpp::Parameter("ring_plane.grid_patch.enable", spec.grid_patches),
      rclcpp::Parameter("patchworkpp.mode", spec.patchwork_mode),
      rclcpp::Parameter("patchworkpp.sensor_height", 1.723),
      rclcpp::Parameter("static_map_filter.enable", spec.legacy_filter),
  };
  if (spec.ground_grid_max_hole_cells.has_value()) {
    overrides.push_back(rclcpp::Parameter(
        "map_save.ground_grid.max_hole_cells",
        *spec.ground_grid_max_hole_cells));
  }
  if (spec.ground_resolution.has_value()) {
    overrides.push_back(rclcpp::Parameter("map_save.ground_resolution",
                                          *spec.ground_resolution));
  }
  options.parameter_overrides(overrides);
  auto node = std::make_unique<mapOptimization>(options);
  SeedSingleKeyFrame(*node);
  return node;
}

// Ordinary export fixture: online DUFOMap is explicitly off so these tests
// never block on a worker that has no evidence.
std::unique_ptr<mapOptimization> MakeBasicExportNode() {
  return MakeExportNode(UniqueTempRoot() / "cache", ExportNodeSpec{});
}

fs::path GroundArtifact(const fs::path& destination) {
  return MapArtifactLayout(destination).GroundMap();
}

MapExportOutcome ExportThroughOptionsService(mapOptimization& node,
                                             const fs::path& destination,
                                             bool save_ground_map,
                                             bool save_tiled_map = false,
                                             float ground_resolution = 0.0F) {
  std::string error;
  const auto options = lio_sam_hesai::MakeSaveMapExportOptions(
      destination.string(), 0.0F, ground_resolution, save_ground_map,
      save_tiled_map, "/home/robot", &error);
  if (!options.has_value()) {
    ADD_FAILURE() << error;
    return MapExportOutcome{};
  }
  return node.ExportMap(*options);
}

void CacheSeed(mapOptimization& node) {
  std::string error;
  ASSERT_TRUE(node.StoreFullScanForExport(0, node.surfCloudKeyFrames[0], &error))
      << error;
}

std::vector<lio_sam_hesai::DufomapPoint> ToDufomapPoints(
    const CloudPtr& cloud) {
  std::vector<lio_sam_hesai::DufomapPoint> points;
  points.reserve(cloud->size());
  for (const auto& point : cloud->points) {
    points.push_back(lio_sam_hesai::DufomapPoint{
        point.x, point.y, point.z, point.intensity});
  }
  return points;
}

bool CacheCompleteScan(mapOptimization& node, std::size_t index,
                       const CloudPtr& cloud, std::string* error);

// Registers the current keyframe targets, enqueues the supplied evidence
// clouds, and waits for one real worker snapshot covering keyframe 0.
std::shared_ptr<const lio_sam_hesai::LabelSnapshot> SeedOnlineSnapshot(
    mapOptimization& node, const std::vector<CloudPtr>& evidence_clouds,
    bool cache_complete_scan = true) {
  if (!node.online_dufomap_) return nullptr;
  auto& worker = *node.online_dufomap_;
  worker.RegisterKeyframe(0, ToDufomapPoints(node.cornerCloudKeyFrames[0]),
                          ToDufomapPoints(node.surfCloudKeyFrames[0]),
                          IdentityPose());
  if (cache_complete_scan) {
    // Default fixture: every synthetic return is also a feature. Tests of
    // complete-scan detail replace this with an independently built scan.
    CloudPtr complete(new Cloud());
    *complete += *node.cornerCloudKeyFrames[0];
    *complete += *node.surfCloudKeyFrames[0];
    std::string error;
    if (!CacheCompleteScan(node, 0, complete, &error)) {
      ADD_FAILURE() << error;
      return nullptr;
    }
  }
  for (const auto& cloud : evidence_clouds) {
    auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud, *message);
    lio_sam_hesai::OnlineDufomapEvidence evidence;
    evidence.cloud = std::move(message);
    evidence.anchor_keyframe = 0;
    evidence.anchor_T_scan = IdentityPose();
    evidence.fallback_map_T_scan = IdentityPose();
    if (!worker.Enqueue(evidence)) {
      ADD_FAILURE() << "online DUFOMap rejected fixture evidence";
      return nullptr;
    }
  }
  const auto request = worker.CaptureRequest(1);
  const auto snapshot = worker.Barrier(request, std::chrono::seconds(20));
  if (!snapshot.has_value()) {
    ADD_FAILURE() << worker.last_error();
    return nullptr;
  }
  return *snapshot;
}

std::size_t CountKept(const std::vector<std::uint8_t>& labels) {
  return static_cast<std::size_t>(
      std::count(labels.begin(), labels.end(), std::uint8_t{0}));
}

std::optional<sensor_msgs::msg::PointCloud2> ReceiveGlobalMap(
    mapOptimization& node) {
  auto observer = std::make_shared<rclcpp::Node>("online_map_observer");
  std::optional<sensor_msgs::msg::PointCloud2> received;
  auto subscription = observer->create_subscription<sensor_msgs::msg::PointCloud2>(
      node.pubLaserCloudSurround->get_topic_name(), rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        received = *message;
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(3);
  while (node.pubLaserCloudSurround->get_subscription_count() == 0 &&
         std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  node.publishGlobalMap();
  while (!received && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return received;
}

CloudPtr CurrentExportPlane() {
  CloudPtr cloud(new Cloud());
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 10; ++column) {
      PointType point;
      point.x = 5.0F + static_cast<float>(row);
      point.y = 0.3F * static_cast<float>(column);
      point.z = -1.0F;
      point.intensity =
          10.0F * static_cast<float>(row) + static_cast<float>(column);
      cloud->push_back(point);
    }
  }
  return cloud;
}

bool CacheCurrentInputs(mapOptimization& node, const CloudPtr& scan,
                        std::string* error) {
  if (scan == nullptr || scan->size() != 40U) {
    if (error != nullptr) {
      *error = "current export fixture must be a 4x10 plane";
    }
    return false;
  }
  // Match the real keyframe callback: only this function stores export input.
  pcl::toROSMsg(*scan, node.cloudInfo.cloud_deskewed);
  node.cloudInfo.cloud_projection_rows = 4;
  node.cloudInfo.cloud_projection_columns = 10;
  node.cloudInfo.cloud_projection_indices.clear();
  for (std::size_t index = 0; index < scan->size(); ++index) {
    node.cloudInfo.cloud_projection_indices.push_back(
        static_cast<std::uint32_t>(index));
  }
  return node.StoreGroundInputsForExport(0, error);
}

CloudPtr PatchworkExportPlane() {
  CloudPtr cloud(new Cloud());
  for (int x = -50; x <= 50; ++x) {
    for (int y = -50; y <= 50; ++y) {
      const float px = 0.2F * static_cast<float>(x);
      const float py = 0.2F * static_cast<float>(y);
      if (std::hypot(px, py) <= 0.6F) {
        continue;
      }
      PointType point;
      point.x = px;
      point.y = py;
      point.z = -1.723F + 0.002F * std::sin(static_cast<float>(x + 3 * y));
      point.intensity = 0.25F * static_cast<float>(cloud->size());
      cloud->push_back(point);
    }
  }
  return cloud;
}

bool CachePatchworkInputs(mapOptimization& node, const CloudPtr& scan,
                          std::string* error, std::size_t index = 0) {
  pcl::toROSMsg(*scan, node.cloudInfo.cloud_export_scan);
  return node.StoreGroundInputsForExport(index, error);
}

void AppendEmptyKeyFrame(mapOptimization& node) {
  PointType pose3d;
  pose3d.x = 1.0F;
  pose3d.y = 0.0F;
  pose3d.z = 0.0F;
  pose3d.intensity = 0.0F;
  node.cloudKeyPoses3D->push_back(pose3d);

  PointTypePose pose6d = PointTypePose();
  pose6d.x = 1.0F;
  pose6d.y = 0.0F;
  pose6d.z = 0.0F;
  pose6d.roll = 0.0F;
  pose6d.pitch = 0.0F;
  pose6d.yaw = 0.0F;
  pose6d.time = 0.0;
  node.cloudKeyPoses6D->push_back(pose6d);

  node.cornerCloudKeyFrames.emplace_back(new Cloud());
  node.surfCloudKeyFrames.emplace_back(new Cloud());
  node.groundCloudKeyFrames.emplace_back(new Cloud());
}

PointType MakePoint(float x, float y, float z, float intensity = 1.0F) {
  PointType point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  return point;
}

bool HasPoint(const Cloud& cloud, float x, float y, float z, float tolerance) {
  for (const auto& point : cloud.points) {
    if (std::abs(point.x - x) <= tolerance &&
        std::abs(point.y - y) <= tolerance &&
        std::abs(point.z - z) <= tolerance) {
      return true;
    }
  }
  return false;
}

CloudPtr CompleteScanWithTargetAndExtra() {
  CloudPtr cloud = DenseWall(10.0F);
  cloud->push_back(MakePoint(5.0F, 0.05F, 0.05F));  // dynamic target
  cloud->push_back(MakePoint(4.0F, 4.0F, 0.5F));    // non-feature static
  return cloud;
}

CloudPtr EvidenceObservingTarget() {
  CloudPtr cloud = DenseWall(10.0F);
  cloud->push_back(MakePoint(5.0F, 0.05F, 0.05F));
  return cloud;
}

// Uses the same production cache writer the mapping callback uses.
bool CacheCompleteScan(mapOptimization& node, std::size_t index,
                       const CloudPtr& cloud, std::string* error) {
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(*cloud, message);
  return node.StoreCompleteKeyframeScan(index, message, error);
}

std::vector<PointType> FilterExpected(
    const CloudPtr& cloud, const std::vector<std::uint8_t>& labels) {
  std::vector<PointType> expected;
  for (std::size_t index = 0; index < cloud->size() && index < labels.size();
       ++index) {
    if (labels[index] == 0U) expected.push_back(cloud->points[index]);
  }
  return expected;
}

void AppendKeyFrame(mapOptimization& node, float x) {
  PointType pose3d = node.cloudKeyPoses3D->back();
  pose3d.x = x;
  pose3d.intensity = static_cast<float>(node.cloudKeyPoses3D->size());
  node.cloudKeyPoses3D->push_back(pose3d);
  PointTypePose pose6d = node.cloudKeyPoses6D->back();
  pose6d.x = x;
  pose6d.intensity = pose3d.intensity;
  node.cloudKeyPoses6D->push_back(pose6d);
  node.cornerCloudKeyFrames.push_back(MakePlaneCloud(1.0F));
  node.surfCloudKeyFrames.push_back(MakePlaneCloud(0.5F));
  node.groundCloudKeyFrames.push_back(MakePlaneCloud(0.0F));
}

}  // namespace

TEST(MapOptimizationSaveExport, DisabledTransformedCacheRebuildsSameLocalMap) {
  EnsureRosInitialized();
  TempDirectory temp;
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);
  options.parameter_overrides({
      rclcpp::Parameter("local_map.transformed_cache_max_mb", 0),
      rclcpp::Parameter("dufomap.enable", false),
      rclcpp::Parameter("dufomap.cache_directory",
                        (temp.path() / "cache").string()),
      rclcpp::Parameter("map_save.generate_scan_context_db", false),
      rclcpp::Parameter("static_map_filter.enable", false),
  });
  mapOptimization node(options);
  SeedSingleKeyFrame(node);
  ASSERT_EQ(node.transformed_keyframe_cache_.max_bytes(), 0U);

  node.ExtractCloudByKeyIndices({0});
  const Cloud first_corner = *node.laserCloudCornerFromMapDS;
  const Cloud first_surf = *node.laserCloudSurfFromMapDS;
  ASSERT_FALSE(first_corner.empty());
  ASSERT_FALSE(first_surf.empty());
  EXPECT_EQ(node.transformed_keyframe_cache_.size(), 0U);
  EXPECT_EQ(node.transformed_keyframe_cache_.bytes(), 0U);

  node.ExtractCloudByKeyIndices({0});
  ASSERT_EQ(node.laserCloudCornerFromMapDS->size(), first_corner.size());
  ASSERT_EQ(node.laserCloudSurfFromMapDS->size(), first_surf.size());
  for (std::size_t i = 0; i < first_corner.size(); ++i) {
    EXPECT_FLOAT_EQ(node.laserCloudCornerFromMapDS->points[i].x,
                    first_corner.points[i].x);
    EXPECT_FLOAT_EQ(node.laserCloudCornerFromMapDS->points[i].y,
                    first_corner.points[i].y);
    EXPECT_FLOAT_EQ(node.laserCloudCornerFromMapDS->points[i].z,
                    first_corner.points[i].z);
  }
  for (std::size_t i = 0; i < first_surf.size(); ++i) {
    EXPECT_FLOAT_EQ(node.laserCloudSurfFromMapDS->points[i].x,
                    first_surf.points[i].x);
    EXPECT_FLOAT_EQ(node.laserCloudSurfFromMapDS->points[i].y,
                    first_surf.points[i].y);
    EXPECT_FLOAT_EQ(node.laserCloudSurfFromMapDS->points[i].z,
                    first_surf.points[i].z);
  }
}

TEST(MapOptimizationSaveExport, TransformedCacheHitAndEvictionPreserveLocalMap) {
  EnsureRosInitialized();
  TempDirectory temp;
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);
  options.parameter_overrides({
      rclcpp::Parameter("local_map.transformed_cache_max_mb", 1),
      rclcpp::Parameter("dufomap.enable", false),
      rclcpp::Parameter("dufomap.cache_directory",
                        (temp.path() / "cache").string()),
      rclcpp::Parameter("map_save.generate_scan_context_db", false),
      rclcpp::Parameter("static_map_filter.enable", false),
  });
  mapOptimization node(options);
  SeedSingleKeyFrame(node);
  node.cloudKeyPoses3D->front().x = 1.5F;
  node.cloudKeyPoses6D->front().x = 1.5F;
  node.cloudKeyPoses6D->front().yaw = 0.25F;

  node.ExtractCloudByKeyIndices({0});
  const Cloud expected_corner = *node.laserCloudCornerFromMapDS;
  const Cloud expected_surf = *node.laserCloudSurfFromMapDS;
  ASSERT_FALSE(expected_corner.empty());
  ASSERT_FALSE(expected_surf.empty());
  ASSERT_NE(node.transformed_keyframe_cache_.Find(0), nullptr);
  const auto expect_same_map = [&]() {
    const auto expect_same_cloud = [](const Cloud& actual,
                                      const Cloud& expected) {
      ASSERT_EQ(actual.size(), expected.size());
      for (std::size_t i = 0; i < actual.size(); ++i) {
        EXPECT_FLOAT_EQ(actual[i].x, expected[i].x);
        EXPECT_FLOAT_EQ(actual[i].y, expected[i].y);
        EXPECT_FLOAT_EQ(actual[i].z, expected[i].z);
        EXPECT_FLOAT_EQ(actual[i].intensity, expected[i].intensity);
      }
    };
    expect_same_cloud(*node.laserCloudCornerFromMapDS, expected_corner);
    expect_same_cloud(*node.laserCloudSurfFromMapDS, expected_surf);
  };

  node.ExtractCloudByKeyIndices({0});
  expect_same_map();

  Cloud eviction_cloud;
  const std::size_t budget = node.transformed_keyframe_cache_.max_bytes();
  eviction_cloud.points.reserve(budget / sizeof(PointType));
  ASSERT_EQ(eviction_cloud.points.capacity() * sizeof(PointType), budget);
  ASSERT_TRUE(node.transformed_keyframe_cache_.Insert(
      99, std::move(eviction_cloud), Cloud()));
  ASSERT_EQ(node.transformed_keyframe_cache_.Find(0), nullptr);

  node.ExtractCloudByKeyIndices({0});
  expect_same_map();
  EXPECT_LE(node.transformed_keyframe_cache_.bytes(), budget);

  node.transformed_keyframe_cache_.Clear();
  auto key_positions = pcl::make_shared<Cloud>();
  key_positions->push_back(node.cloudKeyPoses3D->front());
  node.extractCloud(key_positions);
  expect_same_map();
}

TEST(MapOptimizationSaveExport, PoseCorrectionClearsTransformedCache) {
  EnsureRosInitialized();
  TempDirectory temp;
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);
  options.parameter_overrides({
      rclcpp::Parameter("dufomap.enable", false),
      rclcpp::Parameter("dufomap.cache_directory",
                        (temp.path() / "cache").string()),
      rclcpp::Parameter("map_save.generate_scan_context_db", false),
      rclcpp::Parameter("static_map_filter.enable", false),
  });
  mapOptimization node(options);
  SeedSingleKeyFrame(node);
  node.ExtractCloudByKeyIndices({0});
  ASSERT_EQ(node.transformed_keyframe_cache_.size(), 1U);
  ASSERT_GT(node.transformed_keyframe_cache_.bytes(), 0U);
  node.UpdateLocalMapCache({0});
  ASSERT_TRUE(node.localMapCacheValid);

  node.isamCurrentEstimate.insert(0, gtsam::Pose3());
  node.aLoopIsClosed = true;
  node.correctPoses();

  EXPECT_EQ(node.transformed_keyframe_cache_.size(), 0U);
  EXPECT_EQ(node.transformed_keyframe_cache_.bytes(), 0U);
  EXPECT_FALSE(node.localMapCacheValid);
}

TEST(MapOptimizationSaveExport, OptionsRequestSavesGroundMap) {
  EnsureRosInitialized();
  TempDirectory temp;
  auto node = MakeBasicExportNode();

  const fs::path destination = temp.path() / "ground_requested_on";
  const MapExportOutcome outcome =
      ExportThroughOptionsService(*node, destination, /*save_ground_map=*/true);

  EXPECT_TRUE(outcome.success) << outcome.message;
  const fs::path ground = GroundArtifact(destination);
  ASSERT_TRUE(fs::exists(ground)) << ground;

  pcl::PointCloud<PointType> saved;
  ASSERT_EQ(pcl::io::loadPCDFile(ground.string(), saved), 0);
  EXPECT_EQ(saved.size(), kSeededGroundPoints);
}

TEST(MapOptimizationSaveExport, GroundGridExportPreservesSlopeAtCellCenters) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.ground_grid = true;
  spec.ground_resolution = 0.5;
  auto node = MakeExportNode(temp.path() / "cache", spec);

  CloudPtr plane(new Cloud());
  for (int ix = -15; ix <= 19; ++ix) {
    for (int iy = -15; iy <= 19; ++iy) {
      const float x = 0.1F * static_cast<float>(ix);
      const float y = 0.1F * static_cast<float>(iy);
      PointType point;
      point.x = x;
      point.y = y;
      point.z = 0.2F * x + 0.1F * y + 0.3F;
      point.intensity = 0.0F;
      plane->push_back(point);
    }
  }
  node->groundCloudKeyFrames[0] = plane;

  const fs::path destination = temp.path() / "ground_grid_slope";
  const MapExportOutcome outcome = ExportThroughOptionsService(
      *node, destination, /*save_ground_map=*/true, /*save_tiled_map=*/false,
      /*ground_resolution=*/-1.0F);
  ASSERT_TRUE(outcome.success) << outcome.message;

  pcl::PointCloud<PointType> saved;
  ASSERT_EQ(pcl::io::loadPCDFile(GroundArtifact(destination).string(), saved), 0);
  ASSERT_FALSE(saved.empty());
  std::set<std::pair<int, int>> keys;
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();
  for (const auto& point : saved.points) {
    ASSERT_TRUE(std::isfinite(point.x));
    ASSERT_TRUE(std::isfinite(point.y));
    ASSERT_TRUE(std::isfinite(point.z));
    const int ix = static_cast<int>(std::floor(point.x / 0.5F));
    const int iy = static_cast<int>(std::floor(point.y / 0.5F));
    keys.insert({ix, iy});
    EXPECT_FLOAT_EQ(point.x, (static_cast<float>(ix) + 0.5F) * 0.5F);
    EXPECT_FLOAT_EQ(point.y, (static_cast<float>(iy) + 0.5F) * 0.5F);
    EXPECT_NEAR(point.z, 0.2F * point.x + 0.1F * point.y + 0.3F, 0.02F);
    min_z = std::min(min_z, point.z);
    max_z = std::max(max_z, point.z);
  }
  EXPECT_EQ(keys.size(), saved.size());
  EXPECT_GT(max_z - min_z, 0.4F);
}

TEST(MapOptimizationSaveExport, GroundGridExportWiresMaxHoleCells) {
  EnsureRosInitialized();
  TempDirectory temp;
  const auto export_hole = [&temp](const fs::path& destination,
                                   int max_hole_cells) {
    ExportNodeSpec spec;
    spec.ground_grid = true;
    spec.grid_patches = true;
    spec.ground_resolution = 0.1;
    spec.ground_grid_max_hole_cells = max_hole_cells;
    auto node = MakeExportNode(temp.path() / "cache", spec);

    CloudPtr field(new Cloud());
    for (int ix = -6; ix <= 6; ++ix) {
      for (int iy = -6; iy <= 6; ++iy) {
        if ((ix == 0 || ix == 1) && iy == 0) {
          continue;
        }
        PointType point;
        point.x = (static_cast<float>(ix) + 0.5F) * 0.1F;
        point.y = (static_cast<float>(iy) + 0.5F) * 0.1F;
        point.z = 0.1F;
        point.intensity = 0.0F;
        field->push_back(point);
      }
    }
    node->groundCloudKeyFrames[0] = field;
    return ExportThroughOptionsService(
        *node, destination, /*save_ground_map=*/true,
        /*save_tiled_map=*/false, /*ground_resolution=*/-1.0F);
  };

  const auto has_cell = [](const Cloud& cloud, int ix, int iy) {
    const float x = (static_cast<float>(ix) + 0.5F) * 0.1F;
    const float y = (static_cast<float>(iy) + 0.5F) * 0.1F;
    for (const auto& point : cloud.points) {
      if (std::fabs(point.x - x) < 1.0e-5F &&
          std::fabs(point.y - y) < 1.0e-5F) {
        return true;
      }
    }
    return false;
  };

  const fs::path limited_destination = temp.path() / "hole_limit_one";
  const MapExportOutcome limited =
      export_hole(limited_destination, /*max_hole_cells=*/1);
  ASSERT_TRUE(limited.success) << limited.message;
  Cloud limited_cloud;
  ASSERT_EQ(pcl::io::loadPCDFile(
                GroundArtifact(limited_destination).string(), limited_cloud),
            0);
  EXPECT_FALSE(has_cell(limited_cloud, 0, 0));
  EXPECT_FALSE(has_cell(limited_cloud, 1, 0));

  const fs::path filled_destination = temp.path() / "hole_limit_fourteen";
  const MapExportOutcome filled =
      export_hole(filled_destination, /*max_hole_cells=*/14);
  ASSERT_TRUE(filled.success) << filled.message;
  Cloud filled_cloud;
  ASSERT_EQ(pcl::io::loadPCDFile(
                GroundArtifact(filled_destination).string(), filled_cloud),
            0);
  EXPECT_TRUE(has_cell(filled_cloud, 0, 0));
  EXPECT_TRUE(has_cell(filled_cloud, 1, 0));
}

TEST(MapOptimizationSaveExport,
     GroundGridExportRejectsZeroResolvedResolutionBeforeReplacingArtifacts) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.ground_grid = true;
  spec.ground_resolution = 0.5;
  auto node = MakeExportNode(temp.path() / "cache", spec);

  const fs::path destination = temp.path() / "ground_grid_zero";
  const fs::path ground = GroundArtifact(destination);
  fs::create_directories(ground.parent_path());
  { std::ofstream stale(ground); stale << "previous ground artifact\n"; }

  const MapExportOutcome outcome = ExportThroughOptionsService(
      *node, destination, /*save_ground_map=*/true, /*save_tiled_map=*/false,
      /*ground_resolution=*/0.0F);
  EXPECT_FALSE(outcome.success);
  EXPECT_NE(outcome.message.find("ground_resolution"), std::string::npos)
      << outcome.message;

  std::ifstream stream(ground);
  const std::string contents((std::istreambuf_iterator<char>(stream)), {});
  EXPECT_EQ(contents, "previous ground artifact\n");
}

TEST(MapOptimizationSaveExport, GroundGridExportUsesServiceResolution) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.ground_grid = true;
  spec.ground_resolution = 0.5;
  auto node = MakeExportNode(temp.path() / "cache", spec);
  node->groundCloudKeyFrames[0] = MakePointCloud({
      {0.02F, 0.03F, 0.0F}, {0.03F, 0.04F, 0.01F},
      {0.22F, 0.03F, 0.02F}, {0.23F, 0.04F, 0.02F},
  });
  const fs::path destination = temp.path() / "ground_grid_override";
  const MapExportOutcome outcome = ExportThroughOptionsService(
      *node, destination, true, false, 0.1F);
  ASSERT_TRUE(outcome.success) << outcome.message;

  Cloud saved;
  ASSERT_EQ(pcl::io::loadPCDFile(GroundArtifact(destination).string(), saved), 0);
  ASSERT_EQ(saved.size(), 2U);
  EXPECT_NEAR(saved[0].x, 0.05F, 1e-6F);
  EXPECT_NEAR(saved[1].x, 0.25F, 1e-6F);
  for (const auto& point : saved.points) {
    EXPECT_NEAR(point.y, 0.05F, 1e-6F);
  }
}

TEST(MapOptimizationSaveExport,
     DisabledGroundGridPreservesRawGroundAtZeroResolution) {
  EnsureRosInitialized();
  TempDirectory temp;
  auto node = MakeExportNode(temp.path() / "cache", ExportNodeSpec{});
  const auto raw = MakePointCloud({
      {0.02F, 0.03F, 0.0F}, {0.03F, 0.04F, 0.01F},
      {0.22F, 0.03F, 0.02F}, {0.23F, 0.04F, 0.02F},
  });
  node->groundCloudKeyFrames[0] = raw;
  const fs::path destination = temp.path() / "ground_grid_off";
  const MapExportOutcome outcome = ExportThroughOptionsService(
      *node, destination, true, false, 0.0F);
  ASSERT_TRUE(outcome.success) << outcome.message;

  Cloud saved;
  ASSERT_EQ(pcl::io::loadPCDFile(GroundArtifact(destination).string(), saved), 0);
  ASSERT_EQ(saved.size(), raw->size());
  for (std::size_t i = 0; i < saved.size(); ++i) {
    EXPECT_FLOAT_EQ(saved[i].x, (*raw)[i].x);
    EXPECT_FLOAT_EQ(saved[i].y, (*raw)[i].y);
    EXPECT_FLOAT_EQ(saved[i].z, (*raw)[i].z);
  }
}

TEST(MapOptimizationSaveExport, GroundGridSkipsValidationWhenGroundSaveIsOff) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.ground_grid = true;
  auto node = MakeExportNode(temp.path() / "cache", spec);
  const fs::path destination = temp.path() / "ground_grid_not_requested";
  const MapExportOutcome outcome = ExportThroughOptionsService(
      *node, destination, false, false, 0.0F);
  EXPECT_TRUE(outcome.success) << outcome.message;
  EXPECT_FALSE(fs::exists(GroundArtifact(destination)));
}

TEST(MapOptimizationSaveExport, OptionsRequestRemovesStaleGroundMap) {
  EnsureRosInitialized();
  TempDirectory temp;
  auto node = MakeBasicExportNode();

  const fs::path destination = temp.path() / "ground_requested_off";
  const fs::path ground = GroundArtifact(destination);
  fs::create_directories(ground.parent_path());
  { std::ofstream stale(ground); stale << "stale ground from an earlier save\n"; }
  ASSERT_TRUE(fs::exists(ground));

  const MapExportOutcome outcome =
      ExportThroughOptionsService(*node, destination, /*save_ground_map=*/false);

  EXPECT_TRUE(outcome.success) << outcome.message;
  EXPECT_FALSE(fs::exists(ground)) << ground;
}

TEST(MapOptimizationSaveExport, OnlineSnapshotDrivesSavedLabels) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.dufomap_enable = true;
  auto node = MakeExportNode(temp.path() / "cache", spec);
  const auto snapshot = SeedOnlineSnapshot(*node, {node->surfCloudKeyFrames[0]});
  ASSERT_NE(snapshot, nullptr);
  ASSERT_EQ(snapshot->labels.size(), 1U);
  ASSERT_EQ(snapshot->labels[0].surf.size(),
            node->surfCloudKeyFrames[0]->size());

  const fs::path destination = temp.path() / "map";
  const MapExportOutcome outcome =
      ExportThroughOptionsService(*node, destination, /*save_ground_map=*/false);
  ASSERT_TRUE(outcome.success) << outcome.message;

  pcl::PointCloud<PointType> saved;
  ASSERT_EQ(pcl::io::loadPCDFile(
                MapArtifactLayout(destination).SurfaceMap().string(), saved), 0);
  EXPECT_EQ(saved.size(), CountKept(snapshot->labels[0].surf));
}

TEST(MapOptimizationSaveExport, OnlineSnapshotAppliesEveryFreeLabel) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.dufomap_enable = true;
  auto node = MakeExportNode(temp.path() / "cache", spec);
  // A ray-through target and a lateral target exercise both a filtered and a
  // retained point. The assertions follow the real snapshot labels instead of
  // hard-coding an algorithm outcome that the worker unit tests already pin.
  node->cornerCloudKeyFrames[0] =
      MakePointCloud({{5.0F, 0.0F, 0.0F}, {0.0F, 5.0F, 0.0F}});
  node->surfCloudKeyFrames[0] = DenseWall(10.0F);

  const auto snapshot = SeedOnlineSnapshot(
      *node, {ShiftY(DenseWall(10.0F), 5.0F), DenseWall(10.0F)});
  ASSERT_NE(snapshot, nullptr);
  ASSERT_EQ(snapshot->labels[0].corner.size(), 2U);

  const fs::path destination = temp.path() / "map";
  const MapExportOutcome outcome =
      ExportThroughOptionsService(*node, destination, /*save_ground_map=*/false);
  ASSERT_TRUE(outcome.success) << outcome.message;

  pcl::PointCloud<PointType> saved;
  ASSERT_EQ(pcl::io::loadPCDFile(
                MapArtifactLayout(destination).CornerMap().string(), saved), 0);
  std::vector<PointType> expected;
  const auto& source = node->cornerCloudKeyFrames[0]->points;
  for (std::size_t index = 0; index < source.size(); ++index) {
    if (snapshot->labels[0].corner[index] == 0U)
      expected.push_back(source[index]);
  }
  ASSERT_EQ(saved.size(), expected.size());
  for (std::size_t index = 0; index < expected.size(); ++index) {
    EXPECT_FLOAT_EQ(saved.points[index].x, expected[index].x);
    EXPECT_FLOAT_EQ(saved.points[index].y, expected[index].y);
  }
}

TEST(MapOptimizationSaveExport, GlobalMapPublishesClassifiedPrefix) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.dufomap_enable = true;
  auto node = MakeExportNode(temp.path() / "cache", spec);
  ASSERT_NE(SeedOnlineSnapshot(*node, {node->surfCloudKeyFrames[0]}), nullptr);

  // A new keyframe may be waiting for background classification while the
  // previous snapshot remains valid. RViz must still receive the ready map.
  PointType pose3d = node->cloudKeyPoses3D->front();
  pose3d.intensity = 1.0F;
  node->cloudKeyPoses3D->push_back(pose3d);
  PointTypePose pose6d = node->cloudKeyPoses6D->front();
  pose6d.intensity = 1.0F;
  node->cloudKeyPoses6D->push_back(pose6d);
  node->cornerCloudKeyFrames.push_back(
      MakePointCloud({{50.0F, 50.0F, 50.0F}}));
  node->surfCloudKeyFrames.push_back(MakePointCloud({{60.0F, 60.0F, 60.0F}}));

  const auto message = ReceiveGlobalMap(*node);
  ASSERT_TRUE(message.has_value());
  Cloud published;
  pcl::fromROSMsg(*message, published);
  ASSERT_FALSE(published.empty());
  for (const auto& point : published) {
    EXPECT_LT(point.x, 10.0F);
  }
}

TEST(MapOptimizationSaveExport, GlobalMapAndSaveRemoveTheSameDynamicTarget) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.dufomap_enable = true;
  spec.dufomap_dp = 0;
  auto node = MakeExportNode(temp.path() / "cache", spec);
  node->cornerCloudKeyFrames[0] =
      MakePointCloud({{5.0F, 0.05F, 0.05F}, {0.0F, 5.0F, 0.0F}});
  node->surfCloudKeyFrames[0] = DenseWall(10.0F);
  const auto snapshot = SeedOnlineSnapshot(*node, {DenseWall(10.0F)});
  ASSERT_NE(snapshot, nullptr);
  ASSERT_EQ(snapshot->labels[0].corner.size(), 2U);
  ASSERT_EQ(snapshot->labels[0].corner[0], 1U);
  ASSERT_EQ(snapshot->labels[0].corner[1], 0U);

  const auto message = ReceiveGlobalMap(*node);
  ASSERT_TRUE(message.has_value());
  Cloud published;
  pcl::fromROSMsg(*message, published);
  bool has_static_target = false;
  for (const auto& point : published) {
    EXPECT_FALSE(std::abs(point.x - 5.0F) < 0.1F &&
                 std::abs(point.y - 0.05F) < 0.1F);
    has_static_target = has_static_target ||
        (std::abs(point.x) < 0.1F && std::abs(point.y - 5.0F) < 0.1F);
  }
  EXPECT_TRUE(has_static_target);

  const auto destination = temp.path() / "map";
  const auto saved = ExportThroughOptionsService(*node, destination, false);
  ASSERT_TRUE(saved.success) << saved.message;
  Cloud corner;
  ASSERT_EQ(pcl::io::loadPCDFile(
                MapArtifactLayout(destination).CornerMap().string(), corner), 0);
  ASSERT_EQ(corner.size(), 1U);
  EXPECT_FLOAT_EQ(corner[0].x, 0.0F);
  EXPECT_FLOAT_EQ(corner[0].y, 5.0F);
  EXPECT_EQ(node->cornerCloudKeyFrames[0]->size(), 2U);
}

TEST(MapOptimizationSaveExport, LabelLengthMismatchFailsBeforeReplacingArtifacts) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.dufomap_enable = true;
  auto node = MakeExportNode(temp.path() / "cache", spec);
  ASSERT_NE(SeedOnlineSnapshot(*node, {node->surfCloudKeyFrames[0]}), nullptr);

  const fs::path destination = temp.path() / "map";
  ASSERT_TRUE(ExportThroughOptionsService(*node, destination, false).success);
  const fs::path surface = MapArtifactLayout(destination).SurfaceMap();
  pcl::PointCloud<PointType> before;
  ASSERT_EQ(pcl::io::loadPCDFile(surface.string(), before), 0);

  // The worker snapshot still covers the old cloud length. Save must reject
  // this instead of silently writing an empty or truncated map.
  PointType extra;
  extra.x = 9.0F;
  extra.y = 9.0F;
  extra.z = 9.0F;
  extra.intensity = 1.0F;
  node->surfCloudKeyFrames[0]->push_back(extra);

  const MapExportOutcome outcome =
      ExportThroughOptionsService(*node, destination, false);
  EXPECT_FALSE(outcome.success);
  EXPECT_NE(outcome.message.find("label count mismatch"), std::string::npos)
      << outcome.message;

  pcl::PointCloud<PointType> after;
  ASSERT_EQ(pcl::io::loadPCDFile(surface.string(), after), 0);
  EXPECT_EQ(after.size(), before.size());
}

TEST(MapOptimizationSaveExport, UnavailableWorkerFailsBeforeReplacingArtifacts) {
  EnsureRosInitialized();
  TempDirectory temp;
  const fs::path cache_blocker = temp.path() / "cache_is_a_file";
  { std::ofstream blocker(cache_blocker); blocker << "not a directory\n"; }

  const fs::path destination = temp.path() / "map";
  const fs::path surface = MapArtifactLayout(destination).SurfaceMap();
  fs::create_directories(surface.parent_path());
  { std::ofstream old(surface); old << "previous map artifact\n"; }

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);
  options.parameter_overrides({
      rclcpp::Parameter("dufomap.enable", true),
      rclcpp::Parameter("dufomap.cache_directory", cache_blocker.string()),
      rclcpp::Parameter("map_save.generate_scan_context_db", false),
      rclcpp::Parameter("static_map_filter.enable", false),
  });
  mapOptimization node(options);
  SeedSingleKeyFrame(node);

  const MapExportOutcome outcome =
      ExportThroughOptionsService(node, destination, false);
  EXPECT_FALSE(outcome.success);
  EXPECT_NE(outcome.message.find("DUFOMap"), std::string::npos)
      << outcome.message;
  ASSERT_TRUE(fs::exists(surface));
  std::ifstream stream(surface);
  const std::string contents((std::istreambuf_iterator<char>(stream)), {});
  EXPECT_EQ(contents, "previous map artifact\n");
}

TEST(MapOptimizationSaveExport, DisabledDufomapDoesNotRequireGroundCache) {
  EnsureRosInitialized();
  TempDirectory temp;
  auto node = MakeBasicExportNode();
  node->groundCloudKeyFrames.clear();
  EXPECT_TRUE(ExportThroughOptionsService(*node, temp.path(), false).success);
}

TEST(MapOptimizationSaveExport, DufomapCacheFailureAllowsNullErrorOutput) {
  EnsureRosInitialized();
  TempDirectory temp;
  auto node = MakeBasicExportNode();
  pcl::PointCloud<PointType>::Ptr empty(new pcl::PointCloud<PointType>());
  EXPECT_FALSE(node->StoreFullScanForExport(0, empty, nullptr));
}

TEST(MapOptimizationSaveExport, DufomapPcdWriteExceptionsAreReportedAsFailure) {
  EnsureRosInitialized();
  TempDirectory temp;
  auto node = MakeBasicExportNode();
  const fs::path blocked = temp.path() / "pcd_as_directory";
  fs::create_directories(blocked);

  std::string error;
  EXPECT_FALSE(node->WriteCloudBinary(
      blocked, *node->surfCloudKeyFrames[0], &error));
  EXPECT_NE(error.find("cannot write"), std::string::npos) << error;
}

TEST(MapOptimizationSaveExport, KeyframeCacheRetainedAfterExport) {
  EnsureRosInitialized();
  TempDirectory temp;
  fs::path cache;
  {
    auto node = MakeBasicExportNode();
    cache = node->DufomapCacheDirectory();
    ASSERT_FALSE(cache.empty());
    CacheSeed(*node);
    ASSERT_TRUE(ExportThroughOptionsService(*node, temp.path() / "map", false)
                    .success);
    EXPECT_TRUE(fs::exists(cache));
  }
  EXPECT_TRUE(fs::exists(cache / "scans/000000.pcd"));
}

TEST(MapOptimizationSaveExport, DufomapKeepsGroundPatchFlattenAndSaveSwitch) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.grid_patches = true;
  spec.flatten = true;
  spec.current_mode = "online";
  auto node = MakeExportNode(temp.path() / "cache", spec);
  CacheSeed(*node);

  const fs::path destination = temp.path() / "map";
  const auto skipped =
      ExportThroughOptionsService(*node, destination, /*save_ground_map=*/false);
  ASSERT_TRUE(skipped.success) << skipped.message;
  EXPECT_FALSE(fs::exists(GroundArtifact(destination)));

  const auto saved =
      ExportThroughOptionsService(*node, destination, /*save_ground_map=*/true);
  ASSERT_TRUE(saved.success) << saved.message;
  const fs::path ground = GroundArtifact(destination);
  ASSERT_TRUE(fs::exists(ground)) << ground;
  pcl::PointCloud<PointType> points;
  ASSERT_EQ(pcl::io::loadPCDFile(ground.string(), points), 0);
  EXPECT_GE(points.size(), kSeededGroundPoints);
  for (const auto& point : points) {
    EXPECT_FLOAT_EQ(point.z, 0.1F);
  }
}

TEST(MapOptimizationSaveExport,
     CurrentExportWithoutDufomapUsesOriginalProjectionAndFinalPose) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.current_mode = "export";
  spec.scan_lines = 4;
  spec.horizon_scan = 10;
  auto node = MakeExportNode(temp.path() / "cache", spec);
  const CloudPtr plane = CurrentExportPlane();
  std::string error;
  ASSERT_TRUE(CacheCurrentInputs(*node, plane, &error)) << error;

  node->cloudKeyPoses3D->points[0].x = 7.0F;
  node->cloudKeyPoses6D->points[0].x = 7.0F;
  const fs::path destination = temp.path() / "map";
  const auto result =
      ExportThroughOptionsService(*node, destination, /*save_ground_map=*/true);
  ASSERT_TRUE(result.success) << result.message;

  pcl::PointCloud<PointType> ground;
  ASSERT_EQ(pcl::io::loadPCDFile(GroundArtifact(destination).string(), ground),
            0);
  ASSERT_EQ(ground.size(), 20U);
  for (const auto& point : ground) {
    EXPECT_GT(point.x, 11.9F);
    EXPECT_FLOAT_EQ(point.z, -1.0F);
  }
}

TEST(MapOptimizationSaveExport,
     GroundMeshDefaultOffPreservesExistingCurrentExportPoints) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.current_mode = "export";
  spec.scan_lines = 4;
  spec.horizon_scan = 10;
  auto node = MakeExportNode(temp.path() / "cache", spec);
  const CloudPtr plane = CurrentExportPlane();
  std::string error;
  ASSERT_TRUE(CacheCurrentInputs(*node, plane, &error)) << error;

  const fs::path destination = temp.path() / "map";
  const auto result =
      ExportThroughOptionsService(*node, destination, /*save_ground_map=*/true);
  ASSERT_TRUE(result.success) << result.message;
  Cloud ground;
  ASSERT_EQ(pcl::io::loadPCDFile(GroundArtifact(destination).string(), ground),
            0);
  ASSERT_EQ(ground.size(), 20U);
  for (const auto& point : ground) {
    EXPECT_FLOAT_EQ(point.z, -1.0F);
  }
}

TEST(MapOptimizationSaveExport,
     GroundMeshAddsRasterizedCellCentersWithoutReplacingMeasuredPoints) {
  EnsureRosInitialized();
  TempDirectory temp;
  const auto run = [&temp](bool mesh, const fs::path& destination) {
    ExportNodeSpec spec;
    spec.current_mode = "export";
    spec.scan_lines = 4;
    spec.horizon_scan = 10;
    spec.ground_mesh = mesh;
    spec.ground_grid = true;
    spec.ground_resolution = 0.5;
    auto node = MakeExportNode(temp.path() / "cache", spec);
    const CloudPtr plane = CurrentExportPlane();
    std::string error;
    EXPECT_TRUE(CacheCurrentInputs(*node, plane, &error)) << error;
    return ExportThroughOptionsService(
        *node, destination, /*save_ground_map=*/true,
        /*save_tiled_map=*/false, /*ground_resolution=*/-1.0F);
  };
  const fs::path plain_destination = temp.path() / "mesh_off";
  const auto plain = run(false, plain_destination);
  ASSERT_TRUE(plain.success) << plain.message;
  const fs::path mesh_destination = temp.path() / "mesh_on";
  const auto meshed = run(true, mesh_destination);
  ASSERT_TRUE(meshed.success) << meshed.message;

  Cloud plain_ground;
  Cloud mesh_ground;
  ASSERT_EQ(pcl::io::loadPCDFile(
                GroundArtifact(plain_destination).string(), plain_ground), 0);
  ASSERT_EQ(pcl::io::loadPCDFile(GroundArtifact(mesh_destination).string(),
                                mesh_ground), 0);
  ASSERT_GT(mesh_ground.size(), plain_ground.size());
  std::set<std::pair<float, float>> plain_xy;
  for (const auto& point : plain_ground.points) {
    plain_xy.insert({point.x, point.y});
  }
  std::size_t appended_centers = 0;
  for (const auto& point : mesh_ground.points) {
    if (plain_xy.count({point.x, point.y}) != 0U) continue;
    ++appended_centers;
    EXPECT_NEAR(point.x, (std::floor(point.x / 0.5F) + 0.5F) * 0.5F, 1.0e-5F);
    EXPECT_NEAR(point.y, (std::floor(point.y / 0.5F) + 0.5F) * 0.5F, 1.0e-5F);
    EXPECT_FLOAT_EQ(point.z, -1.0F);
  }
  EXPECT_GT(appended_centers, 0U);
  for (const auto& point : plain_ground.points) {
    EXPECT_NE(mesh_ground.end(),
              std::find_if(mesh_ground.begin(), mesh_ground.end(),
                           [&](const PointType& candidate) {
                             return candidate.x == point.x &&
                                    candidate.y == point.y &&
                                    candidate.z == point.z;
                           }));
  }
}

TEST(MapOptimizationSaveExport,
     GroundMeshRejectsNonPositiveResolvedResolutionBeforeReplacingArtifacts) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.current_mode = "export";
  spec.scan_lines = 4;
  spec.horizon_scan = 10;
  spec.ground_mesh = true;
  spec.ground_resolution = 0.5;
  auto node = MakeExportNode(temp.path() / "cache", spec);
  const CloudPtr plane = CurrentExportPlane();
  std::string error;
  ASSERT_TRUE(CacheCurrentInputs(*node, plane, &error)) << error;

  const fs::path destination = temp.path() / "mesh_zero";
  const fs::path ground = GroundArtifact(destination);
  fs::create_directories(ground.parent_path());
  { std::ofstream stale(ground); stale << "previous ground artifact\n"; }
  const auto outcome = ExportThroughOptionsService(
      *node, destination, /*save_ground_map=*/true,
      /*save_tiled_map=*/false, /*ground_resolution=*/0.0F);
  EXPECT_FALSE(outcome.success);
  EXPECT_NE(outcome.message.find("ground_resolution"), std::string::npos)
      << outcome.message;
  std::ifstream stream(ground);
  const std::string contents((std::istreambuf_iterator<char>(stream)), {});
  EXPECT_EQ(contents, "previous ground artifact\n");
}

TEST(MapOptimizationSaveExport,
     GroundMeshRejectsUnsupportedModeBeforeReplacingArtifacts) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.current_mode = "off";
  spec.patchwork_mode = "export";
  spec.ground_mesh = true;
  auto node = MakeExportNode(temp.path() / "cache", spec);
  const fs::path destination = temp.path() / "mesh_unsupported";
  const fs::path ground = GroundArtifact(destination);
  fs::create_directories(ground.parent_path());
  { std::ofstream stale(ground); stale << "previous ground artifact\n"; }
  const auto outcome =
      ExportThroughOptionsService(*node, destination, /*save_ground_map=*/true);
  EXPECT_FALSE(outcome.success);
  EXPECT_NE(outcome.message.find("Current/RingPlane"), std::string::npos)
      << outcome.message;
  std::ifstream stream(ground);
  const std::string contents((std::istreambuf_iterator<char>(stream)), {});
  EXPECT_EQ(contents, "previous ground artifact\n");
}

TEST(MapOptimizationSaveExport,
     GroundMeshMissingProjectionFailsBeforeReplacingArtifacts) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.current_mode = "export";
  spec.scan_lines = 4;
  spec.horizon_scan = 10;
  spec.ground_mesh = true;
  spec.ground_resolution = 0.5;
  auto node = MakeExportNode(temp.path() / "cache", spec);
  const CloudPtr plane = CurrentExportPlane();
  std::string error;
  ASSERT_TRUE(CacheCurrentInputs(*node, plane, &error)) << error;
  fs::remove(node->CachedProjectionPath(0));

  const fs::path destination = temp.path() / "mesh_missing_projection";
  const fs::path ground = GroundArtifact(destination);
  fs::create_directories(ground.parent_path());
  { std::ofstream stale(ground); stale << "previous ground artifact\n"; }
  const auto outcome = ExportThroughOptionsService(
      *node, destination, /*save_ground_map=*/true,
      /*save_tiled_map=*/false, /*ground_resolution=*/-1.0F);
  EXPECT_FALSE(outcome.success);
  EXPECT_NE(outcome.message.find("projection"), std::string::npos)
      << outcome.message;
  std::ifstream stream(ground);
  const std::string contents((std::istreambuf_iterator<char>(stream)), {});
  EXPECT_EQ(contents, "previous ground artifact\n");
}

TEST(MapOptimizationSaveExport,
     GroundMeshLimitsAreIgnoredWhenGroundSaveIsOff) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.current_mode = "off";
  spec.patchwork_mode = "export";
  spec.ground_mesh = true;
  auto node = MakeExportNode(temp.path() / "cache", spec);
  const fs::path destination = temp.path() / "mesh_not_requested";
  const auto outcome =
      ExportThroughOptionsService(*node, destination, /*save_ground_map=*/false);
  EXPECT_TRUE(outcome.success) << outcome.message;
  EXPECT_FALSE(fs::exists(GroundArtifact(destination)));
}

TEST(MapOptimizationSaveExport,
     PatchworkExportWithoutDufomapUsesCompleteScansAndFinalPoses) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.current_mode = "online";
  spec.patchwork_mode = "export";
  auto node = MakeExportNode(temp.path() / "cache", spec);
  const CloudPtr plane = PatchworkExportPlane();
  std::string error;
  ASSERT_TRUE(CachePatchworkInputs(*node, plane, &error)) << error;
  SeedSingleKeyFrame(*node);
  ASSERT_TRUE(CachePatchworkInputs(*node, plane, &error, 1)) << error;
  node->cloudKeyPoses6D->points[0].z = 2.0F;
  node->cloudKeyPoses6D->points[1].z = 3.0F;

  const fs::path destination = temp.path() / "map";
  const auto result =
      ExportThroughOptionsService(*node, destination, /*save_ground_map=*/true);
  ASSERT_TRUE(result.success) << result.message;
  Cloud ground;
  ASSERT_EQ(pcl::io::loadPCDFile(GroundArtifact(destination).string(), ground), 0);
  EXPECT_GT(ground.size(), 18000U);
  std::size_t first = 0;
  std::size_t second = 0;
  for (const auto& point : ground) {
    if (std::abs(point.z - 0.277F) < 0.005F) ++first;
    if (std::abs(point.z - 1.277F) < 0.005F) ++second;
  }
  EXPECT_GT(first, 9000U);
  EXPECT_GT(second, 9000U);
  EXPECT_FALSE(fs::exists(node->CachedFullScanPath(0)));
  EXPECT_TRUE(fs::exists(node->CachedGroundScanPath(0)));
}

TEST(MapOptimizationSaveExport,
     CurrentExportPriorityDoesNotRequirePatchworkInput) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.current_mode = "export";
  spec.patchwork_mode = "export";
  spec.scan_lines = 4;
  spec.horizon_scan = 10;
  auto node = MakeExportNode(temp.path() / "cache", spec);
  EXPECT_TRUE(node->cloudInfo.cloud_export_scan.data.empty());
  const CloudPtr plane = CurrentExportPlane();
  std::string error;
  ASSERT_TRUE(CacheCurrentInputs(*node, plane, &error)) << error;

  const fs::path destination = temp.path() / "map";
  const auto result =
      ExportThroughOptionsService(*node, destination, /*save_ground_map=*/true);
  ASSERT_TRUE(result.success) << result.message;
  pcl::PointCloud<PointType> ground;
  ASSERT_EQ(pcl::io::loadPCDFile(GroundArtifact(destination).string(), ground), 0);
  EXPECT_FALSE(ground.empty());
}

TEST(MapOptimizationSaveExport,
     MissingOrCorruptGroundCacheFailsBeforeReplacingOldArtifacts) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.patchwork_mode = "export";
  auto node = MakeExportNode(temp.path() / "cache", spec);
  const CloudPtr plane = PatchworkExportPlane();
  std::string error;
  ASSERT_TRUE(CachePatchworkInputs(*node, plane, &error)) << error;

  const fs::path destination = temp.path() / "map";
  const auto first =
      ExportThroughOptionsService(*node, destination, /*save_ground_map=*/true);
  ASSERT_TRUE(first.success) << first.message;
  const fs::path ground_artifact = GroundArtifact(destination);
  pcl::PointCloud<PointType> before;
  ASSERT_EQ(pcl::io::loadPCDFile(ground_artifact.string(), before), 0);
  ASSERT_FALSE(before.empty());

  {
    std::ofstream corrupt(node->CachedGroundScanPath(0),
                          std::ios::binary | std::ios::trunc);
    corrupt << "not a pcd file\n";
  }
  const auto second =
      ExportThroughOptionsService(*node, destination, /*save_ground_map=*/true);
  EXPECT_FALSE(second.success);
  EXPECT_NE(second.message.find("Ground export failed"), std::string::npos)
      << second.message;
  EXPECT_TRUE(fs::exists(ground_artifact));
  pcl::PointCloud<PointType> after;
  ASSERT_EQ(pcl::io::loadPCDFile(ground_artifact.string(), after), 0);
  EXPECT_EQ(after.size(), before.size());
  EXPECT_FLOAT_EQ(after.front().x, before.front().x);
}

TEST(MapOptimizationSaveExport,
     GroundCacheErrorDoesNotBlockSaveWhenGroundIsOff) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.patchwork_mode = "export";
  auto node = MakeExportNode(temp.path() / "cache", spec);
  std::string error;
  EXPECT_FALSE(node->StoreGroundInputsForExport(0, &error));
  EXPECT_FALSE(error.empty());
  CacheSeed(*node);

  const fs::path destination = temp.path() / "map";
  const auto result =
      ExportThroughOptionsService(*node, destination, /*save_ground_map=*/false);
  EXPECT_TRUE(result.success) << result.message;
  EXPECT_TRUE(fs::exists(MapArtifactLayout(destination).SurfaceMap()));
  EXPECT_FALSE(fs::exists(GroundArtifact(destination)));
}

TEST(MapOptimizationSaveExport,
     BothGroundModesOffStillCreateSessionCacheAndWriteNoGround) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.current_mode = "off";
  spec.patchwork_mode = "off";
  auto node = MakeExportNode(temp.path() / "cache", spec);
  ASSERT_FALSE(node->DufomapCacheDirectory().empty());
  EXPECT_TRUE(fs::is_directory(node->DufomapCacheDirectory() / "scans"));

  const fs::path destination = temp.path() / "map";
  const auto result =
      ExportThroughOptionsService(*node, destination, /*save_ground_map=*/false);
  EXPECT_TRUE(result.success) << result.message;
  EXPECT_FALSE(fs::exists(GroundArtifact(destination)));
  EXPECT_TRUE(fs::exists(node->DufomapCacheDirectory()));
}

TEST(MapOptimizationSaveExport,
     GlobalMapUsesCompleteKeyframeScansAtFinalPose) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.dufomap_enable = true;
  spec.dufomap_dp = 0;
  auto node = MakeExportNode(temp.path() / "cache", spec);

  // Feature clouds deliberately differ from the complete scan. The complete
  // scan owns the extra static point and the dynamic target; the feature
  // corner is not part of it and must never leak into GlobalMap.
  node->cornerCloudKeyFrames[0] = MakePointCloud({{0.0F, 5.0F, 0.0F}});
  node->surfCloudKeyFrames[0] = DenseWall(10.0F);
  const CloudPtr complete = CompleteScanWithTargetAndExtra();
  const CloudPtr first_evidence = EvidenceObservingTarget();
  const CloudPtr second_evidence = DenseWall(10.0F);

  const auto old_snapshot = SeedOnlineSnapshot(
      *node, {first_evidence, second_evidence});
  ASSERT_NE(old_snapshot, nullptr);
  ASSERT_EQ(old_snapshot->labels[0].corner.size(), 1U);

  std::string cache_error;
  ASSERT_TRUE(CacheCompleteScan(*node, 0, complete, &cache_error))
      << cache_error;
  ASSERT_TRUE(fs::exists(node->CachedCompleteScanPath(0)));

  // Correct the pose after the scan was cached. Classification and export must
  // use this final generation, not the pose at cache time.
  node->cloudKeyPoses3D->points[0].x = 2.0F;
  node->cloudKeyPoses6D->points[0].x = 2.0F;
  node->UpdateOnlineDufomapPoses();
  const auto request = node->online_dufomap_->CaptureRequest(1);
  const auto labels =
      node->online_dufomap_->Barrier(request, std::chrono::seconds(20));
  ASSERT_TRUE(labels.has_value()) << node->online_dufomap_->last_error();

  const fs::path destination = temp.path() / "map";
  const MapExportOutcome outcome =
      ExportThroughOptionsService(*node, destination, /*save_ground_map=*/true);
  ASSERT_TRUE(outcome.success) << outcome.message;

  Cloud global;
  ASSERT_EQ(pcl::io::loadPCDFile(
                MapArtifactLayout(destination).GlobalMap().string(), global),
            0);
  EXPECT_TRUE(HasPoint(global, 6.0F, 4.0F, 0.5F, 0.05F))
      << "extra non-feature static point must survive in GlobalMap";
  EXPECT_FALSE(HasPoint(global, 7.0F, 0.05F, 0.05F, 0.2F))
      << "target cleared by later evidence must be absent";
  EXPECT_FALSE(HasPoint(global, 0.0F, 5.0F, 0.0F, 0.05F))
      << "feature-only points must not enter the complete-scan GlobalMap";

  Cloud corner;
  ASSERT_EQ(pcl::io::loadPCDFile(
                MapArtifactLayout(destination).CornerMap().string(), corner),
            0);
  const auto expected_corner = FilterExpected(
      node->cornerCloudKeyFrames[0], (*labels)->labels[0].corner);
  ASSERT_EQ(corner.size(), expected_corner.size());
  for (std::size_t index = 0; index < corner.size(); ++index) {
    EXPECT_FLOAT_EQ(corner[index].x, expected_corner[index].x + 2.0F);
    EXPECT_FLOAT_EQ(corner[index].y, expected_corner[index].y);
  }

  Cloud surf;
  ASSERT_EQ(pcl::io::loadPCDFile(
                MapArtifactLayout(destination).SurfaceMap().string(), surf),
            0);
  const auto expected_surf = FilterExpected(
      node->surfCloudKeyFrames[0], (*labels)->labels[0].surf);
  EXPECT_EQ(surf.size(), expected_surf.size());

  Cloud ground;
  ASSERT_EQ(pcl::io::loadPCDFile(
                MapArtifactLayout(destination).GroundMap().string(), ground),
            0);
  ASSERT_EQ(ground.size(), kSeededGroundPoints);
  for (const auto& point : ground) {
    EXPECT_GT(point.x, 1.9F);
  }
}

TEST(MapOptimizationSaveExport,
     MissingCompleteScanCacheFailsBeforeReplacingArtifacts) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.dufomap_enable = true;
  auto node = MakeExportNode(temp.path() / "cache", spec);
  // A legacy RingPlane ground PCD exists under scans/, but it is not a
  // complete keyframe scan and must never be used as a fallback.
  CacheSeed(*node);
  ASSERT_TRUE(fs::exists(node->CachedFullScanPath(0)));
  ASSERT_NE(SeedOnlineSnapshot(*node, {node->surfCloudKeyFrames[0]},
                               /*cache_complete_scan=*/false), nullptr);

  const fs::path destination = temp.path() / "map";
  const fs::path global = MapArtifactLayout(destination).GlobalMap();
  fs::create_directories(global.parent_path());
  { std::ofstream old(global); old << "previous global artifact\n"; }

  const MapExportOutcome outcome =
      ExportThroughOptionsService(*node, destination, false);
  EXPECT_FALSE(outcome.success);
  EXPECT_NE(outcome.message.find("complete keyframe scan cache"),
            std::string::npos)
      << outcome.message;
  ASSERT_TRUE(fs::exists(global));
  std::ifstream stream(global);
  const std::string contents((std::istreambuf_iterator<char>(stream)), {});
  EXPECT_EQ(contents, "previous global artifact\n");
}

TEST(MapOptimizationSaveExport,
     CorruptCompleteScanCacheFailsBeforeReplacingArtifacts) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.dufomap_enable = true;
  auto node = MakeExportNode(temp.path() / "cache", spec);
  ASSERT_NE(SeedOnlineSnapshot(*node, {node->surfCloudKeyFrames[0]}), nullptr);
  std::string cache_error;
  ASSERT_TRUE(CacheCompleteScan(*node, 0, node->surfCloudKeyFrames[0],
                                &cache_error))
      << cache_error;

  const fs::path destination = temp.path() / "map";
  ASSERT_TRUE(ExportThroughOptionsService(*node, destination, false).success);
  const fs::path global = MapArtifactLayout(destination).GlobalMap();
  Cloud before;
  ASSERT_EQ(pcl::io::loadPCDFile(global.string(), before), 0);
  ASSERT_FALSE(before.empty());

  {
    std::ofstream corrupt(node->CachedCompleteScanPath(0),
                          std::ios::binary | std::ios::trunc);
    corrupt << "not a complete keyframe scan\n";
  }
  const MapExportOutcome outcome =
      ExportThroughOptionsService(*node, destination, false);
  EXPECT_FALSE(outcome.success);
  EXPECT_NE(outcome.message.find("complete keyframe scan cache"),
            std::string::npos)
      << outcome.message;
  Cloud after;
  ASSERT_EQ(pcl::io::loadPCDFile(global.string(), after), 0);
  EXPECT_EQ(after.size(), before.size());
}

TEST(MapOptimizationSaveExport,
     DisabledDufomapIgnoresCompleteCacheState) {
  EnsureRosInitialized();
  TempDirectory temp;
  auto node = MakeBasicExportNode();
  EXPECT_TRUE(ExportThroughOptionsService(*node, temp.path() / "map", false)
                  .success);
  EXPECT_FALSE(fs::exists(node->CachedCompleteScanPath(0)));
}

TEST(MapOptimizationSaveExport,
     CompleteMapAppliesSaveOffsetAndResolutionAfterClassification) {
  EnsureRosInitialized();
  TempDirectory temp;
  ExportNodeSpec spec;
  spec.dufomap_enable = true;
  spec.dufomap_dp = 0;
  spec.align_ground_to_zero = true;
  spec.ground_z_offset = 7.0;
  auto node = MakeExportNode(temp.path() / "cache", spec);
  ASSERT_NE(SeedOnlineSnapshot(*node, {EvidenceObservingTarget(),
                                     DenseWall(10.0F)}), nullptr);
  const auto complete = MakePointCloud({{4.0F, 4.0F, 0.5F},
                                        {4.01F, 4.0F, 0.5F},
                                        {5.0F, 0.05F, 0.05F}});
  std::string error;
  ASSERT_TRUE(CacheCompleteScan(*node, 0, complete, &error)) << error;
  const auto destination = temp.path() / "map";
  const auto options = lio_sam_hesai::MakeSaveMapExportOptions(
      destination.string(), 0.1F, 0.0F, false, false, "/home/robot", &error);
  ASSERT_TRUE(options.has_value()) << error;
  const auto result = node->ExportMap(*options);
  ASSERT_TRUE(result.success) << result.message;
  Cloud global;
  ASSERT_EQ(pcl::io::loadPCDFile(
                MapArtifactLayout(destination).GlobalMap().string(), global), 0);
  ASSERT_EQ(global.size(), 1U);
  EXPECT_NEAR(global[0].x, 4.005F, 0.001F);
  EXPECT_FLOAT_EQ(global[0].y, 4.0F);
  EXPECT_FLOAT_EQ(global[0].z, 7.5F);
}
