#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "localization_map/global_map_provider.h"

namespace {

namespace fs = std::filesystem;
using localization_map::GlobalMapProvider;
using localization_map::GlobalMapProviderConfig;
using localization_map::MapMode;
using localization_map::MapRequestState;

class TemporaryDirectory {
 public:
  TemporaryDirectory() {
    static std::atomic<std::uint64_t> next_id{0};
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = fs::temp_directory_path() /
            ("global_map_provider_" + std::to_string(stamp) + "_" +
             std::to_string(next_id.fetch_add(1)));
    fs::create_directories(root_);
  }

  ~TemporaryDirectory() {
    std::error_code error;
    fs::remove_all(root_, error);
  }

  const fs::path& root() const { return root_; }

 private:
  fs::path root_;
};

PCLPointXYZI Point(float x, float y, float z, float intensity = 1.0F) {
  PCLPointXYZI point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  return point;
}

void SaveXYZI(const fs::path& path, const std::vector<PCLPointXYZI>& points) {
  PCLPointCloudXYZI cloud;
  cloud.points.assign(points.begin(), points.end());
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.height = 1U;
  cloud.is_dense = true;
  ASSERT_EQ(pcl::io::savePCDFileBinary(path.string(), cloud), 0);
}

void SaveXYZ(const fs::path& path, const std::vector<PCLPointXYZ>& points) {
  PCLPointCloudXYZ cloud;
  cloud.points.assign(points.begin(), points.end());
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.height = 1U;
  cloud.is_dense = true;
  ASSERT_EQ(pcl::io::savePCDFileBinary(path.string(), cloud), 0);
}

void SaveEmptyPcd(const fs::path& path) {
  std::ofstream output(path);
  output << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n"
            "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
            "WIDTH 0\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
            "POINTS 0\nDATA ascii\n";
}

Mat4d Pose(double x, double y, double z) {
  Mat4d pose = Mat4d::Identity();
  pose.block<3, 1>(0, 3) = Vec3d{x, y, z};
  return pose;
}

GlobalMapProviderConfig Config(const fs::path& global,
                               std::optional<fs::path> ground = std::nullopt) {
  GlobalMapProviderConfig config;
  config.global_map_path = global;
  config.ground_map_path = std::move(ground);
  config.global_voxel_size = 0.1F;
  return config;
}

bool HasPoint(const PCLPointCloudXYZI::ConstPtr& cloud, float x, float y,
              float z) {
  if (!cloud) {
    return false;
  }
  for (const auto& point : *cloud) {
    if (std::fabs(point.x - x) < 1e-4F &&
        std::fabs(point.y - y) < 1e-4F &&
        std::fabs(point.z - z) < 1e-4F) {
      return true;
    }
  }
  return false;
}

TEST(GlobalMapProvider, PreparesVoxelizedImmutableOverviewAndGround) {
  TemporaryDirectory directory;
  const auto global_path = directory.root() / "GlobalMap.pcd";
  const auto ground_path = directory.root() / "GroundMap.pcd";
  SaveXYZI(global_path, {Point(0, 0, 2, 7), Point(10, 0, 3, 8)});
  SaveXYZI(ground_path, {Point(0, 0, -1, 9)});

  GlobalMapProvider provider(Config(global_path, ground_path));
  std::string error = "stale";
  ASSERT_TRUE(provider.Prepare(&error));
  EXPECT_TRUE(error.empty());
  const auto overview = provider.Overview();
  ASSERT_NE(overview, nullptr);
  EXPECT_EQ(overview->size(), 2U);
  EXPECT_EQ(provider.Overview().get(), overview.get());

  const auto result = provider.LoadCandidate(Pose(0, 0, 0),
                                             std::chrono::milliseconds(0));
  ASSERT_EQ(result.state, MapRequestState::kReady);
  ASSERT_NE(result.snapshot, nullptr);
  EXPECT_EQ(result.snapshot->ground_cloud->size(), 1U);
  EXPECT_FLOAT_EQ(result.snapshot->ground_cloud->front().z, -1.0F);
  EXPECT_EQ(provider.Overview().get(), overview.get());
}

TEST(GlobalMapProvider, AcceptsLegacyXyzAndSynthesizesZeroIntensity) {
  TemporaryDirectory directory;
  const auto global_path = directory.root() / "legacy.pcd";
  PCLPointXYZ point;
  point.x = 1.0F;
  point.y = 2.0F;
  point.z = 3.0F;
  SaveXYZ(global_path, {point});

  GlobalMapProvider provider(Config(global_path));
  ASSERT_TRUE(provider.Prepare(nullptr));
  const auto overview = provider.Overview();
  ASSERT_NE(overview, nullptr);
  ASSERT_EQ(overview->size(), 1U);
  EXPECT_FLOAT_EQ(overview->front().intensity, 0.0F);
}

TEST(GlobalMapProvider, CandidateUsesExactFiftyMeterXyzBox) {
  TemporaryDirectory directory;
  const auto global_path = directory.root() / "global.pcd";
  SaveXYZI(global_path,
           {Point(50, 0, 0), Point(-50, -50, -50), Point(50.6F, 0, 0),
            Point(0, 50.6F, 0), Point(0, 0, 50.6F), Point(49, 49, 49)});
  GlobalMapProvider provider(Config(global_path));
  ASSERT_TRUE(provider.Prepare(nullptr));

  const auto result = provider.LoadCandidate(Pose(0, 0, 0),
                                             std::chrono::milliseconds(1));
  ASSERT_EQ(result.state, MapRequestState::kReady);
  ASSERT_NE(result.snapshot, nullptr);
  EXPECT_TRUE(HasPoint(result.snapshot->registration_cloud, 50, 0, 0));
  EXPECT_TRUE(HasPoint(result.snapshot->registration_cloud, -50, -50, -50));
  EXPECT_TRUE(HasPoint(result.snapshot->registration_cloud, 49, 49, 49));
  EXPECT_FALSE(HasPoint(result.snapshot->registration_cloud, 50.6F, 0, 0));
  EXPECT_FALSE(HasPoint(result.snapshot->registration_cloud, 0, 50.6F, 0));
  EXPECT_FALSE(HasPoint(result.snapshot->registration_cloud, 0, 0, 50.6F));
}

TEST(GlobalMapProvider, TrackingRefreshesAtExactFiftyAndKeepsOldSnapshotsAlive) {
  TemporaryDirectory directory;
  const auto global_path = directory.root() / "global.pcd";
  SaveXYZI(global_path,
           {Point(0, 0, 0), Point(100, 0, 0), Point(100.1F, 0, 0),
            Point(150, 0, 0)});
  GlobalMapProvider provider(Config(global_path));
  ASSERT_TRUE(provider.Prepare(nullptr));

  const auto first = provider.RequestTracking(Pose(0, 0, 0), std::nullopt,
                                              false);
  ASSERT_EQ(first.state, MapRequestState::kReady);
  ASSERT_NE(first.snapshot, nullptr);
  EXPECT_FALSE(first.previous_snapshot_covers_center);
  EXPECT_TRUE(HasPoint(first.snapshot->registration_cloud, 100, 0, 0));
  EXPECT_FALSE(HasPoint(first.snapshot->registration_cloud, 100.1F, 0, 0));

  const auto unchanged = provider.RequestTracking(Pose(49.999, 0, 0),
                                                  Vec2d{1.0, 0.0}, false);
  EXPECT_EQ(unchanged.state, MapRequestState::kUnchanged);
  EXPECT_EQ(unchanged.snapshot.get(), first.snapshot.get());
  EXPECT_TRUE(unchanged.previous_snapshot_covers_center);

  const auto exact = provider.RequestTracking(Pose(50, 0, 0), std::nullopt,
                                              false);
  ASSERT_EQ(exact.state, MapRequestState::kReady);
  ASSERT_NE(exact.snapshot, nullptr);
  EXPECT_NE(exact.snapshot.get(), first.snapshot.get());
  EXPECT_TRUE(exact.previous_snapshot_covers_center);
  EXPECT_EQ(first.snapshot->registration_cloud->size(), 2U);
  EXPECT_TRUE(HasPoint(exact.snapshot->registration_cloud, 150, 0, 0));

  const auto forced = provider.RequestTracking(Pose(50, 0, 0), std::nullopt,
                                               true);
  EXPECT_EQ(forced.state, MapRequestState::kReady);
  EXPECT_NE(forced.snapshot.get(), exact.snapshot.get());
}

TEST(GlobalMapProvider, CropsDedicatedGroundAndFallsBackForBadGroundFiles) {
  TemporaryDirectory directory;
  const auto global_path = directory.root() / "global.pcd";
  SaveXYZI(global_path, {Point(0, 0, 7), Point(150, 0, 8)});

  const auto dedicated_path = directory.root() / "ground.pcd";
  SaveXYZI(dedicated_path, {Point(0, 0, 1), Point(150, 0, 2)});
  GlobalMapProvider dedicated(Config(global_path, dedicated_path));
  ASSERT_TRUE(dedicated.Prepare(nullptr));
  auto result = dedicated.RequestTracking(Pose(0, 0, 0), std::nullopt, false);
  ASSERT_EQ(result.state, MapRequestState::kReady);
  ASSERT_EQ(result.snapshot->ground_cloud->size(), 1U);
  EXPECT_FLOAT_EQ(result.snapshot->ground_cloud->front().z, 1.0F);

  const auto missing_path = directory.root() / "missing.pcd";
  GlobalMapProvider missing(Config(global_path, missing_path));
  ASSERT_TRUE(missing.Prepare(nullptr));
  result = missing.LoadCandidate(Pose(0, 0, 0), std::chrono::milliseconds(1));
  ASSERT_EQ(result.state, MapRequestState::kReady);
  ASSERT_EQ(result.snapshot->ground_cloud->size(), 1U);
  EXPECT_FLOAT_EQ(result.snapshot->ground_cloud->front().z, 7.0F);

  const auto empty_path = directory.root() / "empty.pcd";
  SaveEmptyPcd(empty_path);
  GlobalMapProvider empty(Config(global_path, empty_path));
  ASSERT_TRUE(empty.Prepare(nullptr));
  result = empty.LoadCandidate(Pose(0, 0, 0), std::chrono::milliseconds(1));
  ASSERT_EQ(result.state, MapRequestState::kReady);
  EXPECT_FLOAT_EQ(result.snapshot->ground_cloud->front().z, 7.0F);

  const auto invalid_path = directory.root() / "invalid.pcd";
  { std::ofstream output(invalid_path); output << "not a pcd\n"; }
  GlobalMapProvider invalid(Config(global_path, invalid_path));
  ASSERT_TRUE(invalid.Prepare(nullptr));
  result = invalid.LoadCandidate(Pose(0, 0, 0), std::chrono::milliseconds(1));
  ASSERT_EQ(result.state, MapRequestState::kReady);
  EXPECT_FLOAT_EQ(result.snapshot->ground_cloud->front().z, 7.0F);
}

TEST(GlobalMapProvider, FailedPrepareIsStableUnavailableAndCanRetry) {
  TemporaryDirectory directory;
  const auto global_path = directory.root() / "global.pcd";
  GlobalMapProvider provider(Config(global_path));
  std::string first_error;
  EXPECT_FALSE(provider.Prepare(&first_error));
  EXPECT_FALSE(first_error.empty());
  EXPECT_EQ(provider.Overview(), nullptr);
  auto diagnostics = provider.Diagnostics();
  EXPECT_EQ(diagnostics.mode, MapMode::kGlobal);
  EXPECT_EQ(diagnostics.load_state, "error");
  EXPECT_EQ(diagnostics.last_error, first_error);
  EXPECT_EQ(diagnostics.active_tiles, 0U);
  EXPECT_EQ(diagnostics.cached_tiles, 0U);

  const auto unavailable = provider.LoadCandidate(
      Pose(0, 0, 0), std::chrono::milliseconds(10));
  EXPECT_EQ(unavailable.state, MapRequestState::kUnavailable);
  EXPECT_EQ(unavailable.error, first_error);
  EXPECT_EQ(provider.RequestTracking(Pose(0, 0, 0), std::nullopt, false).state,
            MapRequestState::kUnavailable);

  SaveXYZI(global_path, {Point(0, 0, 0)});
  std::string retry_error = "stale";
  EXPECT_TRUE(provider.Prepare(&retry_error));
  EXPECT_TRUE(retry_error.empty());
  EXPECT_EQ(provider.Diagnostics().load_state, "ready");
  EXPECT_TRUE(provider.Diagnostics().last_error.empty());
  EXPECT_TRUE(provider.Prepare(nullptr));
}

TEST(GlobalMapProvider, RejectsMissingEmptyInvalidGlobalAndBadVoxelConfig) {
  TemporaryDirectory directory;
  const auto missing = directory.root() / "missing.pcd";
  GlobalMapProvider missing_provider(Config(missing));
  EXPECT_FALSE(missing_provider.Prepare(nullptr));

  const auto empty = directory.root() / "empty.pcd";
  SaveEmptyPcd(empty);
  GlobalMapProvider empty_provider(Config(empty));
  EXPECT_FALSE(empty_provider.Prepare(nullptr));

  const auto invalid = directory.root() / "invalid.pcd";
  { std::ofstream output(invalid); output << "invalid\n"; }
  GlobalMapProvider invalid_provider(Config(invalid));
  EXPECT_FALSE(invalid_provider.Prepare(nullptr));

  const auto valid = directory.root() / "valid.pcd";
  SaveXYZI(valid, {Point(0, 0, 0)});
  auto bad_config = Config(valid);
  bad_config.global_voxel_size = std::numeric_limits<float>::quiet_NaN();
  GlobalMapProvider bad_voxel(std::move(bad_config));
  EXPECT_FALSE(bad_voxel.Prepare(nullptr));
}

TEST(GlobalMapProvider, RejectsNonFinitePosesWithoutReplacingCurrentSnapshot) {
  TemporaryDirectory directory;
  const auto global_path = directory.root() / "global.pcd";
  SaveXYZI(global_path, {Point(0, 0, 0)});
  GlobalMapProvider provider(Config(global_path));
  ASSERT_TRUE(provider.Prepare(nullptr));
  const auto ready = provider.RequestTracking(Pose(0, 0, 0), std::nullopt,
                                              false);
  ASSERT_EQ(ready.state, MapRequestState::kReady);

  Mat4d invalid = Mat4d::Identity();
  invalid(0, 3) = std::numeric_limits<double>::infinity();
  const auto tracking = provider.RequestTracking(invalid, std::nullopt, false);
  EXPECT_EQ(tracking.state, MapRequestState::kUnavailable);
  EXPECT_EQ(tracking.snapshot, nullptr);
  const auto candidate = provider.LoadCandidate(invalid,
                                                std::chrono::milliseconds(0));
  EXPECT_EQ(candidate.state, MapRequestState::kUnavailable);

  const auto unchanged = provider.RequestTracking(Pose(0, 0, 0), std::nullopt,
                                                  false);
  EXPECT_EQ(unchanged.state, MapRequestState::kUnchanged);
  EXPECT_EQ(unchanged.snapshot.get(), ready.snapshot.get());
}

TEST(GlobalMapProvider, CandidateRejectsEveryTranslationOutsideFloatRange) {
  TemporaryDirectory directory;
  const auto global_path = directory.root() / "global.pcd";
  SaveXYZI(global_path, {Point(0, 0, 0)});
  GlobalMapProvider provider(Config(global_path));
  ASSERT_TRUE(provider.Prepare(nullptr));

  const double outside =
      static_cast<double>(std::numeric_limits<float>::max()) * 2.0;
  std::string stable_error;
  for (int axis = 0; axis < 3; ++axis) {
    for (const double sign : {-1.0, 1.0}) {
      Mat4d pose = Mat4d::Identity();
      pose(axis, 3) = sign * outside;
      ASSERT_TRUE(pose.allFinite());
      const auto result = provider.LoadCandidate(
          pose, std::chrono::milliseconds(0));
      EXPECT_EQ(result.state, MapRequestState::kUnavailable);
      EXPECT_EQ(result.snapshot, nullptr);
      EXPECT_FALSE(result.error.empty());
      if (stable_error.empty()) {
        stable_error = result.error;
      } else {
        EXPECT_EQ(result.error, stable_error);
      }
    }
  }
}

TEST(GlobalMapProvider,
     TrackingRejectsEveryTranslationOutsideFloatRangeAndPreservesSnapshot) {
  TemporaryDirectory directory;
  const auto global_path = directory.root() / "global.pcd";
  SaveXYZI(global_path, {Point(0, 0, 0)});
  GlobalMapProvider provider(Config(global_path));
  ASSERT_TRUE(provider.Prepare(nullptr));
  const auto ready = provider.RequestTracking(Pose(0, 0, 0), std::nullopt,
                                              false);
  ASSERT_EQ(ready.state, MapRequestState::kReady);

  const double outside =
      static_cast<double>(std::numeric_limits<float>::max()) * 2.0;
  std::string stable_error;
  for (int axis = 0; axis < 3; ++axis) {
    for (const double sign : {-1.0, 1.0}) {
      Mat4d pose = Mat4d::Identity();
      pose(axis, 3) = sign * outside;
      ASSERT_TRUE(pose.allFinite());
      const auto result =
          provider.RequestTracking(pose, std::nullopt, false);
      EXPECT_EQ(result.state, MapRequestState::kUnavailable);
      EXPECT_EQ(result.snapshot, nullptr);
      EXPECT_FALSE(result.error.empty());
      if (stable_error.empty()) {
        stable_error = result.error;
      } else {
        EXPECT_EQ(result.error, stable_error);
      }

      const auto unchanged = provider.RequestTracking(
          Pose(0, 0, 0), std::nullopt, false);
      EXPECT_EQ(unchanged.state, MapRequestState::kUnchanged);
      EXPECT_EQ(unchanged.snapshot.get(), ready.snapshot.get());
    }
  }
}

TEST(GlobalMapProvider, ShutdownIsIdempotentNoOp) {
  TemporaryDirectory directory;
  const auto global_path = directory.root() / "global.pcd";
  SaveXYZI(global_path, {Point(0, 0, 0)});
  GlobalMapProvider provider(Config(global_path));
  ASSERT_TRUE(provider.Prepare(nullptr));
  provider.Shutdown();
  provider.Shutdown();
  EXPECT_NE(provider.Overview(), nullptr);
  EXPECT_EQ(provider.LoadCandidate(Pose(0, 0, 0),
                                   std::chrono::milliseconds(0)).state,
            MapRequestState::kReady);
}

}  // namespace
