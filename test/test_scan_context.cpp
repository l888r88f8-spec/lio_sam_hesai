#include "relocalization/scan_context.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <string>

#include <gtest/gtest.h>
#include <unistd.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

std::string TestPath(const std::string& suffix) {
  return "/tmp/lio_sam_hesai_scan_context_" +
         std::to_string(static_cast<long long>(getpid())) + suffix;
}

pcl::PointCloud<pcl::PointXYZI> MakeRingCloud(double radius, double z,
                                               int num_points) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.reserve(num_points);
  for (int i = 0; i < num_points; ++i) {
    const double theta = 2.0 * kPi * static_cast<double>(i) / num_points;
    pcl::PointXYZI point;
    point.x = static_cast<float>(radius * std::cos(theta));
    point.y = static_cast<float>(radius * std::sin(theta));
    point.z = static_cast<float>(z);
    cloud.push_back(point);
  }
  return cloud;
}

pcl::PointCloud<pcl::PointXYZI> MakeAsymmetricCloud(double yaw_degrees,
                                                     int scene_id = 0) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  const double yaw = yaw_degrees * kPi / 180.0;
  for (int sector = 0; sector < 60; ++sector) {
    const double source_angle = 2.0 * kPi * (sector + 0.5) / 60.0;
    const double angle = source_angle + yaw;
    const double radius = 12.0 + scene_id * 5.0 + (sector % 7);
    pcl::PointXYZI point;
    point.x = static_cast<float>(radius * std::cos(angle));
    point.y = static_cast<float>(radius * std::sin(angle));
    point.z = static_cast<float>(2.0 + scene_id +
                                 std::sin(source_angle) * 3.0 +
                                 0.15 * sector);
    cloud.push_back(point);
  }
  return cloud;
}

}  // namespace

TEST(ScanContextTest, DescriptorHasConfiguredShapeAndRingKeySize) {
  relocalization::ScanContextConfig config;
  config.num_ring = 20;
  config.num_sector = 60;
  relocalization::ScanContext scan_context(config);

  const auto descriptor =
      scan_context.MakeDescriptor(MakeRingCloud(30.0, 2.0, 720));
  const auto ring_key = relocalization::ScanContext::MakeRingKey(descriptor);

  EXPECT_EQ(descriptor.rows(), config.num_ring);
  EXPECT_EQ(descriptor.cols(), config.num_sector);
  EXPECT_EQ(ring_key.size(), config.num_ring);
}

TEST(ScanContextTest, RingKeyContainsPerRowMeans) {
  relocalization::ScanContextDescriptor descriptor(2, 3);
  descriptor << 0.0F, 1.0F, 2.0F, 3.0F, 3.0F, 3.0F;

  const auto ring_key = relocalization::ScanContext::MakeRingKey(descriptor);

  ASSERT_EQ(ring_key.size(), 2);
  EXPECT_FLOAT_EQ(ring_key(0), 1.0F);
  EXPECT_FLOAT_EQ(ring_key(1), 3.0F);
}

TEST(ScanContextTest, DistanceReportsPositiveYawShiftForRotatedScene) {
  relocalization::ScanContextConfig config;
  config.num_ring = 20;
  config.num_sector = 60;
  relocalization::ScanContextDescriptor database =
      relocalization::ScanContextDescriptor::Zero(config.num_ring,
                                                   config.num_sector);
  for (int column = 0; column < config.num_sector; ++column) {
    database(0, column) = 1.0F;
    database(1, column) = static_cast<float>(column + 1);
    database(2, column) = static_cast<float>((column + 3) * (column + 7));
  }

  // A sensor yawed +60 degrees sees the static scene rotated -60 degrees in
  // its local frame. The returned correction therefore needs to be +60.
  relocalization::ScanContextDescriptor query =
      relocalization::ScanContextDescriptor::Zero(config.num_ring,
                                                   config.num_sector);
  for (int column = 0; column < config.num_sector; ++column) {
    query.col(column) = database.col((column + 10) % config.num_sector);
  }
  const auto [distance, shift] =
      relocalization::ScanContext::DistanceBetween(query, database);

  EXPECT_NEAR(distance, 0.0F, 1.0e-6F);
  EXPECT_EQ(shift, 10);

  config.ringkey_max_dist = 10000.0;
  config.min_valid_points = 1;
  relocalization::ScanContext scan_context(config);
  scan_context.AddKeyFrame(database, Eigen::Matrix4d::Identity());
  const auto candidates = scan_context.Query(query, 1);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_NEAR(candidates.front().yaw_shift, kPi / 3.0, 1.0e-6);
}

TEST(ScanContextTest, DescriptorKeepsNegativeHeightAndRejectsInvalidPoints) {
  relocalization::ScanContextConfig config;
  config.num_ring = 10;
  config.num_sector = 20;
  config.max_radius = 20.0;
  config.height_offset = 2.0;
  relocalization::ScanContext scan_context(config);

  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI valid;
  valid.x = 10.0F;
  valid.y = 0.0F;
  valid.z = -1.25F;
  cloud.push_back(valid);
  pcl::PointXYZI too_far = valid;
  too_far.x = 25.0F;
  cloud.push_back(too_far);
  pcl::PointXYZI invalid = valid;
  invalid.x = std::numeric_limits<float>::quiet_NaN();
  cloud.push_back(invalid);
  pcl::PointXYZI invalid_intensity = valid;
  invalid_intensity.z = 100.0F;
  invalid_intensity.intensity = std::numeric_limits<float>::infinity();
  cloud.push_back(invalid_intensity);

  const auto descriptor = scan_context.MakeDescriptor(cloud);
  EXPECT_NEAR(descriptor(5, 0), 0.75F, 1.0e-6F);
  EXPECT_EQ(descriptor.sum(), 0.75F);
}

TEST(ScanContextTest, SaveLoadQueryAndCorruptDbPreserveExistingDatabase) {
  relocalization::ScanContextConfig config;
  config.ringkey_max_dist = 100.0;
  config.min_valid_points = 1;
  relocalization::ScanContext source(config);

  for (int scene_id = 0; scene_id < 3; ++scene_id) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose(0, 3) = static_cast<double>(scene_id) * 10.0;
    pose(1, 3) = static_cast<double>(scene_id) * -5.0;
    source.AddKeyFrame(
        source.MakeDescriptor(MakeAsymmetricCloud(0.0, scene_id)), pose);
  }

  const std::string path = TestPath(".bin");
  const std::string corrupt_path = TestPath(".corrupt.bin");
  ASSERT_TRUE(source.SaveDB(path, "unit-test-map"));

  relocalization::ScanContext loaded;
  std::string tag;
  ASSERT_TRUE(loaded.LoadDB(path, &tag));
  EXPECT_EQ(tag, "unit-test-map");
  EXPECT_EQ(loaded.Size(), 3U);
  EXPECT_DOUBLE_EQ(loaded.config().height_offset, config.height_offset);

  const auto query = loaded.MakeDescriptor(MakeAsymmetricCloud(0.0, 1));
  const auto candidates = loaded.Query(query, 3);
  ASSERT_FALSE(candidates.empty());
  EXPECT_NEAR(candidates.front().pose(0, 3), 10.0, 1.0e-6);

  {
    std::ofstream corrupt(corrupt_path, std::ios::binary | std::ios::trunc);
    corrupt << "not a scan-context database";
  }
  EXPECT_FALSE(loaded.LoadDB(corrupt_path));
  EXPECT_EQ(loaded.Size(), 3U);

  {
    std::ofstream appended(path, std::ios::binary | std::ios::app);
    appended.put('\x7f');
  }
  EXPECT_FALSE(loaded.LoadDB(path));
  EXPECT_EQ(loaded.Size(), 3U);

  std::remove(path.c_str());
  std::remove(corrupt_path.c_str());
}
