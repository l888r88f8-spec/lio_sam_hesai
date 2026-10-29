#include "lio_sam/patchworkpp_ground_segmenter.h"
#include "common/slam_config.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using lio_sam_hesai::PatchworkppGroundSegmenter;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;

pcl::PointXYZI Point(float x, float y, float z, float intensity = 42.0F)
{
  pcl::PointXYZI point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  return point;
}

Cloud Plane(float slope = 0.0F)
{
  Cloud cloud;
  for (int x = -50; x <= 50; ++x)
    for (int y = -50; y <= 50; ++y)
    {
      const float px = x * 0.2F;
      const float py = y * 0.2F;
      if (std::hypot(px, py) <= 0.6F) continue;
      cloud.push_back(Point(px, py, -1.723F + slope * px,
                            static_cast<float>(cloud.size()) * 0.25F));
    }
  return cloud;
}

std::vector<bool> Mask(const Cloud& cloud, const std::vector<std::size_t>& indices)
{
  std::vector<bool> selected(cloud.size(), false);
  EXPECT_TRUE(std::is_sorted(indices.begin(), indices.end()));
  EXPECT_TRUE(std::adjacent_find(indices.begin(), indices.end()) == indices.end());
  for (const auto index : indices)
  {
    EXPECT_LT(index, cloud.size());
    if (index < cloud.size()) selected[index] = true;
  }
  return selected;
}

TEST(PatchworkppGroundSegmenterTest, FlatGroundIsSelectedAndBoxAndVerticalWallAreRejected)
{
  PatchworkppConfig config;
  Cloud cloud = Plane();
  const auto floor_count = cloud.size();
  for (int x = 0; x <= 16; ++x)
    for (int y = 0; y <= 16; ++y)
      cloud.push_back(Point(3.0F + .125F * x, 2.0F + .125F * y, -.723F));
  for (int y = -32; y <= 32; ++y)
    for (int z = 0; z <= 20; ++z)
      cloud.push_back(Point(-4.0F, y * .125F, -1.473F + z * .125F));

  PatchworkppGroundSegmenter segmenter(config);
  const auto mask = Mask(cloud, segmenter.Segment(cloud));
  const auto floor_selected = std::count(mask.begin(), mask.begin() + floor_count, true);
  const auto obstacles_selected = std::count(mask.begin() + floor_count, mask.end(), true);
  RecordProperty("flat_ground_total", static_cast<int>(floor_count));
  RecordProperty("flat_ground_selected", static_cast<int>(floor_selected));
  RecordProperty("obstacle_total", static_cast<int>(cloud.size() - floor_count));
  RecordProperty("obstacle_selected", static_cast<int>(obstacles_selected));
  EXPECT_GT(floor_selected, .95 * floor_count);
  EXPECT_LT(obstacles_selected, .02 * (cloud.size() - floor_count));
}

TEST(PatchworkppGroundSegmenterTest, SlopedGroundIsSelectedAcrossSequentialFrames)
{
  PatchworkppGroundSegmenter segmenter(PatchworkppConfig{});
  // An approximately 8.5-degree plane, without an assumed horizontal z cutoff.
  for (int frame = 0; frame < 3; ++frame)
  {
    auto cloud = Plane(.15F);
    for (auto& point : cloud.points) point.z += .02F * frame;
    const auto indices = segmenter.Segment(cloud);
    RecordProperty("slope_ground_total", static_cast<int>(cloud.size()));
    RecordProperty("slope_ground_frame_" + std::to_string(frame), static_cast<int>(indices.size()));
    EXPECT_GT(indices.size(), .90 * cloud.size()) << "frame " << frame;
    Mask(cloud, indices);
  }
}

TEST(PatchworkppGroundSegmenterTest, OriginalIndicesDuplicatesAndEveryXyziBitArePreserved)
{
  auto cloud = Plane();
  cloud.push_back(Point(4.0F, 2.0F, -1.723F, 1000001.0F));
  cloud.push_back(Point(4.0F, 2.0F, -1.723F, 2000002.0F));
  cloud.push_back(Point(std::numeric_limits<float>::quiet_NaN(), 1.0F, 0.0F));
  cloud.push_back(Point(1.0F, std::numeric_limits<float>::infinity(), 0.0F));
  cloud.push_back(Point(1.0F, 1.0F, -std::numeric_limits<float>::infinity()));
  std::mt19937 random(7);
  std::shuffle(cloud.points.begin(), cloud.points.end(), random);
  cloud.header.frame_id = "lidar_test";
  cloud.header.stamp = 123456;
  cloud.is_dense = false;
  const Cloud before = cloud;

  PatchworkppGroundSegmenter segmenter(PatchworkppConfig{});
  const auto selected = Mask(cloud, segmenter.Segment(cloud));
  std::size_t duplicates = 0;
  for (std::size_t i = 0; i < cloud.size(); ++i)
  {
    for (int field = 0; field < 3; ++field)
      EXPECT_EQ(std::memcmp(&cloud.points[i].data[field], &before.points[i].data[field], sizeof(float)), 0);
    EXPECT_EQ(std::memcmp(&cloud.points[i].intensity, &before.points[i].intensity, sizeof(float)), 0);
    const auto& point = cloud.points[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      EXPECT_FALSE(selected[i]);
    if (point.intensity == 1000001.0F || point.intensity == 2000002.0F)
    {
      EXPECT_TRUE(selected[i]);
      ++duplicates;
    }
  }
  EXPECT_EQ(duplicates, 2U);
  EXPECT_EQ(cloud.width, before.width);
  EXPECT_EQ(cloud.height, before.height);
  EXPECT_EQ(cloud.header.frame_id, before.header.frame_id);
  EXPECT_EQ(cloud.header.stamp, before.header.stamp);
  EXPECT_EQ(cloud.is_dense, before.is_dense);
}

TEST(PatchworkppGroundSegmenterTest, EmptyInvalidAndSparseCloudsReturnNoStaleIndices)
{
  PatchworkppGroundSegmenter segmenter(PatchworkppConfig{});
  PatchworkppGroundSegmenter uninterrupted(PatchworkppConfig{});
  const auto floor = Plane();
  const auto first = segmenter.Segment(floor);
  EXPECT_GT(first.size(), .95 * floor.size());
  EXPECT_EQ(first, uninterrupted.Segment(floor));
  EXPECT_TRUE(segmenter.Segment(Cloud{}).empty());
  Cloud invalid;
  invalid.push_back(Point(std::numeric_limits<float>::quiet_NaN(), 1.0F, 0.0F));
  invalid.push_back(Point(1.0F, 1.0F, std::numeric_limits<float>::infinity()));
  EXPECT_TRUE(segmenter.Segment(invalid).empty());
  Cloud sparse;
  sparse.push_back(Point(5.0F, 0.0F, -1.723F));
  EXPECT_TRUE(segmenter.Segment(sparse).empty());
  EXPECT_TRUE(uninterrupted.Segment(sparse).empty());
  // Exact zero-noise planes can have reduced second-frame recall in upstream
  // adaptive thresholding. Empty/invalid calls must preserve the same state
  // as a stream without those calls, rather than resetting that algorithm.
  const auto after_empty = segmenter.Segment(floor);
  EXPECT_FALSE(after_empty.empty());
  EXPECT_EQ(after_empty, uninterrupted.Segment(floor));
}

TEST(PatchworkppGroundSegmenterTest, ConfiguredRadialRangeExcludesOutOfRangePoints)
{
  PatchworkppConfig config;
  config.min_range = 2.0;
  config.max_range = 8.0;
  const auto cloud = Plane();
  PatchworkppGroundSegmenter segmenter(config);
  const auto indices = segmenter.Segment(cloud);
  ASSERT_GT(indices.size(), 1000U);
  for (const auto index : indices)
  {
    const auto& point = cloud.points.at(index);
    EXPECT_GT(std::hypot(point.x, point.y), config.min_range);
    EXPECT_LE(std::hypot(point.x, point.y), config.max_range);
  }
}

TEST(PatchworkppGroundSegmenterTest, NonfiniteIntensityIsPreservedWhenRnrIsDisabled)
{
  auto cloud = Plane();
  cloud.push_back(Point(4.0F, 2.0F, -1.723F, std::numeric_limits<float>::quiet_NaN()));
  PatchworkppGroundSegmenter segmenter(PatchworkppConfig{});
  const auto selected = Mask(cloud, segmenter.Segment(cloud));
  EXPECT_TRUE(selected.back());
  EXPECT_TRUE(std::isnan(cloud.back().intensity));
}

TEST(PatchworkppGroundSegmenterTest, RnrRejectsNonfiniteIntensityWithClearFailure)
{
  PatchworkppConfig config;
  config.enable_rnr = true;
  auto cloud = Plane();
  cloud.back().intensity = std::numeric_limits<float>::quiet_NaN();
  PatchworkppGroundSegmenter segmenter(config);
  try
  {
    segmenter.Segment(cloud);
    FAIL() << "RNR must require finite intensity";
  }
  catch (const std::invalid_argument& error)
  {
    EXPECT_NE(std::string(error.what()).find("intensity"), std::string::npos);
  }
}

TEST(PatchworkppGroundSegmenterTest, RejectsInvalidParametersBeforeCallingTheAlgorithm)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::array<double PatchworkppConfig::*, 7> fields = {
      &PatchworkppConfig::sensor_height, &PatchworkppConfig::min_range,
      &PatchworkppConfig::max_range, &PatchworkppConfig::uprightness_thr,
      &PatchworkppConfig::th_seeds, &PatchworkppConfig::th_dist,
      &PatchworkppConfig::rnr_intensity_threshold};
  for (const auto field : fields)
    for (const double value : {nan, inf, -1.0})
    {
      PatchworkppConfig config;
      config.*field = value;
      EXPECT_THROW(PatchworkppGroundSegmenter{config}, std::invalid_argument);
    }
  PatchworkppConfig config;
  config.max_range = config.min_range;
  EXPECT_THROW(PatchworkppGroundSegmenter{config}, std::invalid_argument);
  config = PatchworkppConfig{};
  config.uprightness_thr = 1.1;
  EXPECT_THROW(PatchworkppGroundSegmenter{config}, std::invalid_argument);
  for (const auto field : {&PatchworkppConfig::sensor_height, &PatchworkppConfig::th_seeds,
                           &PatchworkppConfig::th_dist, &PatchworkppConfig::uprightness_thr})
  {
    config = PatchworkppConfig{};
    config.*field = 0.0;
    EXPECT_THROW(PatchworkppGroundSegmenter{config}, std::invalid_argument);
  }
}
}  // namespace
