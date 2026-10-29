#include "lio_sam/online_ground_pipeline.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
using namespace lio_sam_hesai;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;

pcl::PointXYZI Point(float x, float y, float z, float intensity) {
  pcl::PointXYZI point;
  point.x = x; point.y = y; point.z = z; point.intensity = intensity;
  return point;
}
Cloud Plane() {
  Cloud cloud;
  for (int row = 0; row < 4; ++row)
    for (int column = 0; column < 10; ++column)
      cloud.push_back(Point(5 + row, 0.3F * column, -1, row * 10 + column));
  return cloud;
}
void ExpectUnchanged(const Cloud& cloud, const Cloud& original) {
  ASSERT_EQ(cloud.size(), original.size());
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    EXPECT_FLOAT_EQ(cloud[i].x, original[i].x);
    EXPECT_FLOAT_EQ(cloud[i].y, original[i].y);
    EXPECT_FLOAT_EQ(cloud[i].z, original[i].z);
    EXPECT_FLOAT_EQ(cloud[i].intensity, original[i].intensity);
  }
}

TEST(OnlineGroundPipeline, DisabledSkipsBothAlgorithmsAndKeepsFeaturePoints) {
  GroundSegmentationConfig current; current.mode = "off";
  PatchworkppConfig patchwork; patchwork.mode = "export";
  int constructions = 0;
  OnlineGroundPipeline pipeline(current, patchwork, [&]() {
    ++constructions;
    throw std::runtime_error("must not construct offline Patchwork++ online");
    return OnlineGroundPipeline::SegmentFunction{};
  });
  const auto cloud = Plane();
  const auto before = cloud;
  // An absent current-validity mask would fail if current were executed.
  const auto result = pipeline.Segment(cloud, {}, 4, 10, {}, {});
  EXPECT_EQ(pipeline.Algorithm(), GroundAlgorithm::None);
  EXPECT_EQ(constructions, 0);
  EXPECT_TRUE(result.ground.empty());
  ASSERT_EQ(result.classification.size(), cloud.size());
  EXPECT_TRUE(std::all_of(result.classification.begin(), result.classification.end(),
                          [](auto label) { return label == 0; }));
  ExpectUnchanged(cloud, before);
}

TEST(OnlineGroundPipeline, CurrentPriorityAvoidsEvenConstructingPatchwork) {
  GroundSegmentationConfig current;
  current.mode = "online"; current.scan_start_index = 0;
  current.scan_end_index = 1; current.plane_min_points = 3;
  PatchworkppConfig patchwork; patchwork.mode = "online";
  int constructions = 0;
  OnlineGroundPipeline pipeline(current, patchwork, [&]() {
    ++constructions;
    throw std::runtime_error("lower-priority algorithm must not be constructed");
    return OnlineGroundPipeline::SegmentFunction{};
  });
  const auto cloud = Plane();
  const auto before = cloud;
  const auto result = pipeline.Segment(cloud, std::vector<std::uint8_t>(40, 1),
                                       4, 10, {}, {});
  EXPECT_EQ(pipeline.Algorithm(), GroundAlgorithm::Current);
  EXPECT_EQ(constructions, 0);
  EXPECT_EQ(result.ground.size(), 20U);
  EXPECT_TRUE(std::all_of(result.classification.begin(), result.classification.end(),
                          [](auto label) { return label == 1; }));
  ExpectUnchanged(cloud, before);
}

TEST(OnlineGroundPipeline, CompleteReturnClassificationMapsOnlyOriginalProjectionCells) {
  GroundSegmentationConfig current; current.mode = "export";
  PatchworkppConfig patchwork; patchwork.mode = "online";
  int constructions = 0, calls = 0;
  Cloud complete;
  complete.push_back(Point(1, 0, -1, 11));
  complete.push_back(Point(2, 0, 1, 22));
  complete.push_back(Point(3, 0, -1, 33));
  complete.push_back(Point(4, 0, -1, 44));
  const auto before = complete;
  OnlineGroundPipeline pipeline(current, patchwork, [&]() {
    ++constructions;
    return [&](const Cloud& input) {
      ++calls;
      EXPECT_EQ(&input, &complete);
      return std::vector<std::size_t>{0, 2, 3};
    };
  });
  const auto features = Plane();
  const auto feature_before = features;
  const auto result = pipeline.Segment(features, {}, 4, 10, complete, {0, 1, 5, -1});
  EXPECT_EQ(constructions, 1);
  EXPECT_EQ(calls, 1);
  ASSERT_EQ(result.ground.size(), 3U);
  EXPECT_FLOAT_EQ(result.ground[2].intensity, 44);
  EXPECT_EQ(result.classification[0], 1);
  EXPECT_EQ(result.classification[5], 1);
  EXPECT_EQ(std::count(result.classification.begin(), result.classification.end(), 1), 2);
  ExpectUnchanged(features, feature_before);
  ExpectUnchanged(complete, before);
}

TEST(OnlineGroundPipeline, RealPatchworkClassifiesPlaneAndWallWithoutChangingFeatureCloud) {
  GroundSegmentationConfig current; current.mode = "off";
  PatchworkppConfig patchwork; patchwork.mode = "online"; patchwork.sensor_height = 1;
  Cloud complete;
  for (int x = -40; x <= 40; ++x)
    for (int y = -40; y <= 40; ++y)
      complete.push_back(Point(0.25F * x, 0.25F * y,
                               -1 + 0.002F * std::sin(float(x + 3 * y)), 1));
  const auto plane_count = complete.size();
  for (int y = -15; y <= 15; ++y)
    for (int z = 0; z <= 15; ++z)
      complete.push_back(Point(6, 0.1F * y, 0.1F * z, 99));
  const auto before = complete;
  std::vector<int> cells(complete.size());
  for (std::size_t i = 0; i < cells.size(); ++i) cells[i] = int(i);
  OnlineGroundPipeline pipeline(current, patchwork);
  const auto result = pipeline.Segment(complete, {}, 1, int(complete.size()), complete, cells);
  EXPECT_GT(result.ground.size(), plane_count * 0.85);
  EXPECT_TRUE(std::all_of(result.ground.begin(), result.ground.end(),
                          [](const auto& point) { return point.intensity == 1; }));
  ExpectUnchanged(complete, before);
}

TEST(CompleteGroundScan, KeepsUnprojectedReturnsAndUsesEstablishedDeskewReference) {
  Cloud raw;
  raw.push_back(Point(1, 0, 0, 10));
  raw.push_back(Point(2, 0, 0, 20));
  raw.push_back(Point(std::numeric_limits<float>::quiet_NaN(), 0, 0, 30));
  raw.push_back(Point(4, 0, 0, 40));
  std::vector<std::size_t> calls;
  const auto result = MakeCompleteGroundScan(raw, {3, -1, -1, 9}, true,
      [&](pcl::PointXYZI point, std::size_t index) {
        calls.push_back(index); point.x += 0.5F; return point;
      });
  EXPECT_EQ(calls, (std::vector<std::size_t>{0, 1, 3}));
  EXPECT_EQ(result.projection_cells, (std::vector<int>{3, -1, 9}));
  ASSERT_EQ(result.points.size(), 3U);
  EXPECT_FLOAT_EQ(result.points[1].x, 2.5F);
  EXPECT_FLOAT_EQ(result.points[1].intensity, 20);
  EXPECT_FLOAT_EQ(raw[1].x, 2);
}

TEST(CompleteGroundScan, NeverEstablishesReferenceAndRejectsMismatchedMapping) {
  const auto raw = Plane();
  const auto result = MakeCompleteGroundScan(raw, std::vector<int>(raw.size(), -1), false,
      [](pcl::PointXYZI, std::size_t) -> pcl::PointXYZI {
        throw std::runtime_error("must not initialize the LOAM deskew reference");
      });
  ExpectUnchanged(result.points, raw);
  EXPECT_THROW(MakeCompleteGroundScan(raw, {}, true,
      [](pcl::PointXYZI point, std::size_t) { return point; }), std::invalid_argument);
}

TEST(OnlineGroundPipeline, RejectsInvalidBackendIndicesAndMappings) {
  GroundSegmentationConfig current; current.mode = "off";
  PatchworkppConfig patchwork; patchwork.mode = "online";
  const auto cloud = Plane();
  OnlineGroundPipeline bad_index(current, patchwork, []() {
    return [](const Cloud&) { return std::vector<std::size_t>{100}; };
  });
  EXPECT_THROW(bad_index.Segment(cloud, {}, 4, 10, cloud, std::vector<int>(40, 0)),
               std::runtime_error);
  OnlineGroundPipeline bad_cell(current, patchwork, []() {
    return [](const Cloud&) { return std::vector<std::size_t>{0}; };
  });
  EXPECT_THROW(bad_cell.Segment(cloud, {}, 4, 10, cloud, {}), std::invalid_argument);
  EXPECT_THROW(bad_cell.Segment(cloud, {}, 4, 10, cloud, std::vector<int>(40, 100)),
               std::invalid_argument);
}
}  // namespace
