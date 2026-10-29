#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <string>
#include <utility>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lio_sam/ground_grid_processor.h"

namespace {

using Cloud = pcl::PointCloud<lio_sam_hesai::GroundGridPointType>;
using lio_sam_hesai::BuildGroundGrid;
using lio_sam_hesai::GroundGridExportOptions;

void AddPoint(Cloud* cloud, float x, float y, float z) {
  lio_sam_hesai::GroundGridPointType point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = 0.0F;
  cloud->push_back(point);
}

GroundGridExportOptions Options(float resolution) {
  GroundGridExportOptions options;
  options.resolution = resolution;
  options.search_radius = 2.0F * resolution;
  options.max_height_range = 0.12F;
  options.max_roughness = 0.05F;
  options.plane_distance_threshold = 0.12F;
  options.max_slope_deg = 25.0F;
  options.max_prediction_margin = 0.08F;
  options.min_plane_inlier_ratio = 0.70F;
  options.min_neighbor_cells = 6;
  options.min_support_quadrants = 3;
  options.fill_unobserved_gaps = false;
  return options;
}

std::pair<int, int> CellKey(float x, float y, float resolution) {
  return {static_cast<int>(std::floor(x / resolution)),
          static_cast<int>(std::floor(y / resolution))};
}

Cloud BuildCloud(float resolution, int min_cell, int max_cell, float z) {
  Cloud cloud;
  for (int ix = min_cell; ix <= max_cell; ++ix) {
    for (int iy = min_cell; iy <= max_cell; ++iy) {
      AddPoint(&cloud, (static_cast<float>(ix) + 0.5F) * resolution,
          (static_cast<float>(iy) + 0.5F) * resolution, z);
    }
  }
  return cloud;
}

const lio_sam_hesai::GroundGridPointType* FindPoint(const Cloud& cloud,
    int ix, int iy,
    float resolution) {
  for (const auto& point : cloud.points) {
    if (CellKey(point.x, point.y, resolution) == std::make_pair(ix, iy)) {
      return &point;
    }
  }
  return nullptr;
}

Cloud BuildStepCloud(float resolution, int min_cell, int max_cell) {
  Cloud cloud;
  for (int ix = min_cell; ix <= max_cell; ++ix) {
    for (int iy = min_cell; iy <= max_cell; ++iy) {
      AddPoint(&cloud, (static_cast<float>(ix) + 0.5F) * resolution,
          (static_cast<float>(iy) + 0.5F) * resolution,
          ix < 0 ? 0.0F : 0.3F);
    }
  }
  return cloud;
}

Cloud BuildSlopedCloud(float resolution, int min_cell, int max_cell,
    float slope_x, float slope_y, float offset) {
  Cloud cloud;
  for (int ix = min_cell; ix <= max_cell; ++ix) {
    for (int iy = min_cell; iy <= max_cell; ++iy) {
      const float x = (static_cast<float>(ix) + 0.5F) * resolution;
      const float y = (static_cast<float>(iy) + 0.5F) * resolution;
      AddPoint(&cloud, x, y, slope_x * x + slope_y * y + offset);
    }
  }
  return cloud;
}

Cloud RemoveCells(const Cloud& input, float resolution,
    int min_ix, int max_ix, int min_iy, int max_iy) {
  Cloud output;
  for (const auto& point : input.points) {
    const auto key = CellKey(point.x, point.y, resolution);
    if (key.first >= min_ix && key.first <= max_ix &&
        key.second >= min_iy && key.second <= max_iy) {
      continue;
    }
    output.push_back(point);
  }
  return output;
}

void ExpectHoleEmpty(const Cloud& output, float resolution,
    int min_ix, int max_ix, int min_iy, int max_iy) {
  for (int ix = min_ix; ix <= max_ix; ++ix) {
    for (int iy = min_iy; iy <= max_iy; ++iy) {
      EXPECT_EQ(FindPoint(output, ix, iy, resolution), nullptr)
          << ix << "," << iy;
    }
  }
}

TEST(GroundGridProcessor, PlacesOnePointAtExactCellCenter) {
  Cloud input;
  AddPoint(&input, -0.20F, -0.20F, 1.0F);
  AddPoint(&input, -0.10F, -0.10F, 1.1F);
  AddPoint(&input, 0.20F, 0.20F, 2.0F);

  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, Options(0.5F), &output, &error)) << error;
  ASSERT_EQ(output.size(), 2U);

  std::set<std::pair<int, int> > keys;
  for (const auto& point : output.points) {
    ASSERT_TRUE(std::isfinite(point.x));
    ASSERT_TRUE(std::isfinite(point.y));
    ASSERT_TRUE(std::isfinite(point.z));
    const auto key = CellKey(point.x, point.y, 0.5F);
    keys.insert(key);
    EXPECT_FLOAT_EQ(point.x, (static_cast<float>(key.first) + 0.5F) * 0.5F);
    EXPECT_FLOAT_EQ(point.y, (static_cast<float>(key.second) + 0.5F) * 0.5F);
  }
  EXPECT_EQ(keys.size(), output.size());
  EXPECT_EQ(keys.count({-1, -1}), 1U);
  EXPECT_EQ(keys.count({0, 0}), 1U);
}

TEST(GroundGridProcessor, PreservesSlopeAtCellCenters) {
  Cloud input;
  for (int ix = -15; ix <= 19; ++ix) {
    for (int iy = -15; iy <= 19; ++iy) {
      const float x = 0.1F * static_cast<float>(ix);
      const float y = 0.1F * static_cast<float>(iy);
      AddPoint(&input, x, y, 0.2F * x + 0.1F * y + 0.3F);
    }
  }

  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, Options(0.5F), &output, &error)) << error;
  ASSERT_FALSE(output.empty());

  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();
  for (const auto& point : output.points) {
    const auto key = CellKey(point.x, point.y, 0.5F);
    EXPECT_FLOAT_EQ(point.x,
        (static_cast<float>(key.first) + 0.5F) * 0.5F);
    EXPECT_FLOAT_EQ(point.y,
        (static_cast<float>(key.second) + 0.5F) * 0.5F);
    EXPECT_NEAR(point.z, 0.2F * point.x + 0.1F * point.y + 0.3F, 0.02F);
    min_z = std::min(min_z, point.z);
    max_z = std::max(max_z, point.z);
  }
  EXPECT_GT(max_z - min_z, 0.4F);
}

TEST(GroundGridProcessor, StepTerrainKeepsMeasuredHeights) {
  const Cloud input = BuildStepCloud(0.5F, -3, 3);
  GroundGridExportOptions options = Options(0.5F);
  options.search_radius = 1.0F;

  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, options, &output, &error)) << error;

  const auto* low = FindPoint(output, -1, 0, 0.5F);
  const auto* high = FindPoint(output, 0, 0, 0.5F);
  ASSERT_NE(low, nullptr);
  ASSERT_NE(high, nullptr);
  EXPECT_NEAR(low->z, 0.0F, 1e-4F);
  EXPECT_NEAR(high->z, 0.3F, 1e-4F);
  for (const auto& point : output.points) {
    EXPECT_FALSE(point.z > 0.05F && point.z < 0.25F)
      << "fabricated slope at " << point.x << "," << point.y << ": "
      << point.z;
  }
}

TEST(GroundGridProcessor, DoesNotFillStepGapWithIntermediateHeight) {
  Cloud input = BuildStepCloud(0.5F, -3, 3);
  Cloud with_step_gap;
  for (const auto& point : input.points) {
    if (CellKey(point.x, point.y, 0.5F) == std::make_pair(0, 0)) {
      continue;
    }
    with_step_gap.push_back(point);
  }

  GroundGridExportOptions options = Options(0.5F);
  options.search_radius = 1.0F;
  options.fill_unobserved_gaps = true;
  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(with_step_gap, options, &output, &error))
    << error;

  EXPECT_EQ(FindPoint(output, 0, 0, 0.5F), nullptr);
  for (const auto& point : output.points) {
    EXPECT_FALSE(point.z > 0.05F && point.z < 0.25F)
      << "fabricated fill at " << point.x << "," << point.y << ": "
      << point.z;
  }
}

TEST(GroundGridProcessor, DoesNotFillHoleAcrossNarrowHeightBoundary) {
  Cloud input;
  for (const auto& point : BuildCloud(0.1F, -6, 6, 0.0F)) {
    const auto key = CellKey(point.x, point.y, 0.1F);
    if (key == std::make_pair(0, 0)) {
      continue;
    }
    auto measured = point;
    if (key == std::make_pair(1, 0)) {
      measured.z = 0.3F;
    }
    input.push_back(measured);
  }
  auto options = Options(0.1F);
  options.search_radius = 0.4F;
  options.fill_unobserved_gaps = true;
  Cloud output;
  ASSERT_TRUE(BuildGroundGrid(input, options, &output));
  EXPECT_EQ(FindPoint(output, 0, 0, 0.1F), nullptr);
}

TEST(GroundGridProcessor, ExactSlopeUsesSelectedLayerRepresentative) {
  Cloud input;
  for (int ix = -3; ix <= 3; ++ix) {
    for (int iy = -3; iy <= 3; ++iy) {
      const float y = (static_cast<float>(iy) + 0.5F) * 0.5F;
      for (int sample = 0; sample < 3; ++sample) {
        const float x = static_cast<float>(ix) * 0.5F + 0.01F;
        AddPoint(&input, x, y, 0.4F * x);
      }
      const float x = static_cast<float>(ix) * 0.5F + 0.29F;
      AddPoint(&input, x, y, 0.4F * x);
    }
  }

  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, Options(0.5F), &output, &error)) << error;

  const auto* center = FindPoint(output, 0, 0, 0.5F);
  ASSERT_NE(center, nullptr);
  EXPECT_NEAR(center->z, 0.1F, 1e-4F);
  for (const auto& point : output.points) {
    EXPECT_NEAR(point.z, 0.4F * point.x, 1e-4F);
  }
}

TEST(GroundGridProcessor, DoesNotMergeLayersThroughSingleBridgePoint) {
  Cloud input;
  for (const auto& point : BuildCloud(0.5F, -2, 2, 0.0F).points) {
    if (CellKey(point.x, point.y, 0.5F) == std::make_pair(0, 0)) {
      continue;
    }
    input.push_back(point);
  }
  for (const float z : {0.0F, 0.0F, 0.0F, 0.0F, 0.1F,
                        0.2F, 0.2F, 0.2F, 0.2F}) {
    AddPoint(&input, 0.25F, 0.25F, z);
  }

  GroundGridExportOptions options = Options(0.5F);
  options.fill_unobserved_gaps = true;
  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, options, &output, &error)) << error;

  EXPECT_EQ(FindPoint(output, 0, 0, 0.5F), nullptr);
}

TEST(GroundGridProcessor, PreservesSlopeSpanningMultipleHeightBands) {
  Cloud input;
  for (int ix = -3; ix <= 3; ++ix) {
    for (int iy = -3; iy <= 3; ++iy) {
      for (const float dx : {0.01F, 0.49F}) {
        for (const float dy : {0.01F, 0.49F}) {
          const float x = ix * 0.5F + dx;
          AddPoint(&input, x, iy * 0.5F + dy, 0.4F * x);
        }
      }
    }
  }
  Cloud output;
  ASSERT_TRUE(BuildGroundGrid(input, Options(0.5F), &output));
  ASSERT_EQ(output.size(), 49U);
  for (const auto& point : output) {
    EXPECT_NEAR(point.z, 0.4F * point.x, 1e-4F);
  }
}

TEST(GroundGridProcessor, DoesNotFlattenNarrowRaisedGround) {
  Cloud input = BuildCloud(0.1F, -6, 6, 0.0F);
  for (auto& point : input) {
    if (CellKey(point.x, point.y, 0.1F).first == 0) {
      point.z = 0.3F;
    }
  }
  auto options = Options(0.1F);
  options.search_radius = 0.4F;
  Cloud output;
  ASSERT_TRUE(BuildGroundGrid(input, options, &output));
  const auto* center = FindPoint(output, 0, 0, 0.1F);
  ASSERT_NE(center, nullptr);
  EXPECT_NEAR(center->z, 0.3F, 1e-4F);
}

TEST(GroundGridProcessor, ChecksSlopeResidualAtMeasuredCoordinates) {
  Cloud input;
  for (int ix = -3; ix <= 3; ++ix) {
    for (int iy = -3; iy <= 3; ++iy) {
      const float x = ix * 2.0F + 0.01F;
      AddPoint(&input, x, iy * 2.0F + 1.0F, 0.4F * x);
    }
  }
  Cloud output;
  ASSERT_TRUE(BuildGroundGrid(input, Options(2.0F), &output));
  const auto* center = FindPoint(output, 0, 0, 2.0F);
  ASSERT_NE(center, nullptr);
  EXPECT_NEAR(center->z, 0.4F, 1e-4F);
  ASSERT_EQ(output.size(), input.size());
  for (const auto& point : output) {
    EXPECT_NEAR(point.z, 0.4F * point.x, 1e-4F);
  }
}

TEST(GroundGridProcessor, RepeatedFrameJitterStaysOneLayer) {
  Cloud input;
  for (const float jitter :
      {-0.03F, -0.01F, 0.0F, 0.02F, 0.03F, -0.02F, 0.01F}) {
    AddPoint(&input, 0.20F, 0.30F, 1.0F + jitter);
  }

  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, Options(0.5F), &output, &error)) << error;
  ASSERT_EQ(output.size(), 1U);
  EXPECT_NEAR(output.points.front().z, 1.0F, 0.01F);
}

TEST(GroundGridProcessor, LeavesEqualHeightLayersEmptyInsteadOfAveraging) {
  Cloud input;
  for (int i = 0; i < 4; ++i) {
    AddPoint(&input, 0.20F + 0.01F * static_cast<float>(i),
        0.20F, 0.0F);
    AddPoint(&input, 0.20F, 0.20F + 0.01F * static_cast<float>(i), 1.0F);
  }

  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, Options(0.5F), &output, &error)) << error;
  EXPECT_TRUE(output.empty());
  for (const auto& point : output.points) {
    EXPECT_FALSE(point.z > 0.2F && point.z < 0.8F);
  }
}

TEST(GroundGridProcessor, SuppressesIsolatedFloatingCell) {
  Cloud input;
  for (const auto& point : BuildCloud(0.5F, -2, 2, 0.0F).points) {
    if (CellKey(point.x, point.y, 0.5F) == std::make_pair(0, 0)) {
      continue;
    }
    input.push_back(point);
  }
  AddPoint(&input, 0.25F, 0.25F, 5.0F);

  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, Options(0.5F), &output, &error)) << error;
  ASSERT_GE(output.size(), 20U);
  for (const auto& point : output.points) {
    EXPECT_LT(std::fabs(point.z), 0.05F);
  }
}

TEST(GroundGridProcessor, UsesFloorForNegativeCoordinates) {
  Cloud input;
  AddPoint(&input, -0.55F, -0.55F, 0.1F);
  AddPoint(&input, -0.01F, -0.01F, 0.2F);

  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, Options(0.5F), &output, &error)) << error;
  ASSERT_EQ(output.size(), 2U);
  std::set<std::pair<int, int> > keys;
  for (const auto& point : output.points) {
    keys.insert(CellKey(point.x, point.y, 0.5F));
  }
  EXPECT_EQ(keys.count({-2, -2}), 1U);
  EXPECT_EQ(keys.count({-1, -1}), 1U);
}

TEST(GroundGridProcessor, IgnoresNonFiniteInput) {
  Cloud input;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  AddPoint(&input, nan, 0.0F, 0.0F);
  AddPoint(&input, 0.0F, inf, 0.0F);
  AddPoint(&input, 0.0F, 0.0F, -inf);
  AddPoint(&input, 0.25F, 0.25F, 1.0F);

  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, Options(0.5F), &output, &error)) << error;
  ASSERT_EQ(output.size(), 1U);
  EXPECT_TRUE(std::isfinite(output.points.front().x));
  EXPECT_TRUE(std::isfinite(output.points.front().y));
  EXPECT_TRUE(std::isfinite(output.points.front().z));
}

TEST(GroundGridProcessor, FillsSupportedInteriorHoleOnlyWhenEnabled) {
  Cloud input = BuildCloud(0.5F, -2, 2, 0.1F);
  ASSERT_EQ(input.size(), 25U);
  Cloud with_hole;
  for (const auto& point : input.points) {
    if (CellKey(point.x, point.y, 0.5F) == std::make_pair(0, 0)) {
      continue;
    }
    with_hole.push_back(point);
  }

  GroundGridExportOptions disabled = Options(0.5F);
  Cloud closed;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(with_hole, disabled, &closed, &error)) << error;
  EXPECT_EQ(closed.size(), 24U);

  GroundGridExportOptions enabled = Options(0.5F);
  enabled.fill_unobserved_gaps = true;
  Cloud filled;
  ASSERT_TRUE(BuildGroundGrid(with_hole, enabled, &filled, &error)) << error;
  ASSERT_EQ(filled.size(), 25U);
  bool saw_center = false;
  for (const auto& point : filled.points) {
    if (CellKey(point.x, point.y, 0.5F) != std::make_pair(0, 0)) {
      continue;
    }
    saw_center = true;
    EXPECT_NEAR(point.z, 0.1F, 0.02F);
  }
  EXPECT_TRUE(saw_center);
}

TEST(GroundGridProcessor, DoesNotFillOpenBoundary) {
  Cloud input = BuildCloud(0.5F, 0, 2, 0.0F);
  GroundGridExportOptions options = Options(0.5F);
  options.fill_unobserved_gaps = true;

  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, options, &output, &error)) << error;
  for (const auto& point : output.points) {
    const auto key = CellKey(point.x, point.y, 0.5F);
    EXPECT_GE(key.first, 0);
    EXPECT_LE(key.first, 2);
    EXPECT_GE(key.second, 0);
    EXPECT_LE(key.second, 2);
  }
}

TEST(GroundGridProcessor, DoesNotFillLargeInteriorHole) {
  Cloud full = BuildCloud(0.5F, 0, 8, 0.0F);
  Cloud input;
  for (const auto& point : full.points) {
    const auto key = CellKey(point.x, point.y, 0.5F);
    const bool in_hole = key.first >= 2 && key.first <= 6 &&
        key.second >= 2 && key.second <= 6;
    if (!in_hole) {
      input.push_back(point);
    }
  }

  GroundGridExportOptions options = Options(0.5F);
  options.fill_unobserved_gaps = true;
  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, options, &output, &error)) << error;
  for (const auto& point : output.points) {
    const auto key = CellKey(point.x, point.y, 0.5F);
    const bool in_hole = key.first >= 2 && key.first <= 6 &&
        key.second >= 2 && key.second <= 6;
    EXPECT_FALSE(in_hole) << key.first << "," << key.second;
  }
}

TEST(GroundGridProcessor, DoesNotBridgeHeightCliffWithGapFilling) {
  Cloud input = BuildCloud(0.5F, -2, 0, 0.0F);
  const Cloud right = BuildCloud(0.5F, 2, 4, 2.0F);
  input += right;

  GroundGridExportOptions options = Options(0.5F);
  options.fill_unobserved_gaps = true;
  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, options, &output, &error)) << error;
  for (const auto& point : output.points) {
    const auto key = CellKey(point.x, point.y, 0.5F);
    EXPECT_NE(key.first, 1) << key.first << "," << key.second;
  }
}

TEST(GroundGridProcessor, NeverRefillsAmbiguousObservedCell) {
  Cloud input = BuildCloud(0.5F, -2, 2, 0.0F);
  Cloud with_ambiguous_center;
  for (const auto& point : input.points) {
    if (CellKey(point.x, point.y, 0.5F) == std::make_pair(0, 0)) {
      continue;
    }
    with_ambiguous_center.push_back(point);
  }
  for (int i = 0; i < 2; ++i) {
    AddPoint(&with_ambiguous_center, 0.25F + 0.01F * static_cast<float>(i),
        0.25F, 0.0F);
    AddPoint(&with_ambiguous_center, 0.25F,
        0.25F + 0.01F * static_cast<float>(i), 1.0F);
  }

  GroundGridExportOptions options = Options(0.5F);
  options.fill_unobserved_gaps = true;
  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(with_ambiguous_center, options, &output, &error))
    << error;
  for (const auto& point : output.points) {
    EXPECT_NE(CellKey(point.x, point.y, 0.5F), std::make_pair(0, 0));
  }
}

TEST(GroundGridProcessor, PreservesExactCentersAtCentimeterResolution) {
  const Cloud input = BuildCloud(0.01F, -3, 3, 0.5F);

  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, Options(0.01F), &output, &error)) << error;
  ASSERT_FALSE(output.empty());
  for (const auto& point : output.points) {
    const auto key = CellKey(point.x, point.y, 0.01F);
    EXPECT_FLOAT_EQ(point.x, (static_cast<float>(key.first) + 0.5F) * 0.01F);
    EXPECT_FLOAT_EQ(point.y, (static_cast<float>(key.second) + 0.5F) * 0.01F);
  }
}

TEST(GroundGridProcessor, SupportsLargeValidGridKeys) {
  Cloud input;
  AddPoint(&input, 500000.0F, 0.0F, 1.0F);
  GroundGridExportOptions options = Options(0.0001F);
  options.search_radius = 0.4F;

  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, options, &output, &error)) << error;
  ASSERT_EQ(output.size(), 1U);
  EXPECT_TRUE(std::isfinite(output.points.front().x));
  EXPECT_TRUE(std::isfinite(output.points.front().y));
  EXPECT_NEAR(output.points.front().x, 500000.0F, 0.1F);
}

TEST(GroundGridProcessor, RejectsUnrepresentableFiniteCoordinates) {
  Cloud input;
  AddPoint(&input, 1e30F, 1e30F, 0.0F);

  Cloud output;
  std::string error;
  EXPECT_FALSE(BuildGroundGrid(input, Options(0.5F), &output, &error));
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(output.empty());
}

TEST(GroundGridProcessor, SparseSmallResolutionCloudReturnsQuickly) {
  Cloud input;
  for (int ix = -1; ix <= 1; ++ix) {
    for (int iy = -1; iy <= 1; ++iy) {
      AddPoint(&input, static_cast<float>(ix) + 0.5F,
          static_cast<float>(iy) + 0.5F, 0.2F);
    }
  }
  GroundGridExportOptions options = Options(0.0001F);
  options.search_radius = 0.4F;

  const auto start = std::chrono::steady_clock::now();
  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, options, &output, &error)) << error;
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start);
  EXPECT_EQ(output.size(), 9U);
  EXPECT_LT(elapsed.count(), 2000) << "neighbor search was not bounded";
}

TEST(GroundGridProcessor, RejectsNonPositiveResolution) {
  Cloud input;
  AddPoint(&input, 0.0F, 0.0F, 0.0F);
  for (const float resolution : {0.0F, -0.5F}) {
    Cloud output;
    std::string error;
    EXPECT_FALSE(
      BuildGroundGrid(input, Options(resolution), &output, &error));
    EXPECT_FALSE(error.empty());
    EXPECT_TRUE(output.empty());
  }
}

TEST(GroundGridProcessor, RejectsNonPositiveMaxHoleCells) {
  Cloud input;
  AddPoint(&input, 0.0F, 0.0F, 0.0F);
  for (const int max_hole_cells : {0, -1}) {
    GroundGridExportOptions options = Options(0.5F);
    options.max_hole_cells = max_hole_cells;
    Cloud output;
    std::string error;
    EXPECT_FALSE(BuildGroundGrid(input, options, &output, &error));
    EXPECT_FALSE(error.empty());
    EXPECT_TRUE(output.empty());
  }
}

TEST(GroundGridProcessor, FillsEnclosedTwoCellHole) {
  const Cloud full = BuildCloud(0.1F, -6, 6, 0.1F);
  const Cloud input = RemoveCells(full, 0.1F, 0, 1, 0, 0);
  GroundGridExportOptions options = Options(0.1F);
  options.search_radius = 0.4F;
  options.fill_unobserved_gaps = true;

  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, options, &output, &error)) << error;
  for (int ix = 0; ix <= 1; ++ix) {
    const auto* filled = FindPoint(output, ix, 0, 0.1F);
    ASSERT_NE(filled, nullptr) << ix;
    EXPECT_NEAR(filled->z, 0.1F, 0.02F);
  }
}

TEST(GroundGridProcessor, FillsEnclosedFourteenCellHole) {
  const Cloud full = BuildCloud(0.1F, -6, 6, 0.1F);
  const Cloud input = RemoveCells(full, 0.1F, -1, 0, -3, 3);
  GroundGridExportOptions options = Options(0.1F);
  // The production 0.4 m search radius must support this component.
  options.search_radius = 0.4F;
  options.fill_unobserved_gaps = true;

  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, options, &output, &error)) << error;
  for (int ix = -1; ix <= 0; ++ix) {
    for (int iy = -3; iy <= 3; ++iy) {
      const auto* filled = FindPoint(output, ix, iy, 0.1F);
      ASSERT_NE(filled, nullptr) << ix << "," << iy;
      EXPECT_NEAR(filled->z, 0.1F, 0.02F);
    }
  }
}

TEST(GroundGridProcessor, RejectsEnclosedHoleLargerThanFourteenCells) {
  const Cloud full = BuildCloud(0.1F, -6, 6, 0.1F);
  const Cloud input = RemoveCells(full, 0.1F, -1, 1, -2, 2);
  GroundGridExportOptions options = Options(0.1F);
  options.search_radius = 0.6F;
  options.fill_unobserved_gaps = true;

  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, options, &output, &error)) << error;
  ExpectHoleEmpty(output, 0.1F, -1, 1, -2, 2);
}

TEST(GroundGridProcessor, FillsSlopedFourteenCellHoleAtPlaneHeight) {
  const Cloud full = BuildSlopedCloud(0.1F, -6, 6, 0.2F, 0.1F, 0.3F);
  const Cloud input = RemoveCells(full, 0.1F, -1, 0, -3, 3);
  GroundGridExportOptions options = Options(0.1F);
  // A sloped component must also work with the production 0.4 m radius.
  options.search_radius = 0.4F;
  options.fill_unobserved_gaps = true;

  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, options, &output, &error)) << error;
  for (int ix = -1; ix <= 0; ++ix) {
    for (int iy = -3; iy <= 3; ++iy) {
      const auto* filled = FindPoint(output, ix, iy, 0.1F);
      ASSERT_NE(filled, nullptr) << ix << "," << iy;
      const float x = (static_cast<float>(ix) + 0.5F) * 0.1F;
      const float y = (static_cast<float>(iy) + 0.5F) * 0.1F;
      EXPECT_NEAR(filled->z, 0.2F * x + 0.1F * y + 0.3F, 0.02F);
    }
  }
}

TEST(GroundGridProcessor, DoesNotFabricateSlopeAcrossHeightFaultInHole) {
  Cloud input;
  for (int ix = -6; ix <= 6; ++ix) {
    for (int iy = -6; iy <= 6; ++iy) {
      const float x = (static_cast<float>(ix) + 0.5F) * 0.1F;
      const float y = (static_cast<float>(iy) + 0.5F) * 0.1F;
      AddPoint(&input, x, y, ix <= -2 ? 0.0F : 0.3F);
    }
  }
  input = RemoveCells(input, 0.1F, -1, 0, -3, 3);
  GroundGridExportOptions options = Options(0.1F);
  options.search_radius = 0.6F;
  options.fill_unobserved_gaps = true;

  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, options, &output, &error)) << error;
  ExpectHoleEmpty(output, 0.1F, -1, 0, -3, 3);
}

TEST(GroundGridProcessor, DoesNotFillSmallGapOpenToBoundary) {
  const Cloud full = BuildCloud(0.1F, 0, 10, 0.1F);
  const Cloud input = RemoveCells(full, 0.1F, 0, 1, 4, 6);
  GroundGridExportOptions options = Options(0.1F);
  options.search_radius = 0.6F;
  options.fill_unobserved_gaps = true;

  Cloud output;
  std::string error;
  ASSERT_TRUE(BuildGroundGrid(input, options, &output, &error)) << error;
  ExpectHoleEmpty(output, 0.1F, 0, 1, 4, 6);
}

TEST(GroundGridProcessor, FillingDoesNotExpandSparseCoordinateBounds) {
  Cloud input;
  for (int ix = -1; ix <= 1; ++ix) {
    for (int iy = -1; iy <= 1; ++iy) {
      AddPoint(&input, ix * 100.0F, iy * 100.0F, 0.0F);
    }
  }
  auto options = Options(0.0001F);
  options.search_radius = 0.4F;
  options.fill_unobserved_gaps = true;
  const auto start = std::chrono::steady_clock::now();
  Cloud output;
  ASSERT_TRUE(BuildGroundGrid(input, options, &output));
  EXPECT_EQ(output.size(), input.size());
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start);
  EXPECT_LT(elapsed.count(), 2000);
}

}  // namespace
