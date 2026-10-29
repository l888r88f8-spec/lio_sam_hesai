#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <vector>

#include "lio_sam/ground_mesh.h"

namespace
{

using lio_sam_hesai::GroundMeshOptions;
using lio_sam_hesai::GroundMeshRasterizer;
using Point = pcl::PointXYZI;

constexpr float kPi = 3.14159265358979323846f;

Point MakePoint(float x, float y, float z) {
  Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = 1.0f;
  return point;
}

GroundMeshOptions RelaxedOptions() {
  GroundMeshOptions options;
  options.resolution = 1.0f;
  options.max_edge_length = 10.0f;
  options.edge_length_per_meter = 0.0f;
  options.max_range_ratio = 100.0f;
  options.max_slope_deg = 80.0f;
  options.max_height_range = 0.5f;
  return options;
}

struct ProjectedFrame {
  int rows = 0;
  int columns = 0;
  pcl::PointCloud<Point> cloud;
  std::vector<std::uint32_t> projection_indices;
  std::vector<std::int8_t> classification;

  ProjectedFrame(int input_rows, int input_columns)
    : rows(input_rows),
    columns(input_columns),
    classification(static_cast<std::size_t>(input_rows) *
        static_cast<std::size_t>(input_columns),
        0) {
  }

  void Add(int ring, int column, const Point &point, std::int8_t label = 1) {
    const int wrapped_column = column % columns;
    cloud.push_back(point);
    projection_indices.push_back(static_cast<std::uint32_t>(
          ring * columns + wrapped_column));
    classification[static_cast<std::size_t>(ring) * columns +
        wrapped_column] = label;
  }

  void AddDuplicate(int ring, int column, const Point &point) {
    const int wrapped_column = column % columns;
    cloud.push_back(point);
    projection_indices.push_back(static_cast<std::uint32_t>(
          ring * columns + wrapped_column));
    classification[static_cast<std::size_t>(ring) * columns +
        wrapped_column] = 1;
  }
};

void AddQuad(ProjectedFrame *frame, int ring, int column,
    const Point &a, const Point &b,
    const Point &c, const Point &d) {
  frame->Add(ring, column, a);
  frame->Add(ring, column + 1, b);
  frame->Add(ring + 1, column, c);
  frame->Add(ring + 1, column + 1, d);
}

void AddFrame(GroundMeshRasterizer *rasterizer, const ProjectedFrame &frame,
    lio_sam_hesai::GroundRingRange rings) {
  rasterizer->AddFrame(frame.cloud, frame.projection_indices,
      frame.classification, frame.rows, frame.columns,
      rings, Eigen::Matrix4f::Identity());
}

bool HasPointInCell(const pcl::PointCloud<Point> &cloud, float resolution,
    int cell_x, int cell_y) {
  for (const Point &point : cloud) {
    if (static_cast<int>(std::floor(point.x / resolution)) == cell_x &&
        static_cast<int>(std::floor(point.y / resolution)) == cell_y) {
      return true;
    }
  }
  return false;
}

std::size_t CountPointsInCell(const pcl::PointCloud<Point> &cloud,
    float resolution, int cell_x, int cell_y) {
  std::size_t count = 0;
  for (const Point &point : cloud) {
    if (static_cast<int>(std::floor(point.x / resolution)) == cell_x &&
        static_cast<int>(std::floor(point.y / resolution)) == cell_y) {
      ++count;
    }
  }
  return count;
}

}  // namespace

static_assert(!std::is_copy_constructible<GroundMeshRasterizer>::value,
    "GroundMeshRasterizer must not be copy constructible");
static_assert(!std::is_copy_assignable<GroundMeshRasterizer>::value,
    "GroundMeshRasterizer must not be copy assignable");

TEST(GroundMeshApi, RejectsInvalidOptions)
{
  GroundMeshOptions options = RelaxedOptions();
  options.resolution = 0.0f;
  EXPECT_THROW(GroundMeshRasterizer{options}, std::invalid_argument);

  options = RelaxedOptions();
  options.max_edge_length = 0.0f;
  EXPECT_THROW(GroundMeshRasterizer{options}, std::invalid_argument);

  options = RelaxedOptions();
  options.edge_length_per_meter = -0.5f;
  EXPECT_THROW(GroundMeshRasterizer{options}, std::invalid_argument);

  options = RelaxedOptions();
  options.max_range_ratio = 0.0f;
  EXPECT_THROW(GroundMeshRasterizer{options}, std::invalid_argument);

  options = RelaxedOptions();
  options.max_slope_deg = 90.0f;
  EXPECT_THROW(GroundMeshRasterizer{options}, std::invalid_argument);

  options = RelaxedOptions();
  options.max_height_range = 0.0f;
  EXPECT_THROW(GroundMeshRasterizer{options}, std::invalid_argument);
}

TEST(GroundMeshApi, RejectsInvalidFrameInputsAndPose)
{
  GroundMeshRasterizer rasterizer(RelaxedOptions());
  ProjectedFrame frame(2, 2);
  AddQuad(&frame, 0, 0, MakePoint(0.1f, 0.1f, 0.0f),
      MakePoint(0.9f, 0.1f, 0.0f), MakePoint(0.1f, 0.9f, 0.0f),
      MakePoint(0.9f, 0.9f, 0.0f));

  EXPECT_THROW(rasterizer.AddFrame(
        frame.cloud, {}, frame.classification, frame.rows,
        frame.columns, {0, 1}, Eigen::Matrix4f::Identity()),
      std::invalid_argument);

  EXPECT_THROW(rasterizer.AddFrame(
        frame.cloud, frame.projection_indices, {}, frame.rows,
        frame.columns, {0, 1}, Eigen::Matrix4f::Identity()),
      std::invalid_argument);

  auto bad_index = frame.projection_indices;
  bad_index.front() = 99;
  EXPECT_THROW(rasterizer.AddFrame(
        frame.cloud, bad_index, frame.classification, frame.rows,
        frame.columns, {0, 1}, Eigen::Matrix4f::Identity()),
      std::invalid_argument);

  EXPECT_THROW(rasterizer.AddFrame(
        frame.cloud, frame.projection_indices, frame.classification,
        0, frame.columns, {0, 0}, Eigen::Matrix4f::Identity()),
      std::invalid_argument);

  Eigen::Matrix4f scaled = Eigen::Matrix4f::Identity();
  scaled(0, 0) = 2.0f;
  EXPECT_THROW(rasterizer.AddFrame(
        frame.cloud, frame.projection_indices, frame.classification,
        frame.rows, frame.columns, {0, 1}, scaled),
      std::invalid_argument);

  Eigen::Matrix4f not_finite = Eigen::Matrix4f::Identity();
  not_finite(0, 3) = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(rasterizer.AddFrame(
        frame.cloud, frame.projection_indices, frame.classification,
        frame.rows, frame.columns, {0, 1}, not_finite),
      std::invalid_argument);
}

TEST(GroundMeshApi, RejectsNullInfillTarget)
{
  GroundMeshRasterizer rasterizer(RelaxedOptions());
  EXPECT_THROW(rasterizer.AppendMissingSamples(nullptr), std::invalid_argument);
}

TEST(GroundMeshInfill, FlatGroundFillsMoreThanOneCell)
{
  GroundMeshOptions options = RelaxedOptions();
  options.resolution = 0.5f;
  GroundMeshRasterizer rasterizer(options);

  ProjectedFrame frame(4, 4);
  const float coordinates[4] = {0.1f, 0.5f, 1.0f, 1.4f};
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      frame.Add(row, column,
          MakePoint(coordinates[row], coordinates[column], 0.0f));
    }
  }

  AddFrame(&rasterizer, frame, {0, 3});

  pcl::PointCloud<Point> measured;
  const std::size_t added = rasterizer.AppendMissingSamples(&measured);

  EXPECT_EQ(rasterizer.statistics().frames, 1U);
  EXPECT_GE(rasterizer.statistics().accepted_triangles, 1U);
  EXPECT_GE(added, 2U);
  EXPECT_EQ(added, measured.size());
  for (const Point &point : measured) {
    EXPECT_NEAR(point.z, 0.0f, 1.0e-4f);
  }
}

TEST(GroundMeshInfill, SlopedGroundKeepsWorldHeightSlope)
{
  GroundMeshOptions options = RelaxedOptions();
  options.resolution = 0.5f;
  GroundMeshRasterizer rasterizer(options);

  ProjectedFrame frame(4, 4);
  const float coordinates[4] = {0.1f, 0.5f, 1.0f, 1.4f};
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      const float x = coordinates[row];
      const float y = coordinates[column];
      frame.Add(row, column, MakePoint(x, y, 0.2f * x + 0.1f * y));
    }
  }

  AddFrame(&rasterizer, frame, {0, 3});

  pcl::PointCloud<Point> measured;
  const std::size_t added = rasterizer.AppendMissingSamples(&measured);
  ASSERT_GE(added, 2U);
  EXPECT_EQ(rasterizer.statistics().rejected_slope, 0U);
  for (const Point &point : measured) {
    EXPECT_NEAR(point.z, 0.2f * point.x + 0.1f * point.y, 1.0e-4f);
  }
}

TEST(GroundMeshInfill, GlobalPoseChangesCellCentersAndWorldSlope)
{
  GroundMeshOptions options = RelaxedOptions();
  options.resolution = 1.0f;
  GroundMeshRasterizer rasterizer(options);

  Eigen::Affine3f map_from_scan = Eigen::Affine3f::Identity();
  map_from_scan.translation() = Eigen::Vector3f(2.0f, -1.0f, 0.3f);
  map_from_scan.linear() =
      (Eigen::AngleAxisf(30.0f * kPi / 180.0f, Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(10.0f * kPi / 180.0f, Eigen::Vector3f::UnitX()))
      .toRotationMatrix();

  ProjectedFrame frame(3, 3);
  const float coordinates[3] = {0.1f, 0.9f, 1.7f};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      const float x = coordinates[row];
      const float y = coordinates[column];
      const Point world = MakePoint(x, y, 0.1f * x + 0.2f * y + 0.5f);
      const Eigen::Vector3f scan =
          map_from_scan.inverse() * Eigen::Vector3f(world.x, world.y, world.z);
      frame.Add(row, column, MakePoint(scan.x(), scan.y(), scan.z()));
    }
  }

  rasterizer.AddFrame(frame.cloud, frame.projection_indices,
      frame.classification, frame.rows, frame.columns, {0, 2},
      map_from_scan.matrix());

  pcl::PointCloud<Point> measured;
  const std::size_t added = rasterizer.AppendMissingSamples(&measured);
  ASSERT_GE(added, 2U);

  for (const Point &point : measured) {
    const float center_x = (std::floor(point.x) + 0.5f) * options.resolution;
    const float center_y = (std::floor(point.y) + 0.5f) * options.resolution;
    EXPECT_NEAR(point.x, center_x, 1.0e-4f);
    EXPECT_NEAR(point.y, center_y, 1.0e-4f);
    EXPECT_NEAR(point.z, 0.1f * point.x + 0.2f * point.y + 0.5f, 1.0e-4f);
  }
}

TEST(GroundMeshTriangulation, GroundLabelAndOutputRingRestriction)
{
  GroundMeshRasterizer rasterizer(RelaxedOptions());

  ProjectedFrame frame(4, 4);
  AddQuad(&frame, 0, 0, MakePoint(0.1f, 0.1f, 0.0f),
      MakePoint(0.9f, 0.1f, 0.0f), MakePoint(0.1f, 0.9f, 0.0f),
      MakePoint(0.9f, 0.9f, 0.0f));
  frame.classification[1U * 4U + 1U] = -1;
  AddQuad(&frame, 2, 0, MakePoint(2.1f, 0.1f, 0.0f),
      MakePoint(2.9f, 0.1f, 0.0f), MakePoint(2.1f, 0.9f, 0.0f),
      MakePoint(2.9f, 0.9f, 0.0f));

  AddFrame(&rasterizer, frame, {0, 1});

  pcl::PointCloud<Point> measured;
  rasterizer.AppendMissingSamples(&measured);

  EXPECT_EQ(rasterizer.statistics().accepted_triangles, 1U);
  EXPECT_GE(rasterizer.statistics().rejected_ground, 1U);
  EXPECT_TRUE(HasPointInCell(measured, 1.0f, 0, 0));
  EXPECT_FALSE(HasPointInCell(measured, 1.0f, 2, 0));
}

TEST(GroundMeshTriangulation, ExcessivelyFineResolutionFailsBeforeAllocation)
{
  auto options = RelaxedOptions();
  options.resolution = 0.0001f;
  GroundMeshRasterizer rasterizer(options);
  ProjectedFrame frame(2, 2);
  frame.Add(0, 0, MakePoint(10.0f, 0.0f, 0.0f));
  frame.Add(0, 1, MakePoint(10.8f, 0.0f, 0.0f));
  frame.Add(1, 0, MakePoint(10.0f, 0.8f, 0.0f));
  EXPECT_THROW(AddFrame(&rasterizer, frame, {0, 1}), std::length_error);
  EXPECT_EQ(rasterizer.statistics().rasterized_cells, 0U);
}

TEST(GroundMeshTriangulation, ThreeValidCornersFormOneTriangle)
{
  GroundMeshRasterizer rasterizer(RelaxedOptions());

  ProjectedFrame frame(2, 2);
  AddQuad(&frame, 0, 0, MakePoint(0.1f, 0.1f, 0.0f),
      MakePoint(0.9f, 0.15f, 0.0f), MakePoint(0.15f, 0.9f, 0.0f),
      MakePoint(0.9f, 0.9f, 0.0f));
  frame.classification[1U * 2U + 1U] = 0;

  AddFrame(&rasterizer, frame, {0, 1});

  EXPECT_EQ(rasterizer.statistics().candidate_triangles, 1U);
  EXPECT_EQ(rasterizer.statistics().accepted_triangles, 1U);
  EXPECT_EQ(rasterizer.statistics().rejected_ground, 1U);

  pcl::PointCloud<Point> measured;
  EXPECT_EQ(rasterizer.AppendMissingSamples(&measured), 1U);
  EXPECT_TRUE(HasPointInCell(measured, 1.0f, 0, 0));
}

TEST(GroundMeshTriangulation, WrapsAzimuthColumnsButEdgeLengthPreventsFalseJoin)
{
  GroundMeshOptions strict = RelaxedOptions();
  strict.max_edge_length = 0.5f;
  GroundMeshRasterizer far_rasterizer(strict);

  ProjectedFrame far_frame(2, 8);
  const float far_radius = 10.0f;
  const float far_angle = 2.0f * kPi * 7.0f / 8.0f;
  far_frame.Add(0, 7, MakePoint(far_radius * std::cos(far_angle),
      far_radius * std::sin(far_angle), 0.0f));
  far_frame.Add(1, 7, MakePoint((far_radius + 0.05f) * std::cos(far_angle),
      (far_radius + 0.05f) * std::sin(far_angle),
      0.0f));
  far_frame.Add(0, 0, MakePoint(far_radius, 0.0f, 0.0f));

  AddFrame(&far_rasterizer, far_frame, {0, 1});
  EXPECT_EQ(far_rasterizer.statistics().accepted_triangles, 0U);
  EXPECT_GE(far_rasterizer.statistics().rejected_edge, 1U);

  GroundMeshOptions wrap = RelaxedOptions();
  wrap.max_edge_length = 1.0f;
  GroundMeshRasterizer wrap_rasterizer(wrap);

  ProjectedFrame wrap_frame(2, 8);
  const float near_radius = 1.0f;
  const float near_angle = 2.0f * kPi * 7.0f / 8.0f;
  wrap_frame.Add(0, 7, MakePoint(near_radius * std::cos(near_angle),
      near_radius * std::sin(near_angle), 0.0f));
  wrap_frame.Add(1, 7, MakePoint((near_radius + 0.05f) * std::cos(near_angle),
      (near_radius + 0.05f) * std::sin(near_angle),
      0.0f));
  wrap_frame.Add(0, 0, MakePoint(near_radius, 0.0f, 0.0f));

  AddFrame(&wrap_rasterizer, wrap_frame, {0, 1});
  EXPECT_EQ(wrap_rasterizer.statistics().accepted_triangles, 1U);
}

TEST(GroundMeshRejection, EdgeLengthRejectsTriangle)
{
  GroundMeshOptions options = RelaxedOptions();
  options.max_edge_length = 0.5f;
  GroundMeshRasterizer rasterizer(options);

  ProjectedFrame frame(2, 2);
  frame.Add(0, 0, MakePoint(0.0f, 0.0f, 0.0f));
  frame.Add(0, 1, MakePoint(5.0f, 0.0f, 0.0f));
  frame.Add(1, 0, MakePoint(0.0f, 0.1f, 0.0f));

  AddFrame(&rasterizer, frame, {0, 1});
  EXPECT_EQ(rasterizer.statistics().accepted_triangles, 0U);
  EXPECT_EQ(rasterizer.statistics().rejected_edge, 1U);
}

TEST(GroundMeshRejection, RangeRatioRejectsTriangle)
{
  GroundMeshOptions options = RelaxedOptions();
  options.max_edge_length = 1000.0f;
  options.max_range_ratio = 1.2f;
  GroundMeshRasterizer rasterizer(options);

  ProjectedFrame frame(2, 2);
  frame.Add(0, 0, MakePoint(1.0f, 0.0f, 0.0f));
  frame.Add(0, 1, MakePoint(10.0f, 0.0f, 0.0f));
  frame.Add(1, 0, MakePoint(0.0f, 1.0f, 0.0f));

  AddFrame(&rasterizer, frame, {0, 1});
  EXPECT_EQ(rasterizer.statistics().accepted_triangles, 0U);
  EXPECT_EQ(rasterizer.statistics().rejected_range, 1U);
}

TEST(GroundMeshRejection, WorldSlopeRejectsTriangle)
{
  GroundMeshOptions steep = RelaxedOptions();
  steep.max_slope_deg = 10.0f;
  GroundMeshRasterizer steep_rasterizer(steep);

  ProjectedFrame frame(2, 2);
  frame.Add(0, 0, MakePoint(10.0f, 0.0f, 0.0f));
  frame.Add(0, 1, MakePoint(11.0f, 0.0f, 0.0f));
  frame.Add(1, 0, MakePoint(10.0f, 1.0f, 1.0f));

  AddFrame(&steep_rasterizer, frame, {0, 1});
  EXPECT_EQ(steep_rasterizer.statistics().accepted_triangles, 0U);
  EXPECT_EQ(steep_rasterizer.statistics().rejected_slope, 1U);

  GroundMeshOptions gentle = RelaxedOptions();
  gentle.max_slope_deg = 80.0f;
  GroundMeshRasterizer gentle_rasterizer(gentle);
  AddFrame(&gentle_rasterizer, frame, {0, 1});
  EXPECT_EQ(gentle_rasterizer.statistics().accepted_triangles, 1U);
  EXPECT_EQ(gentle_rasterizer.statistics().rejected_slope, 0U);
}

TEST(GroundMeshRejection, DegenerateTriangleIsRejected)
{
  GroundMeshRasterizer rasterizer(RelaxedOptions());

  ProjectedFrame frame(2, 2);
  frame.Add(0, 0, MakePoint(0.0f, 0.0f, 0.0f));
  frame.Add(0, 1, MakePoint(1.0f, 0.0f, 0.0f));
  frame.Add(1, 0, MakePoint(2.0f, 0.0f, 0.0f));

  AddFrame(&rasterizer, frame, {0, 1});
  EXPECT_EQ(rasterizer.statistics().accepted_triangles, 0U);
  EXPECT_EQ(rasterizer.statistics().rejected_degenerate, 1U);
}

TEST(GroundMeshLayers, EqualHeightSupportIsAmbiguousAndNotFilled)
{
  GroundMeshOptions options = RelaxedOptions();
  options.max_height_range = 0.1f;
  GroundMeshRasterizer rasterizer(options);

  ProjectedFrame frame(4, 4);
  AddQuad(&frame, 0, 0, MakePoint(0.1f, 0.1f, 0.0f),
      MakePoint(0.9f, 0.15f, 0.0f), MakePoint(0.15f, 0.9f, 0.0f),
      MakePoint(0.9f, 0.9f, 0.0f));
  AddQuad(&frame, 2, 0, MakePoint(0.1f, 0.1f, 0.4f),
      MakePoint(0.9f, 0.15f, 0.4f), MakePoint(0.15f, 0.9f, 0.4f),
      MakePoint(0.9f, 0.9f, 0.4f));

  AddFrame(&rasterizer, frame, {0, 3});

  pcl::PointCloud<Point> measured;
  const std::size_t added = rasterizer.AppendMissingSamples(&measured);
  EXPECT_EQ(added, 0U);
  EXPECT_GE(rasterizer.statistics().ambiguous_cells, 1U);
  EXPECT_FALSE(HasPointInCell(measured, 1.0f, 0, 0));
}

TEST(GroundMeshLayers, DuplicateTrianglesDoNotIncreaseHeightVotes)
{
  GroundMeshOptions options = RelaxedOptions();
  options.max_height_range = 0.1f;
  GroundMeshRasterizer rasterizer(options);

  ProjectedFrame frame(6, 4);
  const Point a0 = MakePoint(0.1f, 0.1f, 0.0f);
  const Point b0 = MakePoint(0.9f, 0.15f, 0.0f);
  const Point c0 = MakePoint(0.15f, 0.9f, 0.0f);
  const Point d0 = MakePoint(0.9f, 0.9f, 0.0f);
  AddQuad(&frame, 0, 0, a0, b0, c0, d0);
  // Two lower patches overlap in one world cell, but belong to one frame.
  AddQuad(&frame, 2, 0, a0, b0, c0, d0);
  AddQuad(&frame, 4, 0, MakePoint(0.1f, 0.1f, 0.4f),
      MakePoint(0.9f, 0.15f, 0.4f), MakePoint(0.15f, 0.9f, 0.4f),
      MakePoint(0.9f, 0.9f, 0.4f));

  AddFrame(&rasterizer, frame, {0, 5});

  pcl::PointCloud<Point> measured;
  const std::size_t added = rasterizer.AppendMissingSamples(&measured);
  EXPECT_EQ(added, 0U);
  EXPECT_GE(rasterizer.statistics().ambiguous_cells, 1U);
  EXPECT_FALSE(HasPointInCell(measured, 1.0f, 0, 0));
}

TEST(GroundMeshInfill, LeavesOriginalMeasuredCellPointsUntouched)
{
  GroundMeshRasterizer rasterizer(RelaxedOptions());

  ProjectedFrame frame(2, 2);
  AddQuad(&frame, 0, 0, MakePoint(0.1f, 0.1f, 0.0f),
      MakePoint(0.9f, 0.15f, 0.0f), MakePoint(0.15f, 0.9f, 0.0f),
      MakePoint(0.9f, 0.9f, 0.0f));
  AddFrame(&rasterizer, frame, {0, 1});

  pcl::PointCloud<Point> measured;
  measured.push_back(MakePoint(0.5f, 0.5f, 7.0f));
  const std::size_t added = rasterizer.AppendMissingSamples(&measured);

  EXPECT_EQ(CountPointsInCell(measured, 1.0f, 0, 0), 1U);
  EXPECT_FLOAT_EQ(measured.front().x, 0.5f);
  EXPECT_FLOAT_EQ(measured.front().y, 0.5f);
  EXPECT_FLOAT_EQ(measured.front().z, 7.0f);
  EXPECT_EQ(added, measured.size() - 1U);
}

TEST(GroundMeshLayers, CrossFrameSupportAndMinimumKeyframes) {
  ProjectedFrame frame(2, 2);
  AddQuad(&frame, 0, 0, MakePoint(0.1f, 0.1f, 0.0f),
      MakePoint(0.9f, 0.15f, 0.0f), MakePoint(0.15f, 0.9f, 0.0f),
      MakePoint(0.9f, 0.9f, 0.0f));
  frame.classification[1U * 2U + 1U] = 0;

  GroundMeshOptions supported = RelaxedOptions();
  supported.max_height_range = 0.1f;
  supported.min_keyframe_support = 2;
  GroundMeshRasterizer supported_rasterizer(supported);
  AddFrame(&supported_rasterizer, frame, {0, 1});
  AddFrame(&supported_rasterizer, frame, {0, 1});

  pcl::PointCloud<Point> measured;
  const std::size_t added =
      supported_rasterizer.AppendMissingSamples(&measured);
  ASSERT_EQ(added, 1U);
  EXPECT_NEAR(measured.front().z, 0.0f, 1.0e-4f);

  GroundMeshOptions under_supported = supported;
  under_supported.min_keyframe_support = 3;
  GroundMeshRasterizer under_supported_rasterizer(under_supported);
  AddFrame(&under_supported_rasterizer, frame, {0, 1});
  AddFrame(&under_supported_rasterizer, frame, {0, 1});
  pcl::PointCloud<Point> unsupported;
  EXPECT_EQ(under_supported_rasterizer.AppendMissingSamples(&unsupported), 0U);
  EXPECT_TRUE(unsupported.empty());
}

TEST(GroundMeshLayers, MajorityHeightLayerWinsWithoutTriangleWeighting) {
  GroundMeshOptions options = RelaxedOptions();
  options.max_height_range = 0.1f;
  GroundMeshRasterizer rasterizer(options);

  ProjectedFrame low(2, 2);
  AddQuad(&low, 0, 0, MakePoint(0.1f, 0.1f, 0.0f),
      MakePoint(0.9f, 0.15f, 0.0f), MakePoint(0.15f, 0.9f, 0.0f),
      MakePoint(0.9f, 0.9f, 0.0f));
  low.classification[1U * 2U + 1U] = 0;

  ProjectedFrame high(2, 2);
  AddQuad(&high, 0, 0, MakePoint(0.1f, 0.1f, 0.4f),
      MakePoint(0.9f, 0.15f, 0.4f), MakePoint(0.15f, 0.9f, 0.4f),
      MakePoint(0.9f, 0.9f, 0.4f));
  high.classification[1U * 2U + 1U] = 0;

  AddFrame(&rasterizer, low, {0, 1});
  AddFrame(&rasterizer, low, {0, 1});
  AddFrame(&rasterizer, low, {0, 1});
  AddFrame(&rasterizer, high, {0, 1});

  pcl::PointCloud<Point> measured;
  const std::size_t added = rasterizer.AppendMissingSamples(&measured);
  ASSERT_EQ(added, 1U);
  EXPECT_NEAR(measured.front().z, 0.0f, 1.0e-4f);

  GroundMeshRasterizer competing(options);
  AddFrame(&competing, low, {0, 1});
  AddFrame(&competing, low, {0, 1});
  AddFrame(&competing, high, {0, 1});
  AddFrame(&competing, high, {0, 1});
  pcl::PointCloud<Point> unresolved;
  EXPECT_EQ(competing.AppendMissingSamples(&unresolved), 0U);
  EXPECT_TRUE(unresolved.empty());
  EXPECT_GE(competing.statistics().ambiguous_cells, 1U);
}

TEST(GroundMeshApi, RejectsDuplicateProjectionIndices) {
  GroundMeshRasterizer rasterizer(RelaxedOptions());

  ProjectedFrame frame(2, 2);
  AddQuad(&frame, 0, 0, MakePoint(0.1f, 0.1f, 0.0f),
      MakePoint(0.9f, 0.1f, 0.0f), MakePoint(0.1f, 0.9f, 0.0f),
      MakePoint(0.9f, 0.9f, 0.0f));
  frame.AddDuplicate(0, 0, MakePoint(0.1f, 0.1f, 0.0f));

  EXPECT_THROW(rasterizer.AddFrame(frame.cloud, frame.projection_indices,
      frame.classification, frame.rows,
      frame.columns, {0, 1},
      Eigen::Matrix4f::Identity()),
      std::invalid_argument);
}

