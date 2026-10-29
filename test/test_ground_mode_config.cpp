#include "common/slam_config.h"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
SlamConfig Load(const std::vector<rclcpp::Parameter>& overrides) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  auto node = std::make_shared<rclcpp::Node>("ground_mode_config", options);
  return LoadSlamConfig(*node, SlamMode::kMapping);
}
}  // namespace

TEST(GroundModeConfig, RejectsInvalidModesIncludingBoth) {
  for (const auto* name : {"ground_segmentation.mode", "patchworkpp.mode"}) {
    for (const auto* value : {"both", "offline", "disabled", "", "ONLINE"}) {
      SCOPED_TRACE(std::string(name) + "=" + value);
      EXPECT_THROW(Load({rclcpp::Parameter(name, std::string(value))}),
                   std::invalid_argument);
    }
  }
}

TEST(GroundModeConfig, RejectsInvalidPatchworkGeometry) {
  for (const auto& parameter : std::vector<rclcpp::Parameter>{
           {"patchworkpp.sensor_height", 0.0},
           {"patchworkpp.min_range", -1.0},
           {"patchworkpp.max_range", 0.1},
           {"patchworkpp.uprightness_thr", 0.0},
           {"patchworkpp.uprightness_thr", 1.1},
           {"patchworkpp.th_seeds", 0.0},
           {"patchworkpp.th_dist", -0.1},
           {"patchworkpp.rnr_intensity_threshold", -0.1},
           {"patchworkpp.sensor_height", std::numeric_limits<double>::quiet_NaN()},
           {"patchworkpp.th_dist", std::numeric_limits<double>::infinity()}}) {
    SCOPED_TRACE(parameter.get_name());
    EXPECT_THROW(Load({parameter}), std::invalid_argument);
  }
}

TEST(GroundModeConfig, PreservesExistingOnlineDefaults) {
  const auto config = Load({});
  EXPECT_EQ(config.mapping.ground_segmentation.mode, "online");
  EXPECT_EQ(config.mapping.patchworkpp.mode, "off");
  EXPECT_FALSE(config.mapping.patchworkpp.enable_rnr);
}

TEST(GroundModeConfig, LoadsIndependentModesAndPatchworkParameters) {
  for (const auto* current : {"off", "online", "export"}) {
    for (const auto* patchwork : {"off", "online", "export"}) {
      const auto config = Load({
          {"ground_segmentation.mode", std::string(current)},
          {"patchworkpp.mode", std::string(patchwork)},
          {"patchworkpp.sensor_height", 0.68},
          {"patchworkpp.enable_rnr", true},
          {"patchworkpp.rnr_intensity_threshold", 3.0}});
      EXPECT_EQ(config.mapping.ground_segmentation.mode, current);
      EXPECT_EQ(config.mapping.patchworkpp.mode, patchwork);
      EXPECT_DOUBLE_EQ(config.mapping.patchworkpp.sensor_height, 0.68);
      EXPECT_TRUE(config.mapping.patchworkpp.enable_rnr);
      EXPECT_DOUBLE_EQ(config.mapping.patchworkpp.rnr_intensity_threshold, 3.0);
    }
  }
}

namespace {
std::vector<rclcpp::Parameter> GroundParameters(const std::string& prefix) {
  return {{prefix+"mode",std::string("export")},
      {prefix+"only_mode",true},{prefix+"leaf_size",.23},
      {prefix+"scan_start_index",2},{prefix+"scan_end_index",8},
      {prefix+"angle_threshold_deg",14.0},{prefix+"plane_distance",.18},
      {prefix+"plane_max_angle_deg",22.0},{prefix+"plane_min_points",44},
      {prefix+"flatten_z",true},{prefix+"flatten_value",-.6},
      {prefix+"grid_patch.enable",true},{prefix+"grid_patch.resolution",.2},
      {prefix+"grid_patch.search_radius",.9},{prefix+"grid_patch.max_slope_deg",20.0},
      {prefix+"grid_patch.max_height_range",.16},{prefix+"grid_patch.max_roughness",.07},
      {prefix+"grid_patch.max_prediction_margin",.09},{prefix+"grid_patch.min_inlier_ratio",.8},
      {prefix+"grid_patch.min_neighbor_cells",9},{prefix+"grid_patch.min_support_quadrants",4}};
}
void ExpectGroundParameters(const GroundSegmentationConfig& ground,
                            const std::string& mode="export") {
  EXPECT_EQ(ground.mode,mode);EXPECT_TRUE(ground.only_mode);
  EXPECT_DOUBLE_EQ(ground.leaf_size,.23);
  EXPECT_EQ(ground.scan_start_index,2);EXPECT_EQ(ground.scan_end_index,8);
  EXPECT_DOUBLE_EQ(ground.angle_threshold_deg,14.0);
  EXPECT_DOUBLE_EQ(ground.plane_distance,.18);
  EXPECT_DOUBLE_EQ(ground.plane_max_angle_deg,22.0);EXPECT_EQ(ground.plane_min_points,44);
  EXPECT_TRUE(ground.flatten_z);EXPECT_DOUBLE_EQ(ground.flatten_value,-.6);
  const auto& patch=ground.grid_patch;EXPECT_TRUE(patch.enable);
  EXPECT_DOUBLE_EQ(patch.resolution,.2);EXPECT_DOUBLE_EQ(patch.search_radius,.9);
  EXPECT_DOUBLE_EQ(patch.max_slope_deg,20.0);EXPECT_DOUBLE_EQ(patch.max_height_range,.16);
  EXPECT_DOUBLE_EQ(patch.max_roughness,.07);EXPECT_DOUBLE_EQ(patch.max_prediction_margin,.09);
  EXPECT_DOUBLE_EQ(patch.min_inlier_ratio,.8);EXPECT_EQ(patch.min_neighbor_cells,9);
  EXPECT_EQ(patch.min_support_quadrants,4);
}
}
TEST(RingPlaneConfig, LoadsEveryCanonicalGeometryAndPostprocessingField) {
  ExpectGroundParameters(Load(GroundParameters("ring_plane.")).mapping.ground_segmentation);
}
TEST(RingPlaneConfig, KeepsEveryDeprecatedGroupAlias) {
  ExpectGroundParameters(Load(GroundParameters("ground_segmentation.")).mapping.ground_segmentation);
}
TEST(RingPlaneConfig, KeepsAllFlatLegacyAliases) {
  const std::vector<std::string> flat={"segmentation_only_mode","groundLeafSize",
      "groundScanStartIndex","groundScanEndIndex","groundAngleThreshold",
      "groundPlaneDistance","groundPlaneMaxAngle","groundPlaneMinPoints",
      "groundFlattenZ","groundFlattenValue","groundGridPatchEnable",
      "groundGridPatchResolution","groundGridPatchSearchRadius","groundGridPatchMaxSlopeDeg",
      "groundGridPatchMaxHeightRange","groundGridPatchMaxRoughness",
      "groundGridPatchMaxPredictionMargin","groundGridPatchMinInlierRatio",
      "groundGridPatchMinNeighborCells","groundGridPatchMinSupportQuadrants"};
  const auto canonical=GroundParameters("ring_plane.");
  ASSERT_EQ(canonical.size(),flat.size()+1);
  std::vector<rclcpp::Parameter> overrides;
  for(std::size_t index=0;index<flat.size();++index)
    overrides.emplace_back(flat[index],canonical[index+1].get_parameter_value());
  ExpectGroundParameters(Load(overrides).mapping.ground_segmentation,"online");
}
TEST(RingPlaneConfig, EqualCanonicalGroupAndFlatAliasesWarnOnceEach) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"ring_plane.leaf_size",.25},
      {"ground_segmentation.leaf_size",.25},{"groundLeafSize",.25}});
  auto node=std::make_shared<rclcpp::Node>("ring_plane_alias_warning",options);
  ::testing::internal::CaptureStderr();
  const auto first=LoadSlamConfig(*node,SlamMode::kMapping);
  const auto second=LoadSlamConfig(*node,SlamMode::kMapping);
  const auto logs=::testing::internal::GetCapturedStderr();
  EXPECT_DOUBLE_EQ(first.mapping.ground_segmentation.leaf_size,.25);
  EXPECT_DOUBLE_EQ(second.mapping.ground_segmentation.leaf_size,.25);
  for(const std::string alias:{"ground_segmentation.leaf_size","groundLeafSize"}) {
    const std::string warning="Parameter '"+alias+"' is deprecated";
    const auto position=logs.find(warning);
    ASSERT_NE(position,std::string::npos)<<logs;
    EXPECT_EQ(logs.find(warning,position+warning.size()),std::string::npos);
  }
  EXPECT_NE(logs.find("use 'ring_plane.leaf_size'"),std::string::npos);
}
TEST(RingPlaneConfig, RejectsConflictingCanonicalGroupAndFlatAliases) {
  for(const auto& overrides:std::vector<std::vector<rclcpp::Parameter>>{
      {{"ring_plane.leaf_size",.25},{"ground_segmentation.leaf_size",.3}},
      {{"ring_plane.leaf_size",.25},{"groundLeafSize",.3}},
      {{"ground_segmentation.leaf_size",.25},{"groundLeafSize",.3}},
      {{"ring_plane.mode",std::string("export")},{"ground_segmentation.mode",std::string("online")}}}) {
    EXPECT_THROW(Load(overrides),ParameterAliasConflict);
  }
}
TEST(RingPlaneConfig, CanonicalValidationNamesRingPlane) {
  for(const auto& parameter:std::vector<rclcpp::Parameter>{
      {"ring_plane.mode",std::string("both")},{"ring_plane.leaf_size",-1.0},
      {"ring_plane.plane_distance",-1.0},{"ring_plane.plane_min_points",0},
      {"ring_plane.grid_patch.resolution",0.0},{"ring_plane.grid_patch.search_radius",0.0}}) {
    try { Load({parameter});FAIL()<<"accepted "<<parameter.get_name(); }
    catch(const std::invalid_argument& error) {
      EXPECT_NE(std::string(error.what()).find(parameter.get_name()),std::string::npos);
    }
  }
}
TEST(RingPlaneConfig, CanonicalModesRemainExactlyOffOnlineExport) {
  for(const std::string mode:{"off","online","export"})
    EXPECT_EQ(Load({{"ring_plane.mode",mode}}).mapping.ground_segmentation.mode,mode);
}
TEST(RingPlaneConfig, LocalizationKeepsOriginalCanonicalGroup) {
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args","--params-file",std::string(PROJECT_SOURCE_DIR)+"/config/relocalization.yaml"});
  options.parameter_overrides({{"ground_segmentation.leaf_size",.27},
      {"ring_plane.leaf_size",.49}});
  auto node=std::make_shared<rclcpp::Node>("localization_ground_unchanged",options);
  const auto config=LoadSlamConfig(*node,SlamMode::kLocalization);
  EXPECT_DOUBLE_EQ(config.localization.ground_segmentation.leaf_size,.27);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
