#include "common/slam_config.h"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

rclcpp::NodeOptions OptionsWith(
    const std::vector<rclcpp::Parameter>& overrides) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  return options;
}

SlamConfig LoadMapping(const std::vector<rclcpp::Parameter>& overrides = {}) {
  auto node = std::make_shared<rclcpp::Node>(
      "test_slam_config_mapping", OptionsWith(overrides));
  return LoadSlamConfig(*node, SlamMode::kMapping);
}

SlamConfig LoadLocalization(
    const std::vector<rclcpp::Parameter>& overrides = {}) {
  auto node = std::make_shared<rclcpp::Node>(
      "test_slam_config_localization", OptionsWith(overrides));
  return LoadSlamConfig(*node, SlamMode::kLocalization);
}

}  // namespace

TEST(SlamConfig, MappingDefaultsMatchPreP22TopicStrings) {
  const SlamConfig config = LoadMapping();

  EXPECT_EQ(config.mode, SlamMode::kMapping);
  EXPECT_EQ(config.outputs.odometry, "odometry/imu");
  EXPECT_EQ(config.outputs.path, "lio_sam/mapping/path");
  EXPECT_EQ(config.outputs.global_map, "lio_sam/mapping/map_global");
  EXPECT_EQ(config.outputs.local_map, "lio_sam/mapping/map_local");
  EXPECT_EQ(config.outputs.registered_cloud, "lio_sam/mapping/cloud_registered");
  EXPECT_EQ(config.outputs.registered_cloud_raw,
            "lio_sam/mapping/cloud_registered_raw");
  EXPECT_EQ(config.outputs.ground_map, "lio_sam/mapping/ground_cloud_global");
  EXPECT_EQ(config.outputs.trajectory, "lio_sam/mapping/trajectory");
  EXPECT_EQ(config.outputs.imu_path, "lio_sam/imu/path");
  EXPECT_EQ(config.outputs.diagnostics, "/diagnostics");
  EXPECT_EQ(config.outputs.non_ground_cloud, "lio_sam/deskew/cloud_nonground");

  EXPECT_EQ(config.pipeline_topics.mapping_odometry,
            "lio_sam/mapping/odometry");
  EXPECT_EQ(config.pipeline_topics.mapping_odometry_incremental,
            "lio_sam/mapping/odometry_incremental");
  EXPECT_EQ(config.pipeline_topics.imu_odometry_incremental,
            "odometry/imu_incremental");
  EXPECT_EQ(config.pipeline_topics.cloud_info, "lio_sam/deskew/cloud_info");
  EXPECT_EQ(config.pipeline_topics.feature_cloud_info,
            "lio_sam/feature/cloud_info");
  EXPECT_EQ(config.pipeline_topics.deskewed_cloud,
            "lio_sam/deskew/cloud_deskewed");
  EXPECT_EQ(config.pipeline_topics.ground_cloud,
            "lio_sam/deskew/ground_cloud_global");
  EXPECT_EQ(config.pipeline_topics.corner_cloud, "lio_sam/feature/cloud_corner");
  EXPECT_EQ(config.pipeline_topics.surface_cloud,
            "lio_sam/feature/cloud_surface");
  EXPECT_EQ(config.pipeline_topics.loop_closure_hint,
            "lio_loop/loop_closure_detection");

  // Mapping accepts no external initial pose.
  EXPECT_TRUE(config.command_topics.initial_pose.empty());
  EXPECT_EQ(config.outputs.current_lidar, "");
  EXPECT_EQ(config.frames.odometry, "odom");
}

TEST(SlamConfig, LocalizationDefaultsMatchPreP22TopicStrings) {
  const SlamConfig config = LoadLocalization();

  EXPECT_EQ(config.mode, SlamMode::kLocalization);
  EXPECT_EQ(config.outputs.odometry, "lio_sam/localization/odometry");
  EXPECT_EQ(config.outputs.path, "lio_sam/localization/path");
  EXPECT_EQ(config.outputs.global_map, "lio_sam/localization/global_map");
  EXPECT_EQ(config.outputs.local_map, "lio_sam/localization/local_map");
  EXPECT_EQ(config.outputs.current_lidar, "lio_sam/localization/current_lidar");
  EXPECT_EQ(config.command_topics.initial_pose, "/initialpose");

  // Localization publishes no mapping-only outputs and consumes no mapping
  // pipeline topics.
  EXPECT_EQ(config.outputs.registered_cloud, "");
  EXPECT_EQ(config.outputs.registered_cloud_raw, "");
  EXPECT_EQ(config.outputs.ground_map, "");
  EXPECT_EQ(config.outputs.trajectory, "");
  EXPECT_EQ(config.outputs.imu_path, "");
  EXPECT_EQ(config.pipeline_topics.mapping_odometry, "");
  EXPECT_EQ(config.pipeline_topics.imu_odometry_incremental, "");

  EXPECT_EQ(config.localization.localization.map_mode, "auto");
  EXPECT_EQ(config.localization.localization.sc_top_n, 8);
  EXPECT_TRUE(config.localization.health.enable);
  EXPECT_DOUBLE_EQ(config.localization.tf.publish_rate_hz, 20.0);
}

TEST(SlamConfig, CanonicalOverridesWin) {
  const SlamConfig config = LoadMapping({rclcpp::Parameter(
      "output_topic.odometry", "custom/odometry")});
  EXPECT_EQ(config.outputs.odometry, "custom/odometry");
}

TEST(SlamConfig, LegacyOnlyKeyIsAccepted) {
  const SlamConfig config =
      LoadMapping({rclcpp::Parameter("sensor_topic.odom_topic", "legacy/odom")});
  EXPECT_EQ(config.outputs.odometry, "legacy/odom");
  // The incremental pipeline topic is derived from the legacy prefix.
  EXPECT_EQ(config.pipeline_topics.imu_odometry_incremental,
            "legacy/odom_incremental");
}

TEST(SlamConfig, EqualDualValuesAreAccepted) {
  const SlamConfig config = LoadMapping(
      {rclcpp::Parameter("output_topic.odometry", "same/topic"),
       rclcpp::Parameter("sensor_topic.odom_topic", "same/topic")});
  EXPECT_EQ(config.outputs.odometry, "same/topic");
}

TEST(SlamConfig, ConflictingDualValuesAreRejected) {
  EXPECT_THROW(
      LoadMapping({rclcpp::Parameter("output_topic.odometry", "a/topic"),
                   rclcpp::Parameter("sensor_topic.odom_topic", "b/topic")}),
      ParameterAliasConflict);
}

TEST(SlamConfig, LocalizationLegacyOdomKeyIsAccepted) {
  const SlamConfig config = LoadLocalization({rclcpp::Parameter(
      "output_topic.mapping_odom_topic", "legacy/localization_odom")});
  EXPECT_EQ(config.outputs.odometry, "legacy/localization_odom");
}

TEST(SlamConfig, RejectsEmptyRequiredFrame) {
  EXPECT_THROW(LoadMapping({rclcpp::Parameter("frames.map", "")}),
               std::invalid_argument);
  EXPECT_THROW(LoadLocalization({rclcpp::Parameter("frames.base_link", "")}),
               std::invalid_argument);
  EXPECT_THROW(LoadMapping({rclcpp::Parameter("frames.lidar", "")}),
               std::invalid_argument);
}

TEST(SlamConfig, RejectsEmptyRequiredSensorTopic) {
  EXPECT_THROW(LoadMapping({rclcpp::Parameter("sensor_topic.lidar_topic", "")}),
               std::invalid_argument);
  EXPECT_THROW(LoadLocalization({rclcpp::Parameter("sensor_topic.imu_topic", "")}),
               std::invalid_argument);
}

TEST(SlamConfig, RejectsNonPositiveSizesAndRates) {
  EXPECT_THROW(LoadMapping({rclcpp::Parameter("lidar.scan_lines", 0)}),
               std::invalid_argument);
  EXPECT_THROW(LoadLocalization({rclcpp::Parameter("lidar.scan_lines", 0)}),
               std::invalid_argument);
  EXPECT_THROW(
      LoadLocalization(
          {rclcpp::Parameter("localization_health.nominal_lidar_rate_hz", 0.0)}),
      std::invalid_argument);
  EXPECT_THROW(
      LoadLocalization({rclcpp::Parameter("localization_tf.publish_rate_hz",
                                          -1.0)}),
      std::invalid_argument);
}

TEST(SlamConfig, RejectsNonFiniteValues) {
  EXPECT_THROW(
      LoadMapping({rclcpp::Parameter("imu.gravity",
                                     std::numeric_limits<double>::infinity())}),
      std::invalid_argument);
}

TEST(SlamConfig, RejectsInvalidCalibrationRotation) {
  // Not orthonormal: the first basis vector has length 2.
  const std::vector<double> bad_rotation = {2.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                            0.0, 0.0, 1.0};
  EXPECT_THROW(LoadMapping({rclcpp::Parameter("calibration.rotation",
                                              bad_rotation)}),
               std::invalid_argument);
}

TEST(SlamConfig, RejectsWrongCalibrationVectorLength) {
  EXPECT_THROW(
      LoadMapping({rclcpp::Parameter("calibration.translation",
                                     std::vector<double>{0.0, 0.0})}),
      std::invalid_argument);
}

TEST(SlamConfig, DerivesCanonicalImuToLidarCalibration) {
  const std::vector<double> rotation = {0.0, -1.0, 0.0, 1.0, 0.0, 0.0,
                                        0.0, 0.0, 1.0};
  const std::vector<double> translation = {0.0, 0.0, 0.10};
  const std::vector<double> identity = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                        0.0, 0.0, 1.0};
  const SlamConfig config =
      LoadMapping({rclcpp::Parameter("calibration.rotation", rotation),
                   rclcpp::Parameter("calibration.translation", translation),
                   rclcpp::Parameter("calibration.rpy", identity)});

  // The configured rotation is lidar->imu, i.e. already imu_T_lidar.
  const Eigen::Matrix3d expected_rotation =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  EXPECT_TRUE(config.calibration.imu_T_lidar.Rotation().isApprox(
      expected_rotation));
  EXPECT_TRUE(config.calibration.imu_T_lidar.Translation().isApprox(
      Eigen::Vector3d(0.0, 0.0, 0.10)));
  // The inverse is derived, not configured.
  EXPECT_TRUE((config.calibration.imu_T_lidar * config.calibration.lidar_T_imu)
                  .Matrix()
                  .isApprox(Eigen::Matrix4d::Identity()));
  // Mapping consumes the transposed rotation and a lever arm expressed in
  // lidar axes.
  EXPECT_TRUE(config.calibration.mapping_imu_to_lidar_rotation.isApprox(
      expected_rotation.transpose()));
  EXPECT_TRUE(config.calibration.mapping_imu_to_lidar_lever_arm.isApprox(
      Eigen::Vector3d(0.0, 0.0, 0.10)));
  EXPECT_TRUE(config.calibration.rpy_rotation.isApprox(
      Eigen::Matrix3d::Identity()));
}

TEST(SlamConfig, RejectsInvalidLocalizationTiledMapConfiguration) {
  EXPECT_THROW(LoadLocalization({rclcpp::Parameter("localization.map_mode",
                                                   "unsupported")}),
               std::invalid_argument);
  // max_cached_tiles must cover the active window (3x3 = 9).
  EXPECT_THROW(
      LoadLocalization({rclcpp::Parameter("localization.tiled_map.active_radius_tiles", 1),
                        rclcpp::Parameter("localization.tiled_map.prefetch_radius_tiles", 2),
                        rclcpp::Parameter("localization.tiled_map.max_cached_tiles", 4)}),
      std::invalid_argument);
  // prefetch radius must not be smaller than the active radius.
  EXPECT_THROW(
      LoadLocalization({rclcpp::Parameter("localization.tiled_map.active_radius_tiles", 2),
                        rclcpp::Parameter("localization.tiled_map.prefetch_radius_tiles", 1),
                        rclcpp::Parameter("localization.tiled_map.max_cached_tiles", 25)}),
      std::invalid_argument);
}

TEST(SlamConfig, RejectsExtrapolationBeyondResultTimeout) {
  EXPECT_THROW(
      LoadLocalization(
          {rclcpp::Parameter("localization_tf.max_extrapolation_seconds", 2.0),
           rclcpp::Parameter("localization_health.result_timeout", 1.0)}),
      std::invalid_argument);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
