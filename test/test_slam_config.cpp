#include "common/slam_config.h"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <algorithm>
#include <limits>
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
    const std::vector<rclcpp::Parameter>& overrides = {},
    const std::string& omit_fixture_key = "") {
  // Explicit minimum runtime fixture; topic defaults must not make an empty
  // algorithm configuration accidentally valid.
  std::vector<rclcpp::Parameter> parameters = {
      {"lidar.sensor_type", "Hesai"}, {"lidar.max_range", 60.0},
      {"lidar.rotation_noise_std", 0.01}, {"lidar.position_noise_std", 0.02},
      {"imu.acc_noise_std", 0.1}, {"imu.gyro_noise_std", 0.01},
      {"imu.acc_bias_rw_noise_std", 1e-5}, {"imu.gyro_bias_rw_noise_std", 1e-5},
      {"imu.data_searcher_buffer_size", 2},
      {"frontend.registration_and_searcher_mode", "PointToPlane_IVOX"},
      {"frontend.feature.planar_voxel_filter_size", 0.25},
      {"frontend.registration.optimization_iter_num", 10},
      {"frontend.registration.point_to_planar_thres", 0.1},
      {"frontend.registration.position_converge_thres", 0.01},
      {"frontend.registration.rotation_converge_thres", 0.01},
      {"frontend.fusion_method", "TightCouplingOptimization"},
      {"frontend.fusion_opti_iters", 30}};
  parameters.erase(std::remove_if(parameters.begin(), parameters.end(),
      [&](const auto& value) { return value.get_name() == omit_fixture_key; }),
      parameters.end());
  for (const auto& override : overrides) {
    parameters.erase(std::remove_if(parameters.begin(), parameters.end(),
        [&](const auto& value) { return value.get_name() == override.get_name(); }),
        parameters.end());
    parameters.push_back(override);
  }
  auto node = std::make_shared<rclcpp::Node>(
      "test_slam_config_localization", OptionsWith(parameters));
  return LoadSlamConfig(*node, SlamMode::kLocalization);
}

}  // namespace

TEST(SlamConfig, RejectsRawEmptyLocalizationConfiguration) {
  auto node = std::make_shared<rclcpp::Node>("empty_localization");
  EXPECT_THROW(LoadSlamConfig(*node, SlamMode::kLocalization), std::invalid_argument);
}

TEST(SlamConfig, RejectsInvalidRuntimeValuesWithCanonicalKey) {
  const std::vector<rclcpp::Parameter> invalid = {
      {"frontend.registration.optimization_iter_num", 0},
      {"frontend.registration.optimization_iter_num", -1},
      {"frontend.fusion_opti_iters", 0}, {"frontend.fusion_opti_iters", -1},
      {"imu.data_searcher_buffer_size", 1}, {"imu.data_searcher_buffer_size", -1},
      {"lidar.sensor_type", "unknown"}, {"lidar.min_range", -0.1},
      {"lidar.max_range", 0.0}, {"lidar.point_time_scale", 0.0},
      {"frontend.registration_and_searcher_mode", "LoamFull_KdTree"},
      {"frontend.fusion_method", "unknown"},
      {"frontend.feature.planar_voxel_filter_size", 0.0},
      {"frontend.registration.point_to_planar_thres", -1.0},
      {"frontend.registration.position_converge_thres", 0.0},
      {"frontend.registration.rotation_converge_thres", 0.0}};
  for (const auto& parameter : invalid) {
    SCOPED_TRACE(parameter.get_name());
    try { LoadLocalization({parameter}); FAIL() << "accepted invalid runtime value"; }
    catch (const std::invalid_argument& error) {
      EXPECT_NE(std::string(error.what()).find(parameter.get_name()), std::string::npos);
    }
  }
}

TEST(SlamConfig, RejectsInvalidNoiseAndNonFiniteBias) {
  for (const std::string key : {"imu.acc_noise_std", "imu.gyro_noise_std",
      "imu.acc_bias_rw_noise_std", "imu.gyro_bias_rw_noise_std",
      "lidar.rotation_noise_std", "lidar.position_noise_std"}) {
    for (double value : {0.0, -0.1, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::denorm_min()}) {
      SCOPED_TRACE(key);
      EXPECT_THROW(LoadLocalization({{key, value}}), std::invalid_argument);
    }
  }
  for (const std::string key : {"imu.init_acc_bias", "imu.init_gyro_bias"}) {
    EXPECT_THROW(LoadLocalization({{key, std::numeric_limits<double>::infinity()}}),
        std::invalid_argument);
  }
}

TEST(SlamConfig, RejectsInvalidRpyRotation) {
  for (const std::vector<double>& rpy : std::vector<std::vector<double>>{
      {2,0,0,0,1,0,0,0,1}, {-1,0,0,0,1,0,0,0,1},
      {1,0,0,0,1,0,0,0,std::numeric_limits<double>::quiet_NaN()}}) {
    EXPECT_THROW(LoadMapping({{"calibration.rpy", rpy}}), std::invalid_argument);
  }
}

TEST(SlamConfig, MappingFlatTopicAliasesAndLegacyConflicts) {
  const auto config = LoadMapping({{"odomTopic", "flat/odom"},
      {"nonGroundCloudTopic", "flat/nonground"}});
  EXPECT_EQ(config.outputs.odometry, "flat/odom");
  EXPECT_EQ(config.pipeline_topics.imu_odometry_incremental, "flat/odom_incremental");
  EXPECT_EQ(config.outputs.non_ground_cloud, "flat/nonground");
  EXPECT_NO_THROW(LoadMapping({{"odomTopic", "same"}, {"sensor_topic.odom_topic", "same"},
      {"output_topic.odometry", "same"}}));
  EXPECT_THROW(LoadMapping({{"odomTopic", "a"}, {"sensor_topic.odom_topic", "b"}}), ParameterAliasConflict);
  EXPECT_THROW(LoadMapping({{"odomTopic", "a"}, {"output_topic.odometry", "b"}}), ParameterAliasConflict);
  EXPECT_THROW(LoadMapping({{"odomTopic", "a"}, {"pipeline_topic.imu_odometry_incremental", "b"}}), ParameterAliasConflict);
  EXPECT_THROW(LoadMapping({{"nonGroundCloudTopic", "a"}, {"sensor_topic.non_ground_cloud_topic", "b"}}), ParameterAliasConflict);
}

TEST(SlamConfig, AcceptsOnlySelectedBackendAndFusionParameters) {
  // IVOX never consumes local map/NDT/corner parameters.
  EXPECT_NO_THROW(LoadLocalization({{"frontend.registration.ndt_capacity", -1},
      {"frontend.registration.local_map_size", -1},
      {"frontend.feature.corner_voxel_filter_size", -1.0}}));
  for (const std::string fusion : {"LooseCoupling", "TightCouplingKF"}) {
    EXPECT_NO_THROW(LoadLocalization({{"frontend.fusion_method", fusion},
        {"frontend.fusion_opti_iters", 0}}));
  }
}

TEST(SlamConfig, RejectsMissingRequiredRuntimeParameters) {
  for (const std::string key : {"frontend.registration.optimization_iter_num",
      "frontend.fusion_opti_iters", "imu.data_searcher_buffer_size",
      "lidar.sensor_type", "lidar.max_range", "imu.acc_noise_std",
      "frontend.feature.planar_voxel_filter_size"}) {
    SCOPED_TRACE(key);
    EXPECT_THROW(LoadLocalization({}, key), std::invalid_argument);
  }
}

TEST(SlamConfig, LocalizationSharedAliasesResolveAndRejectConflicts) {
  const auto sensor = LoadLocalization({{"sensor", "Velodyne_16"}}, "lidar.sensor_type");
  EXPECT_EQ(sensor.localization.lidar.sensor_type, "Velodyne_16");
  EXPECT_THROW(LoadLocalization({{"sensor", "Velodyne_16"}}), ParameterAliasConflict);
  EXPECT_THROW(LoadLocalization({{"sensor", "Velodyne_16"},
      {"lidar.lidar_sensor_type", "Hesai"}}, "lidar.sensor_type"), ParameterAliasConflict);
  for (const auto& keys : std::vector<std::pair<std::string, std::string>>{
      {"N_SCAN", "lidar.scan_lines"}, {"Horizon_SCAN", "lidar.horizon_scan"}}) {
    const auto legacy = LoadLocalization({{keys.first, 32}});
    EXPECT_EQ(keys.first == "N_SCAN" ? legacy.localization.lidar.scan_lines :
        legacy.localization.lidar.horizon_scan, 32);
    EXPECT_NO_THROW(LoadLocalization({{keys.first, 32}, {keys.second, 32}}));
    EXPECT_THROW(LoadLocalization({{keys.first, 32}, {keys.second, 64}}), ParameterAliasConflict);
  }
  const std::vector<std::pair<std::string, std::string>> aliases = {
      {"lidarMinRange", "lidar.min_range"}, {"lidarMaxRange", "lidar.max_range"},
      {"imuAccNoise", "imu.acc_noise_std"}, {"imuGyrNoise", "imu.gyro_noise_std"},
      {"imuAccBiasN", "imu.acc_bias_rw_noise_std"},
      {"imuGyrBiasN", "imu.gyro_bias_rw_noise_std"}, {"imuGravity", "imu.gravity"}};
  for (const auto& keys : aliases) {
    SCOPED_TRACE(keys.first);
    EXPECT_NO_THROW(LoadLocalization({{keys.first, 0.2}}, keys.second));
    EXPECT_NO_THROW(LoadLocalization({{keys.first, 0.2}, {keys.second, 0.2}}));
    EXPECT_THROW(LoadLocalization({{keys.first, 0.2}, {keys.second, 0.3}}), ParameterAliasConflict);
  }
  EXPECT_THROW(LoadLocalization({{"imuAccBiasN", 0.2},
      {"imu.acc_rw_noise_std", 0.3}}, "imu.acc_bias_rw_noise_std"), ParameterAliasConflict);
  EXPECT_THROW(LoadLocalization({{"imuGravity", 9.8}, {"gravity", 9.7}}), ParameterAliasConflict);
}

TEST(SlamConfig, RepositoryCanonicalYamlConfigurationsAreValid) {
  for (const auto mode : {SlamMode::kMapping, SlamMode::kLocalization}) {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", std::string(PROJECT_SOURCE_DIR) +
        (mode == SlamMode::kMapping ? "/config/mapping.yaml" : "/config/relocalization.yaml")});
    auto node = std::make_shared<rclcpp::Node>("canonical_yaml", options);
    EXPECT_NO_THROW(LoadSlamConfig(*node, mode));
  }
}

TEST(SlamConfig, BackendSpecificRequirementsAreValidated) {
  for (const std::string backend : {"IcpOptimized", "IncrementalNDT", "PointToPlane_KdTree"}) {
    const std::vector<rclcpp::Parameter> valid = {
        {"frontend.registration_and_searcher_mode", backend},
        {"frontend.registration.local_map_cloud_filter_size", 0.3},
        {"frontend.registration.source_cloud_filter_size", 0.3},
        {"frontend.registration.point_search_thres", 1.0},
        {"frontend.registration.ndt_voxel_size", 1.0},
        {"frontend.registration.ndt_outlier_threshold", 20.0},
        {"frontend.registration.ndt_min_points_in_voxel", 3},
        {"frontend.registration.ndt_max_points_in_voxel", 50},
        {"frontend.registration.ndt_min_effective_pts", 10},
        {"frontend.registration.ndt_capacity", 1000}};
    EXPECT_NO_THROW(LoadLocalization(valid));
    const std::vector<std::string> required = backend == "IncrementalNDT" ?
        std::vector<std::string>{"source_cloud_filter_size", "ndt_voxel_size", "ndt_outlier_threshold"} :
        (backend == "IcpOptimized" ? std::vector<std::string>{"local_map_cloud_filter_size",
            "source_cloud_filter_size", "point_search_thres"} :
            std::vector<std::string>{"local_map_cloud_filter_size"});
    for (const auto& leaf : required) {
      auto invalid = valid;
      const std::string key = "frontend.registration." + leaf;
      invalid.erase(std::remove_if(invalid.begin(), invalid.end(),
          [&](const auto& p) { return p.get_name() == key; }), invalid.end());
      SCOPED_TRACE(backend + " " + key);
      EXPECT_THROW(LoadLocalization(invalid), std::invalid_argument);
      for (const double value : {0.0, -1.0, std::numeric_limits<double>::infinity()}) {
        invalid.push_back({key, value});
        EXPECT_THROW(LoadLocalization(invalid), std::invalid_argument);
        invalid.pop_back();
      }
    }
    if (backend == "IncrementalNDT") {
      for (const std::string leaf : {"ndt_min_points_in_voxel", "ndt_max_points_in_voxel",
          "ndt_min_effective_pts", "ndt_capacity"}) {
        auto invalid = valid;
        const std::string key = "frontend.registration." + leaf;
        invalid.erase(std::remove_if(invalid.begin(), invalid.end(),
            [&](const auto& p) { return p.get_name() == key; }), invalid.end());
        EXPECT_THROW(LoadLocalization(invalid), std::invalid_argument);
        for (const int value : {0, -1}) {
          invalid.push_back({key, value});
          EXPECT_THROW(LoadLocalization(invalid), std::invalid_argument);
          invalid.pop_back();
        }
      }
      auto inconsistent = valid;
      inconsistent.push_back({"frontend.registration.ndt_min_points_in_voxel", 50});
      EXPECT_THROW(LoadLocalization(inconsistent), std::invalid_argument);
    }
  }
}

TEST(SlamConfig, WarnsOnceForEachLegacyAliasPerNodeAcrossRepeatedReads) {
  auto node = std::make_shared<rclcpp::Node>("warning_capture", OptionsWith({
      {"output_topic.odometry", "same"}, {"odomTopic", "same"},
      {"sensor_topic.odom_topic", "same"},
      {"nonGroundCloudTopic", "cloud"}, {"sensor_topic.non_ground_cloud_topic", "cloud"}}));
  ::testing::internal::CaptureStderr();
  const auto first = LoadSlamConfig(*node, SlamMode::kMapping);
  const auto second = LoadSlamConfig(*node, SlamMode::kMapping);
  const std::string logs = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(first.outputs.odometry, second.outputs.odometry);
  EXPECT_EQ(second.pipeline_topics.imu_odometry_incremental, "same_incremental");
  for (const std::string key : {"odomTopic", "sensor_topic.odom_topic",
      "nonGroundCloudTopic", "sensor_topic.non_ground_cloud_topic"}) {
    const std::string warning = "Parameter '" + key + "' is deprecated";
    const auto position = logs.find(warning);
    ASSERT_NE(position, std::string::npos) << key;
    EXPECT_EQ(logs.find(warning, position + warning.size()), std::string::npos) << key;
  }
}

TEST(SlamConfig, MappingRejectsInvalidConsumedRuntimeParameters) {
  for (const auto& parameter : std::vector<rclcpp::Parameter>{
      {"lidar.sensor_type", "unknown"}, {"lidar.min_range", -1.0},
      {"lidar.max_range", 0.0}, {"imu.acc_noise_std", 0.0},
      {"imu.gyro_noise_std", -1.0}, {"imu.acc_bias_rw_noise_std", 0.0},
      {"imu.gyro_bias_rw_noise_std", std::numeric_limits<double>::infinity()},
      {"voxel_filter.odometry_surface_leaf_size", 0.0},
      {"voxel_filter.mapping_corner_leaf_size", 0.0},
      {"voxel_filter.mapping_surface_leaf_size", 0.0},
      {"local_map.keyframe_density", 0.0}, {"local_map.search_radius", 0.0}}) {
    SCOPED_TRACE(parameter.get_name());
    EXPECT_THROW(LoadMapping({parameter}), std::invalid_argument);
  }
  EXPECT_NO_THROW(LoadMapping({{"ground_segmentation.leaf_size", 0.0},
      {"local_map.keyframe_distance_threshold", 0.0},
      {"local_map.keyframe_angle_threshold", 0.0}}));
}

TEST(SlamConfig, MappingRejectsNonPositiveOrNonFiniteTileSize) {
  // tile_map_generator.cpp rejects a non-positive tile size when generation
  // runs; the configuration has to reject it up front instead of accepting a
  // config that can only fail later.
  for (const double tile_size :
       {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
    SCOPED_TRACE(tile_size);
    EXPECT_THROW(LoadMapping({rclcpp::Parameter(
                     "map_save.tiled_map.tile_size_m", tile_size)}),
                 std::invalid_argument);
  }
  const SlamConfig accepted = LoadMapping(
      {rclcpp::Parameter("map_save.tiled_map.tile_size_m", 25.0)});
  EXPECT_DOUBLE_EQ(accepted.mapping.map_save.tiled_map.tile_size_m, 25.0);
}

TEST(SlamConfig, RejectsRemovedShutdownAutoSaveParameters) {
  // Ctrl+C auto-save was deleted. Both the canonical key and its flat alias
  // must now fail closed at startup instead of being silently accepted, and
  // the error must name the replacement interface.
  for (const std::string key : {"map_save.enabled", "savePCD"}) {
    for (const bool value : {false, true}) {
      SCOPED_TRACE(key + "=" + (value ? "true" : "false"));
      try {
        LoadMapping({rclcpp::Parameter(key, value)});
        FAIL() << "accepted removed shutdown auto-save parameter";
      } catch (const std::invalid_argument& error) {
        const std::string message = error.what();
        EXPECT_NE(message.find(key), std::string::npos);
        EXPECT_NE(message.find("/lio_sam/save_map"), std::string::npos);
      }
    }
  }
}

TEST(SlamConfig, LocalizationMultiAliasesEachWarnOnceWithCanonicalYaml) {
  const std::vector<rclcpp::Parameter> aliases = {
      {"sensor", "Hesai"}, {"lidar.lidar_sensor_type", "Hesai"},
      {"N_SCAN", 16}, {"lidar.lidar_scan", 16},
      {"Horizon_SCAN", 1000}, {"lidar.lidar_horizon_scan", 1000},
      {"lidarMinRange", 0.5}, {"lidar.lidar_use_min_distance", 0.5},
      {"lidarMaxRange", 60.0}, {"lidar.lidar_use_max_distance", 60.0},
      {"imuAccNoise", 0.1}, {"imuGyrNoise", 0.01},
      {"imuAccBiasN", 1e-5}, {"imu.acc_rw_noise_std", 1e-5},
      {"imuGyrBiasN", 1e-5}, {"imu.gyro_rw_noise_std", 1e-5},
      {"imuGravity", 9.805}, {"gravity", 9.805},
      {"loopClosureEnableFlag", false}, {"system.enable_loopclosure", false}};
  // Two distinct nodes must each report every supplied alias, despite having
  // identical names. Repeated full loads of either node remain idempotent.
  for (int i = 0; i < 2; ++i) {
    auto options = OptionsWith(aliases);
    options.arguments({"--ros-args", "--params-file", std::string(PROJECT_SOURCE_DIR) +
        "/config/relocalization.yaml"});
    auto node = std::make_shared<rclcpp::Node>("localization_alias_capture", options);
    ::testing::internal::CaptureStderr();
    LoadSlamConfig(*node, SlamMode::kLocalization);
    LoadSlamConfig(*node, SlamMode::kLocalization);
    const std::string logs = ::testing::internal::GetCapturedStderr();
    for (const auto& alias : aliases) {
      const std::string warning = "Parameter '" + alias.get_name() + "' is deprecated";
      const auto position = logs.find(warning);
      ASSERT_NE(position, std::string::npos) << alias.get_name();
      EXPECT_EQ(logs.find(warning, position + warning.size()), std::string::npos)
          << alias.get_name();
    }
  }
}

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
