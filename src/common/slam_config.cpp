#include "common/slam_config.h"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/lidar_imu_calibration.h"

namespace {

using slam_config_detail::ReadOverride;
using slam_config_detail::ResolveAndWarn;

const char* ModeName(SlamMode mode) {
  return mode == SlamMode::kMapping ? "mapping" : "localization";
}

[[noreturn]] void Reject(const std::string& name, const std::string& why) {
  throw std::invalid_argument(name + " " + why);
}

void RequireNonEmpty(const std::string& value, const std::string& name) {
  if (value.empty()) {
    Reject(name, "must not be empty");
  }
}

void RequireFinite(double value, const std::string& name) {
  if (!std::isfinite(value)) {
    Reject(name, "must be finite");
  }
}

void RequirePositive(double value, const std::string& name) {
  RequireFinite(value, name);
  if (value <= 0.0) {
    Reject(name, "must be greater than zero");
  }
}

void RequireNonNegative(double value, const std::string& name) {
  RequireFinite(value, name);
  if (value < 0.0) {
    Reject(name, "must not be negative");
  }
}

void RequirePositive(int value, const std::string& name) {
  if (value <= 0) {
    Reject(name, "must be greater than zero");
  }
}

void RequireNonNegative(int value, const std::string& name) {
  if (value < 0) {
    Reject(name, "must not be negative");
  }
}

template <typename T>
T ReadParam(rclcpp::Node& node, const std::string& canonical,
            const std::optional<std::string>& legacy, const T& default_value,
            LegacyWarningSet& warnings) {
  return DeclareRead<T>(node, canonical, legacy, default_value, warnings);
}

// Reads a canonical key whose legacy counterpart is a derived value rather
// than a separately declared key (for example the incremental odometry topic
// built from the legacy odometry prefix).
std::string ReadDerivedLegacy(rclcpp::Node& node, const std::string& canonical,
                              const std::optional<std::string>& legacy_key,
                              const std::optional<std::string>& legacy_value,
                              const std::string& default_value,
                              LegacyWarningSet& warnings) {
  const std::optional<std::string> canonical_value =
      ReadOverride<std::string>(node, canonical, default_value);
  return ResolveAndWarn<std::string>(node, canonical, legacy_key,
                                     canonical_value, legacy_value,
                                     default_value, warnings);
}

void LoadFrames(rclcpp::Node& node, SlamMode mode, SlamConfig& config,
                LegacyWarningSet& warnings) {
  FrameIds& frames = config.frames;
  frames.map = ReadParam<std::string>(node, "frames.map",
                                      std::string("mapFrame"), frames.map,
                                      warnings);
  frames.base_link = ReadParam<std::string>(
      node, "frames.base_link", std::string("baselinkFrame"), frames.base_link,
      warnings);
  frames.lidar = ReadParam<std::string>(node, "frames.lidar",
                                        std::string("lidarFrame"),
                                        frames.lidar, warnings);
  frames.imu = ReadParam<std::string>(node, "frames.imu", std::nullopt,
                                      frames.imu, warnings);
  if (mode == SlamMode::kMapping) {
    frames.odometry = ReadParam<std::string>(
        node, "frames.odometry", std::string("odometryFrame"), frames.odometry,
        warnings);
  }

  RequireNonEmpty(frames.map, "frames.map");
  RequireNonEmpty(frames.base_link, "frames.base_link");
  RequireNonEmpty(frames.lidar, "frames.lidar");
  RequireNonEmpty(frames.imu, "frames.imu");
}

void LoadCalibration(rclcpp::Node& node, SlamConfig& config,
                     LegacyWarningSet& warnings) {
  const std::vector<double> identity = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                        0.0, 0.0, 1.0};
  const std::vector<double> default_translation = {0.0, 0.0, 0.10};

  const std::vector<double> translation = ReadParam<std::vector<double>>(
      node, "calibration.translation", std::string("extrinsicTrans"),
      default_translation, warnings);
  const std::vector<double> rotation = ReadParam<std::vector<double>>(
      node, "calibration.rotation", std::string("extrinsicRot"), identity,
      warnings);
  const std::vector<double> rpy = ReadParam<std::vector<double>>(
      node, "calibration.rpy", std::string("extrinsicRPY"), identity, warnings);

  // The parser validates finiteness, the homogeneous last row, orthonormality
  // and a +1 determinant, so a malformed rotation is rejected here.
  LidarImuCalibration parsed;
  try {
    parsed = ParseLidarImuCalibration(translation, rotation, rpy);
  } catch (const std::exception& error) {
    Reject("calibration", std::string("is invalid: ") + error.what());
  }

  config.calibration.imu_T_lidar = parsed.imu_T_lidar;
  config.calibration.lidar_T_imu = parsed.lidar_T_imu;
  config.calibration.mapping_imu_to_lidar_rotation =
      parsed.mapping_imu_to_lidar_rotation;
  config.calibration.mapping_imu_to_lidar_lever_arm =
      parsed.mapping_imu_to_lidar_lever_arm;
  config.calibration.rpy_rotation = parsed.mapping_rpy_rotation;
}

void LoadCommonTopics(rclcpp::Node& node, SlamMode mode, SlamConfig& config,
                      LegacyWarningSet& warnings) {
  const bool mapping = mode == SlamMode::kMapping;
  PipelineTopics& pipeline = config.pipeline_topics;
  CommandTopics& command = config.command_topics;
  OutputTopics& outputs = config.outputs;

  config.sensors.lidar = ReadParam<std::string>(
      node, "sensor_topic.lidar_topic", std::string("pointCloudTopic"),
      config.sensors.lidar, warnings);
  config.sensors.imu = ReadParam<std::string>(
      node, "sensor_topic.imu_topic", std::string("imuTopic"),
      config.sensors.imu, warnings);
  if (mapping) {
    config.sensors.gps = ReadParam<std::string>(
        node, "sensor_topic.gps_topic", std::string("gpsTopic"),
        config.sensors.gps, warnings);
  }

  pipeline.cloud_info = ReadParam<std::string>(
      node, "pipeline_topic.cloud_info", std::nullopt, pipeline.cloud_info,
      warnings);
  pipeline.feature_cloud_info = ReadParam<std::string>(
      node, "pipeline_topic.feature_cloud_info", std::nullopt,
      pipeline.feature_cloud_info, warnings);
  pipeline.deskewed_cloud = ReadParam<std::string>(
      node, "pipeline_topic.deskewed_cloud", std::nullopt,
      pipeline.deskewed_cloud, warnings);
  pipeline.ground_cloud = ReadParam<std::string>(
      node, "pipeline_topic.ground_cloud", std::nullopt, pipeline.ground_cloud,
      warnings);
  pipeline.corner_cloud = ReadParam<std::string>(
      node, "pipeline_topic.corner_cloud", std::nullopt, pipeline.corner_cloud,
      warnings);
  pipeline.surface_cloud = ReadParam<std::string>(
      node, "pipeline_topic.surface_cloud", std::nullopt,
      pipeline.surface_cloud, warnings);
  pipeline.loop_closure_hint = ReadParam<std::string>(
      node, "pipeline_topic.loop_closure_hint", std::nullopt,
      pipeline.loop_closure_hint, warnings);

  // Modes that do not consume a pipeline topic must leave it empty so no
  // subscriber or publisher is created for it.
  pipeline.mapping_odometry.clear();
  pipeline.mapping_odometry_incremental.clear();
  pipeline.imu_odometry_incremental.clear();
  if (mapping) {
    pipeline.mapping_odometry = ReadParam<std::string>(
        node, "pipeline_topic.mapping_odometry", std::nullopt,
        std::string("lio_sam/mapping/odometry"), warnings);
    pipeline.mapping_odometry_incremental = ReadParam<std::string>(
        node, "pipeline_topic.mapping_odometry_incremental", std::nullopt,
        std::string("lio_sam/mapping/odometry_incremental"), warnings);
  }

  const std::string default_odometry =
      mapping ? "odometry/imu" : "lio_sam/localization/odometry";
  const std::optional<std::string> odometry_legacy =
      mapping ? std::optional<std::string>("sensor_topic.odom_topic")
              : std::optional<std::string>("output_topic.mapping_odom_topic");
  outputs.odometry = ReadParam<std::string>(
      node, "output_topic.odometry", odometry_legacy, default_odometry,
      warnings);

  const std::string default_incremental =
      mapping ? "odometry/imu_incremental" : std::string();
  std::optional<std::string> legacy_incremental;
  if (mapping) {
    const std::optional<std::string> legacy_prefix =
        ReadOverride<std::string>(node, "sensor_topic.odom_topic",
                                  std::string());
    if (legacy_prefix.has_value()) {
      legacy_incremental = legacy_prefix.value() + "_incremental";
    }
  }
  pipeline.imu_odometry_incremental = ReadDerivedLegacy(
      node, "pipeline_topic.imu_odometry_incremental",
      mapping ? std::optional<std::string>("sensor_topic.odom_topic")
              : std::nullopt,
      legacy_incremental, default_incremental, warnings);

  const std::string empty;
  outputs.path = ReadParam<std::string>(
      node, "output_topic.path", std::nullopt,
      mapping ? std::string("lio_sam/mapping/path")
              : std::string("lio_sam/localization/path"),
      warnings);
  outputs.global_map = ReadParam<std::string>(
      node, "output_topic.global_map", std::nullopt,
      mapping ? std::string("lio_sam/mapping/map_global")
              : std::string("lio_sam/localization/global_map"),
      warnings);
  outputs.local_map = ReadParam<std::string>(
      node, "output_topic.local_map", std::nullopt,
      mapping ? std::string("lio_sam/mapping/map_local")
              : std::string("lio_sam/localization/local_map"),
      warnings);
  outputs.current_lidar = ReadParam<std::string>(
      node, "output_topic.current_lidar", std::nullopt,
      mapping ? empty : std::string("lio_sam/localization/current_lidar"),
      warnings);
  outputs.registered_cloud = ReadParam<std::string>(
      node, "output_topic.registered_cloud", std::nullopt,
      mapping ? std::string("lio_sam/mapping/cloud_registered") : empty,
      warnings);
  outputs.registered_cloud_raw = ReadParam<std::string>(
      node, "output_topic.registered_cloud_raw", std::nullopt,
      mapping ? std::string("lio_sam/mapping/cloud_registered_raw") : empty,
      warnings);
  outputs.ground_map = ReadParam<std::string>(
      node, "output_topic.ground_map", std::nullopt,
      mapping ? std::string("lio_sam/mapping/ground_cloud_global") : empty,
      warnings);
  outputs.trajectory = ReadParam<std::string>(
      node, "output_topic.trajectory", std::nullopt,
      mapping ? std::string("lio_sam/mapping/trajectory") : empty, warnings);
  outputs.imu_path = ReadParam<std::string>(
      node, "output_topic.imu_path", std::nullopt,
      mapping ? std::string("lio_sam/imu/path") : empty, warnings);
  outputs.loop_history = ReadParam<std::string>(
      node, "output_topic.loop_history", std::nullopt,
      mapping ? std::string("lio_sam/mapping/icp_loop_closure_history_cloud")
              : empty,
      warnings);
  outputs.loop_constraints = ReadParam<std::string>(
      node, "output_topic.loop_constraints", std::nullopt,
      mapping ? std::string("/lio_sam/mapping/loop_closure_constraints")
              : empty,
      warnings);
  outputs.non_ground_cloud = ReadParam<std::string>(
      node, "output_topic.non_ground_cloud_topic",
      std::string("sensor_topic.non_ground_cloud_topic"),
      mapping ? std::string("lio_sam/deskew/cloud_nonground") : empty,
      warnings);
  outputs.diagnostics = ReadParam<std::string>(
      node, "output_topic.diagnostics", std::nullopt,
      std::string("/diagnostics"), warnings);

  command.initial_pose = ReadParam<std::string>(
      node, "command_topic.initial_pose", std::nullopt,
      mapping ? empty : std::string("/initialpose"), warnings);

  RequireNonEmpty(config.sensors.lidar, "sensor_topic.lidar_topic");
  RequireNonEmpty(config.sensors.imu, "sensor_topic.imu_topic");
}

void LoadGroundSegmentation(rclcpp::Node& node,
                            GroundSegmentationConfig& ground,
                            LegacyWarningSet& warnings) {
  ground.only_mode = ReadParam<bool>(
      node, "ground_segmentation.only_mode", std::string("segmentation_only_mode"),
      ground.only_mode, warnings);
  ground.leaf_size = ReadParam<double>(
      node, "ground_segmentation.leaf_size", std::string("groundLeafSize"),
      ground.leaf_size, warnings);
  ground.scan_start_index = ReadParam<int>(
      node, "ground_segmentation.scan_start_index",
      std::string("groundScanStartIndex"), ground.scan_start_index, warnings);
  ground.scan_end_index = ReadParam<int>(
      node, "ground_segmentation.scan_end_index",
      std::string("groundScanEndIndex"), ground.scan_end_index, warnings);
  ground.angle_threshold_deg = ReadParam<double>(
      node, "ground_segmentation.angle_threshold_deg",
      std::string("groundAngleThreshold"), ground.angle_threshold_deg,
      warnings);
  ground.plane_distance = ReadParam<double>(
      node, "ground_segmentation.plane_distance",
      std::string("groundPlaneDistance"), ground.plane_distance, warnings);
  ground.plane_max_angle_deg = ReadParam<double>(
      node, "ground_segmentation.plane_max_angle_deg",
      std::string("groundPlaneMaxAngle"), ground.plane_max_angle_deg, warnings);
  ground.plane_min_points = ReadParam<int>(
      node, "ground_segmentation.plane_min_points",
      std::string("groundPlaneMinPoints"), ground.plane_min_points, warnings);
  ground.flatten_z = ReadParam<bool>(
      node, "ground_segmentation.flatten_z", std::string("groundFlattenZ"),
      ground.flatten_z, warnings);
  ground.flatten_value = ReadParam<double>(
      node, "ground_segmentation.flatten_value",
      std::string("groundFlattenValue"), ground.flatten_value, warnings);

  GroundGridPatchConfig& patch = ground.grid_patch;
  patch.enable = ReadParam<bool>(
      node, "ground_segmentation.grid_patch.enable",
      std::string("groundGridPatchEnable"), patch.enable, warnings);
  patch.resolution = ReadParam<double>(
      node, "ground_segmentation.grid_patch.resolution",
      std::string("groundGridPatchResolution"), patch.resolution, warnings);
  patch.search_radius = ReadParam<double>(
      node, "ground_segmentation.grid_patch.search_radius",
      std::string("groundGridPatchSearchRadius"), patch.search_radius, warnings);
  patch.max_slope_deg = ReadParam<double>(
      node, "ground_segmentation.grid_patch.max_slope_deg",
      std::string("groundGridPatchMaxSlopeDeg"), patch.max_slope_deg, warnings);
  patch.max_height_range = ReadParam<double>(
      node, "ground_segmentation.grid_patch.max_height_range",
      std::string("groundGridPatchMaxHeightRange"), patch.max_height_range,
      warnings);
  patch.max_roughness = ReadParam<double>(
      node, "ground_segmentation.grid_patch.max_roughness",
      std::string("groundGridPatchMaxRoughness"), patch.max_roughness, warnings);
  patch.max_prediction_margin = ReadParam<double>(
      node, "ground_segmentation.grid_patch.max_prediction_margin",
      std::string("groundGridPatchMaxPredictionMargin"),
      patch.max_prediction_margin, warnings);
  patch.min_inlier_ratio = ReadParam<double>(
      node, "ground_segmentation.grid_patch.min_inlier_ratio",
      std::string("groundGridPatchMinInlierRatio"), patch.min_inlier_ratio,
      warnings);
  patch.min_neighbor_cells = ReadParam<int>(
      node, "ground_segmentation.grid_patch.min_neighbor_cells",
      std::string("groundGridPatchMinNeighborCells"), patch.min_neighbor_cells,
      warnings);
  patch.min_support_quadrants = ReadParam<int>(
      node, "ground_segmentation.grid_patch.min_support_quadrants",
      std::string("groundGridPatchMinSupportQuadrants"),
      patch.min_support_quadrants, warnings);

  RequireNonNegative(ground.leaf_size, "ground_segmentation.leaf_size");
  RequireNonNegative(ground.plane_distance,
                     "ground_segmentation.plane_distance");
  RequirePositive(ground.plane_min_points,
                  "ground_segmentation.plane_min_points");
  RequirePositive(patch.resolution,
                  "ground_segmentation.grid_patch.resolution");
  RequirePositive(patch.search_radius,
                  "ground_segmentation.grid_patch.search_radius");
}

void LoadMappingConfig(rclcpp::Node& node, SlamConfig& config,
                       LegacyWarningSet& warnings) {
  MappingConfig& mapping = config.mapping;

  GpsConfig& gps = mapping.gps;
  gps.use_imu_heading_initialization =
      ReadParam<bool>(node, "gps.use_imu_heading_initialization",
                      std::string("useImuHeadingInitialization"),
                      gps.use_imu_heading_initialization, warnings);
  gps.use_elevation = ReadParam<bool>(
      node, "gps.use_elevation", std::string("useGpsElevation"),
      gps.use_elevation, warnings);
  gps.covariance_threshold = ReadParam<double>(
      node, "gps.covariance_threshold", std::string("gpsCovThreshold"),
      gps.covariance_threshold, warnings);
  gps.pose_covariance_threshold = ReadParam<double>(
      node, "gps.pose_covariance_threshold", std::string("poseCovThreshold"),
      gps.pose_covariance_threshold, warnings);

  MapSaveConfig& map_save = mapping.map_save;
  map_save.enabled = ReadParam<bool>(node, "map_save.enabled",
                                     std::string("savePCD"),
                                     map_save.enabled, warnings);
  map_save.save_tiled_map = ReadParam<bool>(
      node, "map_save.save_tiled_map", std::nullopt, map_save.save_tiled_map,
      warnings);
  map_save.tiled_map.tile_size_m = ReadParam<double>(
      node, "map_save.tiled_map.tile_size_m", std::nullopt,
      map_save.tiled_map.tile_size_m, warnings);
  map_save.tiled_map.output_subdirectory = ReadParam<std::string>(
      node, "map_save.tiled_map.output_subdirectory", std::nullopt,
      map_save.tiled_map.output_subdirectory, warnings);
  map_save.tiled_map.overview_resolution_m = ReadParam<double>(
      node, "map_save.tiled_map.overview_resolution_m", std::nullopt,
      map_save.tiled_map.overview_resolution_m, warnings);
  map_save.generate_scan_context_db = ReadParam<bool>(
      node, "map_save.generate_scan_context_db", std::string("generateScDb"),
      map_save.generate_scan_context_db, warnings);
  map_save.directory = ReadParam<std::string>(
      node, "map_save.directory", std::string("savePCDDirectory"),
      map_save.directory, warnings);
  map_save.resolution = ReadParam<double>(
      node, "map_save.resolution", std::string("savePCDResolution"),
      map_save.resolution, warnings);
  map_save.ground_resolution = ReadParam<double>(
      node, "map_save.ground_resolution",
      std::string("saveGroundPCDResolution"), map_save.ground_resolution,
      warnings);
  map_save.save_ground_map = ReadParam<bool>(
      node, "map_save.save_ground_map", std::string("saveGroundPCD"),
      map_save.save_ground_map, warnings);
  map_save.ground_directory = ReadParam<std::string>(
      node, "map_save.ground_directory", std::string("groundPCDDirectory"),
      map_save.ground_directory, warnings);
  map_save.ground_global_directory = ReadParam<std::string>(
      node, "map_save.ground_global_directory",
      std::string("groundPCDDirectoryGlobal"), map_save.ground_global_directory,
      warnings);
  map_save.ground_final_use_odometry = ReadParam<bool>(
      node, "map_save.ground_final_use_odometry",
      std::string("groundFinalUseOdom"), map_save.ground_final_use_odometry,
      warnings);
  map_save.ground_global_leaf_size = ReadParam<double>(
      node, "map_save.ground_global_leaf_size",
      std::string("groundGlobalLeafSize"), map_save.ground_global_leaf_size,
      warnings);
  map_save.align_ground_to_zero = ReadParam<bool>(
      node, "map_save.align_ground_to_zero",
      std::string("alignSavedMapGroundToZero"), map_save.align_ground_to_zero,
      warnings);
  map_save.ground_z_offset = ReadParam<double>(
      node, "map_save.ground_z_offset", std::string("savedMapGroundZOffset"),
      map_save.ground_z_offset, warnings);

  MappingSystemConfig& system = mapping.system;
  system.mapping_mode = ReadParam<bool>(node, "system.mapping_mode",
                                        std::string("mappingMode"),
                                        system.mapping_mode, warnings);
  system.number_of_cores = ReadParam<int>(
      node, "system.number_of_cores", std::string("numberOfCores"),
      system.number_of_cores, warnings);
  system.mapping_process_interval = ReadParam<double>(
      node, "system.mapping_process_interval",
      std::string("mappingProcessInterval"), system.mapping_process_interval,
      warnings);

  MappingLidarConfig& lidar = mapping.lidar;
  lidar.sensor_type = ReadParam<std::string>(
      node, "lidar.sensor_type", std::string("sensor"), lidar.sensor_type,
      warnings);
  lidar.scan_lines = ReadParam<int>(node, "lidar.scan_lines",
                                    std::string("N_SCAN"), lidar.scan_lines,
                                    warnings);
  lidar.horizon_scan = ReadParam<int>(
      node, "lidar.horizon_scan", std::string("Horizon_SCAN"),
      lidar.horizon_scan, warnings);
  lidar.downsample_rate = ReadParam<int>(
      node, "lidar.downsample_rate", std::string("downsampleRate"),
      lidar.downsample_rate, warnings);
  lidar.min_range = ReadParam<double>(node, "lidar.min_range",
                                      std::string("lidarMinRange"),
                                      lidar.min_range, warnings);
  lidar.max_range = ReadParam<double>(node, "lidar.max_range",
                                      std::string("lidarMaxRange"),
                                      lidar.max_range, warnings);

  LoadGroundSegmentation(node, mapping.ground_segmentation, warnings);

  MappingImuConfig& imu = mapping.imu;
  imu.type = ReadParam<int>(node, "imu.type", std::string("imuType"), imu.type,
                            warnings);
  imu.acc_noise_std = ReadParam<double>(
      node, "imu.acc_noise_std", std::string("imuAccNoise"), imu.acc_noise_std,
      warnings);
  imu.gyro_noise_std = ReadParam<double>(
      node, "imu.gyro_noise_std", std::string("imuGyrNoise"),
      imu.gyro_noise_std, warnings);
  imu.acc_bias_rw_noise_std = ReadParam<double>(
      node, "imu.acc_bias_rw_noise_std", std::string("imuAccBiasN"),
      imu.acc_bias_rw_noise_std, warnings);
  imu.gyro_bias_rw_noise_std = ReadParam<double>(
      node, "imu.gyro_bias_rw_noise_std", std::string("imuGyrBiasN"),
      imu.gyro_bias_rw_noise_std, warnings);
  imu.gravity = ReadParam<double>(node, "imu.gravity",
                                  std::string("imuGravity"), imu.gravity,
                                  warnings);
  imu.rpy_weight = ReadParam<double>(node, "imu.rpy_weight",
                                     std::string("imuRPYWeight"),
                                     imu.rpy_weight, warnings);

  FeatureExtractionConfig& feature = mapping.feature_extraction;
  feature.edge_threshold = ReadParam<double>(
      node, "feature_extraction.edge_threshold", std::string("edgeThreshold"),
      feature.edge_threshold, warnings);
  feature.surface_threshold = ReadParam<double>(
      node, "feature_extraction.surface_threshold",
      std::string("surfThreshold"), feature.surface_threshold, warnings);
  feature.min_edge_features = ReadParam<int>(
      node, "feature_extraction.min_edge_features",
      std::string("edgeFeatureMinValidNum"), feature.min_edge_features,
      warnings);
  feature.min_surface_features = ReadParam<int>(
      node, "feature_extraction.min_surface_features",
      std::string("surfFeatureMinValidNum"), feature.min_surface_features,
      warnings);

  VoxelFilterConfig& voxel = mapping.voxel_filter;
  voxel.odometry_surface_leaf_size = ReadParam<double>(
      node, "voxel_filter.odometry_surface_leaf_size",
      std::string("odometrySurfLeafSize"), voxel.odometry_surface_leaf_size,
      warnings);
  voxel.mapping_corner_leaf_size = ReadParam<double>(
      node, "voxel_filter.mapping_corner_leaf_size",
      std::string("mappingCornerLeafSize"), voxel.mapping_corner_leaf_size,
      warnings);
  voxel.mapping_surface_leaf_size = ReadParam<double>(
      node, "voxel_filter.mapping_surface_leaf_size",
      std::string("mappingSurfLeafSize"), voxel.mapping_surface_leaf_size,
      warnings);

  DynamicFilterConfig& dynamic = mapping.dynamic_filter;
  dynamic.enable = ReadParam<bool>(
      node, "dynamic_filter.enable", std::string("dynamicKeyframeFilterEnable"),
      dynamic.enable, warnings);
  dynamic.surface_knn_max_distance = ReadParam<double>(
      node, "dynamic_filter.surface_knn_max_distance",
      std::string("dynamicSurfKnnMaxDist"), dynamic.surface_knn_max_distance,
      warnings);
  dynamic.surface_plane_max_distance = ReadParam<double>(
      node, "dynamic_filter.surface_plane_max_distance",
      std::string("dynamicSurfPlaneMaxDist"), dynamic.surface_plane_max_distance,
      warnings);

  StaticMapFilterConfig& static_filter = mapping.static_map_filter;
  static_filter.enable = ReadParam<bool>(
      node, "static_map_filter.enable", std::string("globalStaticMapFilterEnable"),
      static_filter.enable, warnings);
  static_filter.voxel_size = ReadParam<double>(
      node, "static_map_filter.voxel_size", std::string("globalStaticVoxelSize"),
      static_filter.voxel_size, warnings);
  static_filter.min_keyframes = ReadParam<int>(
      node, "static_map_filter.min_keyframes", std::string("globalStaticMinKF"),
      static_filter.min_keyframes, warnings);
  static_filter.filter_corners = ReadParam<bool>(
      node, "static_map_filter.filter_corners",
      std::string("globalStaticFilterCorners"), static_filter.filter_corners,
      warnings);

  MotionConstraintConfig& motion = mapping.motion_constraint;
  motion.z_tolerance = ReadParam<double>(
      node, "motion_constraint.z_tolerance", std::string("z_tollerance"),
      motion.z_tolerance, warnings);
  motion.rotation_tolerance = ReadParam<double>(
      node, "motion_constraint.rotation_tolerance",
      std::string("rotation_tollerance"), motion.rotation_tolerance, warnings);

  LocalMapConfig& local_map = mapping.local_map;
  local_map.keyframe_distance_threshold = ReadParam<double>(
      node, "local_map.keyframe_distance_threshold",
      std::string("surroundingkeyframeAddingDistThreshold"),
      local_map.keyframe_distance_threshold, warnings);
  local_map.keyframe_angle_threshold = ReadParam<double>(
      node, "local_map.keyframe_angle_threshold",
      std::string("surroundingkeyframeAddingAngleThreshold"),
      local_map.keyframe_angle_threshold, warnings);
  local_map.keyframe_density = ReadParam<double>(
      node, "local_map.keyframe_density", std::string("surroundingKeyframeDensity"),
      local_map.keyframe_density, warnings);
  local_map.search_radius = ReadParam<double>(
      node, "local_map.search_radius", std::string("surroundingKeyframeSearchRadius"),
      local_map.search_radius, warnings);

  MappingLoopClosureConfig& loop = mapping.loop_closure;
  loop.enable = ReadParam<bool>(node, "loop_closure.enable",
                                std::string("loopClosureEnableFlag"),
                                loop.enable, warnings);
  loop.frequency = ReadParam<double>(node, "loop_closure.frequency",
                                     std::string("loopClosureFrequency"),
                                     loop.frequency, warnings);
  loop.surrounding_keyframe_size = ReadParam<int>(
      node, "loop_closure.surrounding_keyframe_size",
      std::string("surroundingKeyframeSize"), loop.surrounding_keyframe_size,
      warnings);
  loop.history_search_radius = ReadParam<double>(
      node, "loop_closure.history_search_radius",
      std::string("historyKeyframeSearchRadius"), loop.history_search_radius,
      warnings);
  loop.history_search_time_difference = ReadParam<double>(
      node, "loop_closure.history_search_time_difference",
      std::string("historyKeyframeSearchTimeDiff"),
      loop.history_search_time_difference, warnings);
  loop.history_search_count = ReadParam<int>(
      node, "loop_closure.history_search_count",
      std::string("historyKeyframeSearchNum"), loop.history_search_count,
      warnings);
  loop.fitness_threshold = ReadParam<double>(
      node, "loop_closure.fitness_threshold",
      std::string("historyKeyframeFitnessScore"), loop.fitness_threshold,
      warnings);

  VisualizationConfig& visualization = mapping.visualization;
  visualization.global_map_search_radius = ReadParam<double>(
      node, "visualization.global_map_search_radius",
      std::string("globalMapVisualizationSearchRadius"),
      visualization.global_map_search_radius, warnings);
  visualization.global_map_pose_density = ReadParam<double>(
      node, "visualization.global_map_pose_density",
      std::string("globalMapVisualizationPoseDensity"),
      visualization.global_map_pose_density, warnings);
  visualization.global_map_leaf_size = ReadParam<double>(
      node, "visualization.global_map_leaf_size",
      std::string("globalMapVisualizationLeafSize"),
      visualization.global_map_leaf_size, warnings);

  RequirePositive(lidar.scan_lines, "lidar.scan_lines");
  RequirePositive(lidar.horizon_scan, "lidar.horizon_scan");
  RequirePositive(lidar.downsample_rate, "lidar.downsample_rate");
  RequirePositive(system.number_of_cores, "system.number_of_cores");
  RequirePositive(system.mapping_process_interval,
                  "system.mapping_process_interval");
  RequirePositive(imu.gravity, "imu.gravity");
  RequireNonNegative(map_save.tiled_map.tile_size_m,
                     "map_save.tiled_map.tile_size_m");
  RequirePositive(map_save.tiled_map.overview_resolution_m,
                  "map_save.tiled_map.overview_resolution_m");
  RequireNonEmpty(map_save.tiled_map.output_subdirectory,
                  "map_save.tiled_map.output_subdirectory");
}

void LoadLocalizationConfig(rclcpp::Node& node, SlamConfig& config,
                            LegacyWarningSet& warnings) {
  LocalizationConfig& localization = config.localization;

  LocalizationMapConfig& map = localization.localization;
  map.map_path = ReadParam<std::string>(node, "localization.map_path",
                                        std::nullopt, map.map_path, warnings);
  map.map_mode = ReadParam<std::string>(node, "localization.map_mode",
                                        std::nullopt, map.map_mode, warnings);
  map.tiled_map.active_radius_tiles = ReadParam<int>(
      node, "localization.tiled_map.active_radius_tiles", std::nullopt,
      map.tiled_map.active_radius_tiles, warnings);
  map.tiled_map.prefetch_radius_tiles = ReadParam<int>(
      node, "localization.tiled_map.prefetch_radius_tiles", std::nullopt,
      map.tiled_map.prefetch_radius_tiles, warnings);
  map.tiled_map.max_cached_tiles = ReadParam<int>(
      node, "localization.tiled_map.max_cached_tiles", std::nullopt,
      map.tiled_map.max_cached_tiles, warnings);
  map.tiled_map.max_cache_mib = ReadParam<int>(
      node, "localization.tiled_map.max_cache_mib", std::nullopt,
      map.tiled_map.max_cache_mib, warnings);
  map.tiled_map.load_timeout_ms = ReadParam<int>(
      node, "localization.tiled_map.load_timeout_ms", std::nullopt,
      map.tiled_map.load_timeout_ms, warnings);
  map.enable_ground_height_constraint = ReadParam<bool>(
      node, "localization.enable_ground_height_constraint", std::nullopt,
      map.enable_ground_height_constraint, warnings);
  map.base_link_ground_height = ReadParam<double>(
      node, "localization.base_link_ground_height", std::nullopt,
      map.base_link_ground_height, warnings);
  map.ground_search_radius = ReadParam<double>(
      node, "localization.ground_search_radius", std::nullopt,
      map.ground_search_radius, warnings);
  map.ground_z_percentile = ReadParam<double>(
      node, "localization.ground_z_percentile", std::nullopt,
      map.ground_z_percentile, warnings);
  map.ground_min_points = ReadParam<int>(
      node, "localization.ground_min_points", std::nullopt,
      map.ground_min_points, warnings);
  map.auto_relocalization = ReadParam<bool>(
      node, "localization.auto_relocalization", std::nullopt,
      map.auto_relocalization, warnings);
  map.sc_db_path = ReadParam<std::string>(node, "localization.sc_db_path",
                                          std::nullopt, map.sc_db_path,
                                          warnings);
  map.sc_top_n = ReadParam<int>(node, "localization.sc_top_n", std::nullopt,
                                map.sc_top_n, warnings);
  map.sc_fitness_thres = ReadParam<double>(
      node, "localization.sc_fitness_thres", std::nullopt,
      map.sc_fitness_thres, warnings);
  map.sc_consistency_frames = ReadParam<int>(
      node, "localization.sc_consistency_frames", std::nullopt,
      map.sc_consistency_frames, warnings);
  map.sc_consistency_trans_thres = ReadParam<double>(
      node, "localization.sc_consistency_trans_thres", std::nullopt,
      map.sc_consistency_trans_thres, warnings);
  map.sc_consistency_yaw_thres = ReadParam<double>(
      node, "localization.sc_consistency_yaw_thres", std::nullopt,
      map.sc_consistency_yaw_thres, warnings);
  map.sc_ringkey_max_dist = ReadParam<double>(
      node, "localization.sc_ringkey_max_dist", std::nullopt,
      map.sc_ringkey_max_dist, warnings);
  map.sc_accept_fitness = ReadParam<double>(
      node, "localization.sc_accept_fitness", std::nullopt,
      map.sc_accept_fitness, warnings);
  map.ground_map_path = ReadParam<std::string>(
      node, "localization.ground_map_path", std::nullopt, map.ground_map_path,
      warnings);

  LocalizationHealthConfig& health = localization.health;
  health.enable = ReadParam<bool>(node, "localization_health.enable",
                                  std::nullopt, health.enable, warnings);
  health.degrade_enter_frames = ReadParam<int>(
      node, "localization_health.degrade_enter_frames", std::nullopt,
      health.degrade_enter_frames, warnings);
  health.recover_frames = ReadParam<int>(
      node, "localization_health.recover_frames", std::nullopt,
      health.recover_frames, warnings);
  health.lost_enter_frames = ReadParam<int>(
      node, "localization_health.lost_enter_frames", std::nullopt,
      health.lost_enter_frames, warnings);
  health.degraded_fitness_thres = ReadParam<double>(
      node, "localization_health.degraded_fitness_thres", std::nullopt,
      health.degraded_fitness_thres, warnings);
  health.lost_fitness_thres = ReadParam<double>(
      node, "localization_health.lost_fitness_thres", std::nullopt,
      health.lost_fitness_thres, warnings);
  health.nominal_lidar_rate_hz = ReadParam<double>(
      node, "localization_health.nominal_lidar_rate_hz", std::nullopt,
      health.nominal_lidar_rate_hz, warnings);
  health.nominal_imu_rate_hz = ReadParam<double>(
      node, "localization_health.nominal_imu_rate_hz", std::nullopt,
      health.nominal_imu_rate_hz, warnings);
  health.min_rate_ratio = ReadParam<double>(
      node, "localization_health.min_rate_ratio", std::nullopt,
      health.min_rate_ratio, warnings);
  health.lidar_stall_timeout = ReadParam<double>(
      node, "localization_health.lidar_stall_timeout", std::nullopt,
      health.lidar_stall_timeout, warnings);
  health.imu_stall_timeout = ReadParam<double>(
      node, "localization_health.imu_stall_timeout", std::nullopt,
      health.imu_stall_timeout, warnings);
  health.result_timeout = ReadParam<double>(
      node, "localization_health.result_timeout", std::nullopt,
      health.result_timeout, warnings);
  health.base_covariance = ReadParam<double>(
      node, "localization_health.base_covariance", std::nullopt,
      health.base_covariance, warnings);
  health.max_covariance_scale = ReadParam<double>(
      node, "localization_health.max_covariance_scale", std::nullopt,
      health.max_covariance_scale, warnings);

  LocalizationTfConfig& tf = localization.tf;
  tf.publish_rate_hz = ReadParam<double>(
      node, "localization_tf.publish_rate_hz", std::nullopt,
      tf.publish_rate_hz, warnings);
  tf.max_extrapolation_seconds = ReadParam<double>(
      node, "localization_tf.max_extrapolation_seconds", std::nullopt,
      tf.max_extrapolation_seconds, warnings);
  tf.enable_imu_extrapolation = ReadParam<bool>(
      node, "localization_tf.enable_imu_extrapolation", std::nullopt,
      tf.enable_imu_extrapolation, warnings);

  RuntimeConfig& runtime = localization.runtime;
  runtime.enable_realtime_scheduling = ReadParam<bool>(
      node, "runtime.enable_realtime_scheduling", std::nullopt,
      runtime.enable_realtime_scheduling, warnings);
  runtime.preprocessing_fifo_priority = ReadParam<int>(
      node, "runtime.preprocessing_fifo_priority", std::nullopt,
      runtime.preprocessing_fifo_priority, warnings);
  runtime.localization_fifo_priority = ReadParam<int>(
      node, "runtime.localization_fifo_priority", std::nullopt,
      runtime.localization_fifo_priority, warnings);
  runtime.preprocessing_cpu_core = ReadParam<int>(
      node, "runtime.preprocessing_cpu_core", std::nullopt,
      runtime.preprocessing_cpu_core, warnings);
  runtime.localization_cpu_core = ReadParam<int>(
      node, "runtime.localization_cpu_core", std::nullopt,
      runtime.localization_cpu_core, warnings);
  runtime.frame_processing_timeout_ms = ReadParam<int>(
      node, "runtime.frame_processing_timeout_ms", std::nullopt,
      runtime.frame_processing_timeout_ms, warnings);

  LocalizationLidarConfig& lidar = localization.lidar;
  lidar.sensor_type = ReadParam<std::string>(
      node, "lidar.sensor_type", std::string("lidar.lidar_sensor_type"),
      lidar.sensor_type, warnings);
  lidar.point_jump_span = ReadParam<int>(
      node, "lidar.point_jump_span", std::string("lidar.lidar_point_jump_span"),
      lidar.point_jump_span, warnings);
  lidar.scan_lines = ReadParam<int>(node, "lidar.scan_lines",
                                    std::string("lidar.lidar_scan"),
                                    lidar.scan_lines, warnings);
  lidar.lower_angle_deg = ReadParam<double>(
      node, "lidar.lower_angle_deg", std::string("lidar.lidar_lower_angle"),
      lidar.lower_angle_deg, warnings);
  lidar.horizon_scan = ReadParam<int>(
      node, "lidar.horizon_scan", std::string("lidar.lidar_horizon_scan"),
      lidar.horizon_scan, warnings);
  lidar.vertical_resolution_deg = ReadParam<double>(
      node, "lidar.vertical_resolution_deg",
      std::string("lidar.lidar_vertical_resolution"),
      lidar.vertical_resolution_deg, warnings);
  lidar.min_range = ReadParam<double>(
      node, "lidar.min_range", std::string("lidar.lidar_use_min_distance"),
      lidar.min_range, warnings);
  lidar.max_range = ReadParam<double>(
      node, "lidar.max_range", std::string("lidar.lidar_use_max_distance"),
      lidar.max_range, warnings);
  lidar.point_time_scale = ReadParam<double>(
      node, "lidar.point_time_scale", std::string("lidar.lidar_point_time_scale"),
      lidar.point_time_scale, warnings);
  lidar.rotation_noise_std = ReadParam<double>(
      node, "lidar.rotation_noise_std",
      std::string("lidar.lidar_rotation_noise_std"), lidar.rotation_noise_std,
      warnings);
  lidar.position_noise_std = ReadParam<double>(
      node, "lidar.position_noise_std",
      std::string("lidar.lidar_position_noise_std"), lidar.position_noise_std,
      warnings);

  LocalizationImuConfig& imu = localization.imu;
  imu.has_orientation = ReadParam<bool>(node, "imu.has_orientation",
                                        std::nullopt, imu.has_orientation,
                                        warnings);
  imu.init_acc_bias = ReadParam<double>(node, "imu.init_acc_bias", std::nullopt,
                                        imu.init_acc_bias, warnings);
  imu.init_gyro_bias = ReadParam<double>(
      node, "imu.init_gyro_bias", std::nullopt, imu.init_gyro_bias, warnings);
  imu.acc_noise_std = ReadParam<double>(node, "imu.acc_noise_std",
                                        std::nullopt, imu.acc_noise_std,
                                        warnings);
  imu.gyro_noise_std = ReadParam<double>(node, "imu.gyro_noise_std", std::nullopt,
                                         imu.gyro_noise_std, warnings);
  imu.acc_bias_rw_noise_std = ReadParam<double>(
      node, "imu.acc_bias_rw_noise_std", std::string("imu.acc_rw_noise_std"),
      imu.acc_bias_rw_noise_std, warnings);
  imu.gyro_bias_rw_noise_std = ReadParam<double>(
      node, "imu.gyro_bias_rw_noise_std", std::string("imu.gyro_rw_noise_std"),
      imu.gyro_bias_rw_noise_std, warnings);
  imu.data_searcher_buffer_size = ReadParam<int>(
      node, "imu.data_searcher_buffer_size", std::nullopt,
      imu.data_searcher_buffer_size, warnings);
  imu.gravity = ReadParam<double>(node, "imu.gravity", std::string("gravity"),
                                  imu.gravity, warnings);

  LocalizationFrontendConfig& frontend = localization.frontend;
  frontend.registration_and_searcher_mode = ReadParam<std::string>(
      node, "frontend.registration_and_searcher_mode", std::nullopt,
      frontend.registration_and_searcher_mode, warnings);
  frontend.feature.corner_thres = ReadParam<double>(
      node, "frontend.feature.corner_thres", std::nullopt,
      frontend.feature.corner_thres, warnings);
  frontend.feature.planar_thres = ReadParam<double>(
      node, "frontend.feature.planar_thres", std::nullopt,
      frontend.feature.planar_thres, warnings);
  frontend.feature.planar_voxel_filter_size = ReadParam<double>(
      node, "frontend.feature.planar_voxel_filter_size", std::nullopt,
      frontend.feature.planar_voxel_filter_size, warnings);
  frontend.feature.corner_voxel_filter_size = ReadParam<double>(
      node, "frontend.feature.corner_voxel_filter_size", std::nullopt,
      frontend.feature.corner_voxel_filter_size, warnings);

  LocalizationRegistrationConfig& registration = frontend.registration;
  registration.line_ratio_thres = ReadParam<double>(
      node, "frontend.registration.line_ratio_thres", std::nullopt,
      registration.line_ratio_thres, warnings);
  registration.point_search_thres = ReadParam<double>(
      node, "frontend.registration.point_search_thres", std::nullopt,
      registration.point_search_thres, warnings);
  registration.point_to_planar_thres = ReadParam<double>(
      node, "frontend.registration.point_to_planar_thres", std::nullopt,
      registration.point_to_planar_thres, warnings);
  registration.local_planar_map_size = ReadParam<int>(
      node, "frontend.registration.local_planar_map_size", std::nullopt,
      registration.local_planar_map_size, warnings);
  registration.local_corner_map_size = ReadParam<int>(
      node, "frontend.registration.local_corner_map_size", std::nullopt,
      registration.local_corner_map_size, warnings);
  registration.keyframe_delta_distance = ReadParam<double>(
      node, "frontend.registration.keyframe_delta_distance", std::nullopt,
      registration.keyframe_delta_distance, warnings);
  registration.keyframe_delta_rotation = ReadParam<double>(
      node, "frontend.registration.keyframe_delta_rotation", std::nullopt,
      registration.keyframe_delta_rotation, warnings);
  registration.rotation_converge_thres = ReadParam<double>(
      node, "frontend.registration.rotation_converge_thres", std::nullopt,
      registration.rotation_converge_thres, warnings);
  registration.position_converge_thres = ReadParam<double>(
      node, "frontend.registration.position_converge_thres", std::nullopt,
      registration.position_converge_thres, warnings);
  registration.local_corner_voxel_filter_size = ReadParam<double>(
      node, "frontend.registration.local_corner_voxel_filter_size",
      std::nullopt, registration.local_corner_voxel_filter_size, warnings);
  registration.local_planar_voxel_filter_size = ReadParam<double>(
      node, "frontend.registration.local_planar_voxel_filter_size",
      std::nullopt, registration.local_planar_voxel_filter_size, warnings);
  registration.local_map_size = ReadParam<int>(
      node, "frontend.registration.local_map_size", std::nullopt,
      registration.local_map_size, warnings);
  registration.local_map_cloud_filter_size = ReadParam<double>(
      node, "frontend.registration.local_map_cloud_filter_size", std::nullopt,
      registration.local_map_cloud_filter_size, warnings);
  registration.source_cloud_filter_size = ReadParam<double>(
      node, "frontend.registration.source_cloud_filter_size", std::nullopt,
      registration.source_cloud_filter_size, warnings);
  registration.optimization_iter_num = ReadParam<int>(
      node, "frontend.registration.optimization_iter_num", std::nullopt,
      registration.optimization_iter_num, warnings);
  registration.ndt_voxel_size = ReadParam<double>(
      node, "frontend.registration.ndt_voxel_size", std::nullopt,
      registration.ndt_voxel_size, warnings);
  registration.ndt_outlier_threshold = ReadParam<double>(
      node, "frontend.registration.ndt_outlier_threshold", std::nullopt,
      registration.ndt_outlier_threshold, warnings);
  registration.ndt_min_points_in_voxel = ReadParam<int>(
      node, "frontend.registration.ndt_min_points_in_voxel", std::nullopt,
      registration.ndt_min_points_in_voxel, warnings);
  registration.ndt_max_points_in_voxel = ReadParam<int>(
      node, "frontend.registration.ndt_max_points_in_voxel", std::nullopt,
      registration.ndt_max_points_in_voxel, warnings);
  registration.ndt_min_effective_pts = ReadParam<int>(
      node, "frontend.registration.ndt_min_effective_pts", std::nullopt,
      registration.ndt_min_effective_pts, warnings);
  registration.ndt_capacity = ReadParam<int>(
      node, "frontend.registration.ndt_capacity", std::nullopt,
      registration.ndt_capacity, warnings);

  frontend.fusion_method = ReadParam<std::string>(
      node, "frontend.fusion_method", std::nullopt, frontend.fusion_method,
      warnings);
  frontend.fusion_opti_iters = ReadParam<int>(
      node, "frontend.fusion_opti_iters", std::nullopt,
      frontend.fusion_opti_iters, warnings);

  LocalizationSystemConfig& system = localization.system;
  system.keyframe_delta_distance = ReadParam<double>(
      node, "system.keyframe_delta_distance", std::nullopt,
      system.keyframe_delta_distance, warnings);
  system.keyframe_delta_rotation = ReadParam<double>(
      node, "system.keyframe_delta_rotation", std::nullopt,
      system.keyframe_delta_rotation, warnings);
  system.tile_map_grid_size = ReadParam<double>(
      node, "system.tile_map_grid_size", std::nullopt,
      system.tile_map_grid_size, warnings);

  LocalizationLoopClosureConfig& loop = localization.loop_closure;
  loop.enable = ReadParam<bool>(node, "loop_closure.enable",
                                std::string("system.enable_loopclosure"),
                                loop.enable, warnings);
  loop.registration_converge_threshold = ReadParam<double>(
      node, "loop_closure.registration_converge_threshold",
      std::string("loopclosure.registration_converge_threshold"),
      loop.registration_converge_threshold, warnings);
  loop.skip_near_loopclosure_threshold = ReadParam<int>(
      node, "loop_closure.skip_near_loopclosure_threshold",
      std::string("loopclosure.skip_near_loopclosure_threshold"),
      loop.skip_near_loopclosure_threshold, warnings);
  loop.skip_near_keyframe_threshold = ReadParam<int>(
      node, "loop_closure.skip_near_keyframe_threshold",
      std::string("loopclosure.skip_near_keyframe_threshold"),
      loop.skip_near_keyframe_threshold, warnings);
  loop.candidate_local_map_left_range = ReadParam<int>(
      node, "loop_closure.candidate_local_map_left_range",
      std::string("loopclosure.candidate_local_map_left_range"),
      loop.candidate_local_map_left_range, warnings);
  loop.candidate_local_map_right_range = ReadParam<int>(
      node, "loop_closure.candidate_local_map_right_range",
      std::string("loopclosure.candidate_local_map_right_range"),
      loop.candidate_local_map_right_range, warnings);
  loop.loopclosure_local_map_left_range = ReadParam<int>(
      node, "loop_closure.loopclosure_local_map_left_range",
      std::string("loopclosure.loopclosure_local_map_left_range"),
      loop.loopclosure_local_map_left_range, warnings);
  loop.near_neighbor_distance_threshold = ReadParam<double>(
      node, "loop_closure.near_neighbor_distance_threshold",
      std::string("loopclosure.near_neighbor_distance_threshold"),
      loop.near_neighbor_distance_threshold, warnings);

  LoadGroundSegmentation(node, localization.ground_segmentation, warnings);

  // --- validation ---
  if (map.map_mode != "auto" && map.map_mode != "global" &&
      map.map_mode != "tiled") {
    Reject("localization.map_mode", "must be auto, global, or tiled");
  }
  RequireNonNegative(map.tiled_map.active_radius_tiles,
                     "localization.tiled_map.active_radius_tiles");
  RequirePositive(map.tiled_map.max_cached_tiles,
                  "localization.tiled_map.max_cached_tiles");
  RequirePositive(map.tiled_map.max_cache_mib,
                  "localization.tiled_map.max_cache_mib");
  RequirePositive(map.tiled_map.load_timeout_ms,
                  "localization.tiled_map.load_timeout_ms");

  const std::uint64_t active_width =
      2U * static_cast<std::uint64_t>(map.tiled_map.active_radius_tiles) + 1U;
  const std::uint64_t minimum_cached_tiles = active_width * active_width;
  if (map.tiled_map.prefetch_radius_tiles <
          map.tiled_map.active_radius_tiles ||
      static_cast<std::uint64_t>(map.tiled_map.max_cached_tiles) <
          minimum_cached_tiles) {
    Reject("localization.tiled_map",
           "prefetch radius must cover the active radius and max_cached_tiles "
           "must be at least the active window size");
  }

  RequirePositive(health.nominal_lidar_rate_hz,
                  "localization_health.nominal_lidar_rate_hz");
  RequirePositive(health.nominal_imu_rate_hz,
                  "localization_health.nominal_imu_rate_hz");
  RequirePositive(health.lidar_stall_timeout,
                  "localization_health.lidar_stall_timeout");
  RequirePositive(health.imu_stall_timeout,
                  "localization_health.imu_stall_timeout");
  RequirePositive(health.result_timeout,
                  "localization_health.result_timeout");
  RequirePositive(health.degrade_enter_frames,
                  "localization_health.degrade_enter_frames");
  RequirePositive(health.recover_frames,
                  "localization_health.recover_frames");
  RequirePositive(health.lost_enter_frames,
                  "localization_health.lost_enter_frames");
  RequirePositive(imu.gravity, "imu.gravity");
  RequirePositive(lidar.scan_lines, "lidar.scan_lines");
  RequirePositive(lidar.horizon_scan, "lidar.horizon_scan");
  RequirePositive(lidar.point_jump_span, "lidar.point_jump_span");
  RequirePositive(runtime.frame_processing_timeout_ms,
                  "runtime.frame_processing_timeout_ms");

  RequirePositive(tf.publish_rate_hz, "localization_tf.publish_rate_hz");
  RequirePositive(tf.max_extrapolation_seconds,
                  "localization_tf.max_extrapolation_seconds");
  if (tf.max_extrapolation_seconds > health.result_timeout) {
    Reject("localization_tf.max_extrapolation_seconds",
           "must not exceed localization_health.result_timeout");
  }
}

}  // namespace

SlamConfig LoadSlamConfig(rclcpp::Node& node, SlamMode mode) {
  SlamConfig config;
  config.mode = mode;
  LegacyWarningSet warnings;

  LoadFrames(node, mode, config, warnings);
  LoadCalibration(node, config, warnings);
  LoadCommonTopics(node, mode, config, warnings);

  if (mode == SlamMode::kMapping) {
    LoadMappingConfig(node, config, warnings);
  } else {
    LoadLocalizationConfig(node, config, warnings);
  }

  return config;
}
