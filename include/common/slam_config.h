#ifndef LIO_SAM_HESAI_COMMON_SLAM_CONFIG_H
#define LIO_SAM_HESAI_COMMON_SLAM_CONFIG_H

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "common/parameter_alias.h"
#include "common/rigid_transform.h"
#include "common/ros_pose_adapter.h"

enum class SlamMode { kMapping, kLocalization };

// ---------------------------------------------------------------------------
// Shared groups
// ---------------------------------------------------------------------------

struct SensorTopics {
  std::string lidar{"points"};
  std::string imu{"imu/data"};
  std::string gps{"lio_sam/odometry/gps"};
};

// Package-internal protocol between nodes. These are never user-facing.
struct PipelineTopics {
  std::string cloud_info{"lio_sam/deskew/cloud_info"};
  std::string feature_cloud_info{"lio_sam/feature/cloud_info"};
  std::string mapping_odometry{"lio_sam/mapping/odometry"};
  std::string mapping_odometry_incremental{
      "lio_sam/mapping/odometry_incremental"};
  std::string imu_odometry_incremental{"odometry/imu_incremental"};
  std::string deskewed_cloud{"lio_sam/deskew/cloud_deskewed"};
  std::string ground_cloud{"lio_sam/deskew/ground_cloud_global"};
  std::string corner_cloud{"lio_sam/feature/cloud_corner"};
  std::string surface_cloud{"lio_sam/feature/cloud_surface"};
  std::string loop_closure_hint{"lio_loop/loop_closure_detection"};
};

// External control inputs. An empty string means the mode does not accept that
// input and no subscriber may be created for it.
struct CommandTopics {
  std::string initial_pose;
};

// Downstream-facing results. An empty string means the mode does not publish
// that output.
struct OutputTopics {
  std::string odometry;
  std::string path;
  std::string global_map;
  std::string local_map;
  std::string registered_cloud;
  std::string registered_cloud_raw;
  std::string ground_map;
  std::string current_lidar;
  std::string trajectory;
  std::string imu_path;
  std::string diagnostics{"/diagnostics"};
  std::string loop_history;
  std::string loop_constraints;
  std::string non_ground_cloud;
};

// Canonical calibration semantics are imu_T_lidar. The mapping-specific
// rotation/lever-arm/rpy values are derived from it and never configured
// independently.
struct CalibrationConfig {
  ImuTLidar imu_T_lidar{ImuTLidar::Identity()};
  LidarTImu lidar_T_imu{LidarTImu::Identity()};
  Eigen::Matrix3d mapping_imu_to_lidar_rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d mapping_imu_to_lidar_lever_arm{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rpy_rotation{Eigen::Matrix3d::Identity()};
};

// ---------------------------------------------------------------------------
// Mapping configuration
// ---------------------------------------------------------------------------

struct GpsConfig {
  bool use_imu_heading_initialization{false};
  bool use_elevation{false};
  double covariance_threshold{2.0};
  double pose_covariance_threshold{25.0};
};

struct TiledMapSaveConfig {
  double tile_size_m{100.0};
  std::string output_subdirectory{"tiled"};
  double overview_resolution_m{5.0};
};

struct MapSaveConfig {
  bool enabled{false};
  bool save_tiled_map{false};
  TiledMapSaveConfig tiled_map;
  bool generate_scan_context_db{true};
  std::string directory{"/Downloads/LOAM/"};
  double resolution{0.0};
  double ground_resolution{0.0};
  bool save_ground_map{false};
  std::string ground_directory{"/Downloads/LOAM_ground/"};
  std::string ground_global_directory{"/Downloads/LOAM_ground_global/"};
  bool ground_final_use_odometry{true};
  double ground_global_leaf_size{0.2};
  bool align_ground_to_zero{false};
  double ground_z_offset{0.0};
};

struct GroundGridPatchConfig {
  bool enable{false};
  double resolution{0.10};
  double search_radius{0.40};
  double max_slope_deg{25.0};
  double max_height_range{0.12};
  double max_roughness{0.05};
  double max_prediction_margin{0.08};
  double min_inlier_ratio{0.70};
  int min_neighbor_cells{6};
  int min_support_quadrants{3};
};

struct GroundSegmentationConfig {
  bool only_mode{false};
  double leaf_size{0.1};
  int scan_start_index{0};
  int scan_end_index{-1};
  double angle_threshold_deg{10.0};
  GroundGridPatchConfig grid_patch;
  double plane_distance{0.12};
  double plane_max_angle_deg{35.0};
  int plane_min_points{80};
  bool flatten_z{false};
  double flatten_value{0.0};
};

struct MappingSystemConfig {
  bool mapping_mode{true};
  int number_of_cores{4};
  double mapping_process_interval{0.15};
};

struct MappingLidarConfig {
  std::string sensor_type{"ouster"};
  int scan_lines{64};
  int horizon_scan{512};
  int downsample_rate{1};
  double min_range{5.5};
  double max_range{1000.0};
};

struct MappingImuConfig {
  int type{0};
  double acc_noise_std{9e-4};
  double gyro_noise_std{1.6e-4};
  double acc_bias_rw_noise_std{5e-4};
  double gyro_bias_rw_noise_std{7e-5};
  double gravity{9.80511};
  double rpy_weight{0.01};
};

struct FeatureExtractionConfig {
  double edge_threshold{1.0};
  double surface_threshold{0.1};
  int min_edge_features{10};
  int min_surface_features{100};
};

struct VoxelFilterConfig {
  double odometry_surface_leaf_size{0.4};
  double mapping_corner_leaf_size{0.2};
  double mapping_surface_leaf_size{0.4};
};

struct DynamicFilterConfig {
  bool enable{true};
  double surface_knn_max_distance{1.0};
  double surface_plane_max_distance{0.3};
};

struct StaticMapFilterConfig {
  bool enable{false};
  double voxel_size{0.4};
  int min_keyframes{3};
  bool filter_corners{true};
};

struct MotionConstraintConfig {
  double z_tolerance{1000.0};
  double rotation_tolerance{1000.0};
};

struct LocalMapConfig {
  double keyframe_distance_threshold{1.0};
  double keyframe_angle_threshold{0.2};
  double keyframe_density{2.0};
  double search_radius{50.0};
};

struct MappingLoopClosureConfig {
  bool enable{true};
  double frequency{1.0};
  int surrounding_keyframe_size{50};
  double history_search_radius{15.0};
  double history_search_time_difference{30.0};
  int history_search_count{25};
  double fitness_threshold{0.3};
};

struct VisualizationConfig {
  double global_map_search_radius{1000.0};
  double global_map_pose_density{10.0};
  double global_map_leaf_size{1.0};
};

struct MappingConfig {
  GpsConfig gps;
  MapSaveConfig map_save;
  MappingSystemConfig system;
  MappingLidarConfig lidar;
  GroundSegmentationConfig ground_segmentation;
  MappingImuConfig imu;
  FeatureExtractionConfig feature_extraction;
  VoxelFilterConfig voxel_filter;
  DynamicFilterConfig dynamic_filter;
  StaticMapFilterConfig static_map_filter;
  MotionConstraintConfig motion_constraint;
  LocalMapConfig local_map;
  MappingLoopClosureConfig loop_closure;
  VisualizationConfig visualization;
};

// ---------------------------------------------------------------------------
// Localization configuration
// ---------------------------------------------------------------------------

struct LocalizationTiledMapConfig {
  int active_radius_tiles{1};
  int prefetch_radius_tiles{2};
  int max_cached_tiles{25};
  int max_cache_mib{1024};
  int load_timeout_ms{1000};
};

// The `localization.*` group's own leaves.
struct LocalizationMapConfig {
  std::string map_path;
  std::string map_mode{"auto"};
  LocalizationTiledMapConfig tiled_map;
  bool enable_ground_height_constraint{false};
  double base_link_ground_height{0.275};
  double ground_search_radius{5.0};
  double ground_z_percentile{0.20};
  int ground_min_points{100};
  bool auto_relocalization{false};
  std::string sc_db_path;
  int sc_top_n{8};
  double sc_fitness_thres{1.0};
  int sc_consistency_frames{3};
  double sc_consistency_trans_thres{0.5};
  double sc_consistency_yaw_thres{0.1};
  double sc_ringkey_max_dist{5.0};
  double sc_accept_fitness{0.3};
  std::string ground_map_path;
};

struct LocalizationHealthConfig {
  bool enable{true};
  int degrade_enter_frames{3};
  int recover_frames{5};
  int lost_enter_frames{10};
  double degraded_fitness_thres{0.5};
  double lost_fitness_thres{1.0};
  double nominal_lidar_rate_hz{10.0};
  double nominal_imu_rate_hz{100.0};
  double min_rate_ratio{0.5};
  double lidar_stall_timeout{0.5};
  double imu_stall_timeout{0.1};
  double result_timeout{1.0};
  double base_covariance{0.05};
  double max_covariance_scale{100.0};
};

struct LocalizationTfConfig {
  double publish_rate_hz{20.0};
  double max_extrapolation_seconds{0.5};
  bool enable_imu_extrapolation{true};
};

struct RuntimeConfig {
  bool enable_realtime_scheduling{true};
  int preprocessing_fifo_priority{70};
  int localization_fifo_priority{80};
  int preprocessing_cpu_core{4};
  int localization_cpu_core{5};
  int frame_processing_timeout_ms{500};
};

struct LocalizationLidarConfig {
  std::string sensor_type;
  int point_jump_span{1};
  int scan_lines{64};
  double lower_angle_deg{0.0};
  int horizon_scan{512};
  double vertical_resolution_deg{0.0};
  double min_range{0.0};
  double max_range{0.0};
  double point_time_scale{1.0};
  double rotation_noise_std{0.0};
  double position_noise_std{0.0};
};

struct LocalizationImuConfig {
  bool has_orientation{false};
  double init_acc_bias{0.0};
  double init_gyro_bias{0.0};
  double acc_noise_std{0.0};
  double gyro_noise_std{0.0};
  double acc_bias_rw_noise_std{0.0};
  double gyro_bias_rw_noise_std{0.0};
  int data_searcher_buffer_size{0};
  double gravity{9.80511};
};

struct LocalizationFeatureConfig {
  double corner_thres{0.0};
  double planar_thres{0.0};
  double planar_voxel_filter_size{0.0};
  double corner_voxel_filter_size{0.0};
};

struct LocalizationRegistrationConfig {
  double line_ratio_thres{0.0};
  double point_search_thres{0.0};
  double point_to_planar_thres{0.0};
  int local_planar_map_size{0};
  int local_corner_map_size{0};
  double keyframe_delta_distance{0.0};
  double keyframe_delta_rotation{0.0};
  double rotation_converge_thres{0.0};
  double position_converge_thres{0.0};
  double local_corner_voxel_filter_size{0.0};
  double local_planar_voxel_filter_size{0.0};
  int local_map_size{0};
  double local_map_cloud_filter_size{0.0};
  double source_cloud_filter_size{0.0};
  int optimization_iter_num{0};
  double ndt_voxel_size{0.0};
  double ndt_outlier_threshold{0.0};
  int ndt_min_points_in_voxel{0};
  int ndt_max_points_in_voxel{0};
  int ndt_min_effective_pts{0};
  int ndt_capacity{0};
};

struct LocalizationFrontendConfig {
  std::string registration_and_searcher_mode;
  LocalizationFeatureConfig feature;
  LocalizationRegistrationConfig registration;
  std::string fusion_method;
  int fusion_opti_iters{0};
};

struct LocalizationSystemConfig {
  double keyframe_delta_distance{0.0};
  double keyframe_delta_rotation{0.0};
  double tile_map_grid_size{0.0};
};

struct LocalizationLoopClosureConfig {
  bool enable{false};
  double registration_converge_threshold{0.0};
  int skip_near_loopclosure_threshold{0};
  int skip_near_keyframe_threshold{0};
  int candidate_local_map_left_range{0};
  int candidate_local_map_right_range{0};
  int loopclosure_local_map_left_range{0};
  double near_neighbor_distance_threshold{0.0};
};

struct LocalizationConfig {
  LocalizationMapConfig localization;
  LocalizationHealthConfig health;
  LocalizationTfConfig tf;
  RuntimeConfig runtime;
  LocalizationLidarConfig lidar;
  LocalizationImuConfig imu;
  LocalizationFrontendConfig frontend;
  LocalizationSystemConfig system;
  LocalizationLoopClosureConfig loop_closure;
  GroundSegmentationConfig ground_segmentation;
};

// ---------------------------------------------------------------------------
// Aggregate
// ---------------------------------------------------------------------------

struct SlamConfig {
  SlamMode mode{SlamMode::kMapping};
  FrameIds frames;
  SensorTopics sensors;
  PipelineTopics pipeline_topics;
  CommandTopics command_topics;
  OutputTopics outputs;
  CalibrationConfig calibration;
  MappingConfig mapping;
  LocalizationConfig localization;
};

// Declares, reads, resolves aliases for, and validates every parameter the
// given mode needs. Throws ParameterAliasConflict on conflicting dual keys and
// std::invalid_argument on invalid values.
SlamConfig LoadSlamConfig(rclcpp::Node& node, SlamMode mode);

// Legacy keys already reported by this loader, so each deprecation warning is
// emitted exactly once per node.
using LegacyWarningSet = std::set<std::string>;

namespace slam_config_detail {

// Keep warning state on node lifetime, not loader-call lifetime. Weak owners
// avoid retaining nodes (or confusing a newly allocated node with an old one).
inline bool FirstLegacyWarning(rclcpp::Node& node, const std::string& key) {
  static std::mutex mutex;
  using Owner = std::weak_ptr<rclcpp::node_interfaces::NodeBaseInterface>;
  static std::map<Owner, LegacyWarningSet, std::owner_less<Owner>> per_node;
  std::lock_guard<std::mutex> lock(mutex);
  for (auto it = per_node.begin(); it != per_node.end();) {
    if (it->first.expired()) it = per_node.erase(it);
    else ++it;
  }
  return per_node[node.get_node_base_interface()].insert(key).second;
}

// A parameter counts as explicitly set only when it appears in the node's
// override map. Reading the declared value alone cannot tell a user-supplied
// value apart from the default. Declaring is idempotent so callers may read the
// same key more than once (a legacy key can also be a derived value's source).
template <typename T>
std::optional<T> ReadOverride(rclcpp::Node& node, const std::string& name,
                              const T& default_value) {
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, default_value);
  }
  const auto overrides =
      node.get_node_parameters_interface()->get_parameter_overrides();
  if (overrides.count(name) == 0U) {
    return std::nullopt;
  }
  return node.get_parameter(name).get_value<T>();
}

// Applies the alias precedence rules and emits at most one deprecation warning
// per legacy key. `legacy_key` names the key to report; `legacy_value` is the
// resolved legacy value, which a caller may derive rather than read directly.
template <typename T>
T ResolveAndWarn(rclcpp::Node& node, const std::string& canonical,
                 const std::optional<std::string>& legacy_key,
                 const std::optional<T>& canonical_value,
                 const std::optional<T>& legacy_value,
                 const T& default_value, LegacyWarningSet& warnings) {
  ParameterAliasResult<T> result;
  try {
    result =
        ResolveParameterAlias<T>(canonical_value, legacy_value, default_value);
  } catch (const ParameterAliasConflict&) {
    const std::string canonical_text =
        node.get_parameter(canonical).value_to_string();
    std::string legacy_text = "(unset)";
    if (legacy_key.has_value() && node.has_parameter(*legacy_key)) {
      legacy_text = node.get_parameter(*legacy_key).value_to_string();
    }
    throw ParameterAliasConflict(
        "conflicting parameter values: '" + canonical + "' = " +
        canonical_text + " but deprecated '" +
        (legacy_key.has_value() ? *legacy_key : std::string("<legacy>")) +
        "' = " + legacy_text + "; set only one of them");
  }

  if (result.used_legacy && legacy_key.has_value() &&
      warnings.insert(*legacy_key).second && FirstLegacyWarning(node, *legacy_key)) {
    RCLCPP_WARN(node.get_logger(),
                "Parameter '%s' is deprecated; use '%s' instead. The legacy "
                "alias remains supported for the P2.2 release.",
                legacy_key->c_str(), canonical.c_str());
  }
  return result.value;
}

}  // namespace slam_config_detail

// All aliases are peers: reject legacy-vs-legacy conflicts as well as conflicts
// with the canonical override, before emitting any deprecation warnings.
template <typename T>
T DeclareRead(rclcpp::Node& node, const std::string& canonical,
              const std::vector<std::string>& aliases, const T& default_value,
              LegacyWarningSet& warnings) {
  const auto canonical_value =
      slam_config_detail::ReadOverride<T>(node, canonical, default_value);
  std::optional<T> resolved = canonical_value;
  std::string resolved_key = canonical;
  std::vector<std::pair<std::string, std::optional<T>>> values;
  for (const auto& alias : aliases) {
    if (alias.empty() || alias == canonical) continue;
    const auto value = slam_config_detail::ReadOverride<T>(node, alias, default_value);
    if (resolved && value && !(*resolved == *value)) {
      throw ParameterAliasConflict("conflicting parameter values for '" + canonical +
          "': '" + resolved_key + "' and deprecated '" + alias + "'; set only one of them");
    }
    if (!resolved && value) { resolved = value; resolved_key = alias; }
    values.emplace_back(alias, value);
  }
  for (const auto& value : values) {
    slam_config_detail::ResolveAndWarn<T>(node, canonical, value.first,
        canonical_value, value.second, default_value, warnings);
  }
  return resolved.value_or(default_value);
}

// Reads one canonical/legacy key pair. Canonical wins; legacy-only works and
// warns once; equal dual values warn once; unequal dual values throw
// ParameterAliasConflict naming both keys.
template <typename T>
T DeclareRead(rclcpp::Node& node, const std::string& canonical,
              const std::optional<std::string>& legacy, const T& default_value,
              LegacyWarningSet& warnings) {
  return DeclareRead<T>(node, canonical,
      legacy ? std::vector<std::string>{*legacy} : std::vector<std::string>{},
      default_value, warnings);
}

#endif  // LIO_SAM_HESAI_COMMON_SLAM_CONFIG_H
