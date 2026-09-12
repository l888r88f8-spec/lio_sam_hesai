#include <gtest/gtest.h>

#include <array>
#include <cctype>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{

const std::array<std::string, 8> kParameterFiles = {
  "include/lio_sam/utility.hpp",
  "src/imageProjection.cpp",
  "src/slam/system.cpp",
  "config/mapping.yaml",
  "config/relocalization.yaml",
  "launch/mapping.launch.py",
  "launch/localization.launch.py",
  "launch/localization_with_nonground.launch.py",
};

constexpr unsigned kMappingReader = 1U << 0;
constexpr unsigned kRelocalizationReader = 1U << 1;
constexpr unsigned kMappingYaml = 1U << 2;
constexpr unsigned kRelocalizationYaml = 1U << 3;
constexpr unsigned kRelocalizationLaunch = 1U << 4;

constexpr unsigned kMappingSurfaces = kMappingReader | kMappingYaml;
constexpr unsigned kSharedSurfaces =
  kMappingReader | kRelocalizationReader | kMappingYaml |
  kRelocalizationYaml;
constexpr unsigned kSharedYaml = kMappingYaml | kRelocalizationYaml;
constexpr unsigned kRelocalizationSurfaces =
  kRelocalizationReader | kRelocalizationYaml;
constexpr unsigned kGroundSegmentationSurfaces =
  kMappingReader | kMappingYaml | kRelocalizationYaml;

struct ParameterContract
{
  std::string legacy;
  std::string canonical;
  unsigned surfaces;
};

const std::vector<ParameterContract> kParameterContracts = {
  {"pointCloudTopic", "sensor_topic.lidar_topic", kSharedSurfaces},
  {"imuTopic", "sensor_topic.imu_topic", kSharedSurfaces},
  {"odomTopic", "output_topic.odometry", kMappingSurfaces},
  {"sensor_topic.odom_topic", "output_topic.odometry", kMappingSurfaces},
  {"gpsTopic", "sensor_topic.gps_topic", kMappingSurfaces},
  {"nonGroundCloudTopic", "output_topic.non_ground_cloud_topic",
    kGroundSegmentationSurfaces},
  {"sensor_topic.non_ground_cloud_topic", "output_topic.non_ground_cloud_topic",
    kGroundSegmentationSurfaces},
  {"output_topic.mapping_odom_topic", "output_topic.odometry",
    kRelocalizationSurfaces},
  {"mapFrame", "frames.map", kSharedSurfaces},
  {"odometryFrame", "frames.odometry", kMappingSurfaces},
  {"baselinkFrame", "frames.base_link", kSharedSurfaces},
  {"lidarFrame", "frames.lidar", kSharedSurfaces},
  {"mappingMode", "system.mapping_mode",
    kMappingSurfaces | kRelocalizationYaml},
  {"numberOfCores", "system.number_of_cores", kMappingSurfaces},
  {"mappingProcessInterval", "system.mapping_process_interval",
    kMappingSurfaces},
  {"useImuHeadingInitialization", "gps.use_imu_heading_initialization",
    kMappingSurfaces},
  {"useGpsElevation", "gps.use_elevation", kMappingSurfaces},
  {"gpsCovThreshold", "gps.covariance_threshold", kMappingSurfaces},
  {"poseCovThreshold", "gps.pose_covariance_threshold", kMappingSurfaces},
  {"sensor", "lidar.sensor_type", kSharedSurfaces},
  {"lidar.lidar_sensor_type", "lidar.sensor_type",
    kRelocalizationSurfaces},
  {"N_SCAN", "lidar.scan_lines", kSharedSurfaces},
  {"lidar.lidar_scan", "lidar.scan_lines", kRelocalizationSurfaces},
  {"Horizon_SCAN", "lidar.horizon_scan", kSharedSurfaces},
  {"lidar.lidar_horizon_scan", "lidar.horizon_scan",
    kRelocalizationSurfaces},
  {"downsampleRate", "lidar.downsample_rate", kMappingSurfaces},
  {"lidarMinRange", "lidar.min_range", kSharedSurfaces},
  {"lidar.lidar_use_min_distance", "lidar.min_range",
    kRelocalizationSurfaces},
  {"lidarMaxRange", "lidar.max_range", kSharedSurfaces},
  {"lidar.lidar_use_max_distance", "lidar.max_range",
    kRelocalizationSurfaces},
  {"lidar.lidar_point_jump_span", "lidar.point_jump_span",
    kRelocalizationSurfaces},
  {"lidar.lidar_point_time_scale", "lidar.point_time_scale",
    kRelocalizationSurfaces},
  {"lidar.lidar_rotation_noise_std", "lidar.rotation_noise_std",
    kRelocalizationSurfaces},
  {"lidar.lidar_position_noise_std", "lidar.position_noise_std",
    kRelocalizationSurfaces},
  {"lidar.lidar_lower_angle", "lidar.lower_angle_deg",
    kRelocalizationSurfaces},
  {"lidar.lidar_vertical_resolution", "lidar.vertical_resolution_deg",
    kRelocalizationSurfaces},
  {"imuType", "imu.type", kMappingSurfaces},
  {"imuAccNoise", "imu.acc_noise_std", kSharedSurfaces},
  {"imuGyrNoise", "imu.gyro_noise_std", kSharedSurfaces},
  {"imuAccBiasN", "imu.acc_bias_rw_noise_std", kSharedSurfaces},
  {"imu.acc_rw_noise_std", "imu.acc_bias_rw_noise_std",
    kRelocalizationSurfaces},
  {"imuGyrBiasN", "imu.gyro_bias_rw_noise_std", kSharedSurfaces},
  {"imu.gyro_rw_noise_std", "imu.gyro_bias_rw_noise_std",
    kRelocalizationSurfaces},
  {"imuGravity", "imu.gravity", kSharedSurfaces},
  {"gravity", "imu.gravity", kRelocalizationSurfaces},
  {"imuRPYWeight", "imu.rpy_weight", kMappingSurfaces},
  {"extrinsicTrans", "calibration.translation", kSharedSurfaces},
  {"extrinsicRot", "calibration.rotation", kSharedSurfaces},
  {"extrinsicRPY", "calibration.rpy", kSharedSurfaces},
  {"calibration.lidar_to_imu", "calibration.translation",
    kRelocalizationSurfaces},
  {"savePCD", "map_save.enabled", kMappingSurfaces},
  {"generateScDb", "map_save.generate_scan_context_db", kMappingSurfaces},
  {"savePCDDirectory", "map_save.directory", kMappingSurfaces},
  {"savePCDResolution", "map_save.resolution", kMappingSurfaces},
  {"saveGroundPCD", "map_save.save_ground_map", kMappingSurfaces},
  {"saveGroundPCDResolution", "map_save.ground_resolution",
    kMappingSurfaces},
  {"groundPCDDirectory", "map_save.ground_directory", kMappingSurfaces},
  {"groundPCDDirectoryGlobal", "map_save.ground_global_directory",
    kMappingSurfaces},
  {"groundFinalUseOdom", "map_save.ground_final_use_odometry",
    kMappingSurfaces},
  {"groundGlobalLeafSize", "map_save.ground_global_leaf_size",
    kMappingSurfaces},
  {"alignSavedMapGroundToZero", "map_save.align_ground_to_zero",
    kMappingSurfaces},
  {"savedMapGroundZOffset", "map_save.ground_z_offset", kMappingSurfaces},
  {"segmentation_only_mode", "ground_segmentation.only_mode",
    kGroundSegmentationSurfaces | kRelocalizationLaunch},
  {"groundLeafSize", "ground_segmentation.leaf_size",
    kGroundSegmentationSurfaces},
  {"groundScanStartIndex", "ground_segmentation.scan_start_index",
    kGroundSegmentationSurfaces},
  {"groundScanEndIndex", "ground_segmentation.scan_end_index",
    kGroundSegmentationSurfaces},
  {"groundAngleThreshold", "ground_segmentation.angle_threshold_deg",
    kGroundSegmentationSurfaces},
  {"groundPlaneDistance", "ground_segmentation.plane_distance",
    kGroundSegmentationSurfaces},
  {"groundPlaneMaxAngle", "ground_segmentation.plane_max_angle_deg",
    kGroundSegmentationSurfaces},
  {"groundPlaneMinPoints", "ground_segmentation.plane_min_points",
    kGroundSegmentationSurfaces},
  {"groundFlattenZ", "ground_segmentation.flatten_z",
    kGroundSegmentationSurfaces},
  {"groundFlattenValue", "ground_segmentation.flatten_value",
    kGroundSegmentationSurfaces},
  {"groundGridPatchEnable", "ground_segmentation.grid_patch.enable",
    kGroundSegmentationSurfaces},
  {"groundGridPatchResolution", "ground_segmentation.grid_patch.resolution",
    kGroundSegmentationSurfaces},
  {"groundGridPatchSearchRadius",
    "ground_segmentation.grid_patch.search_radius",
    kGroundSegmentationSurfaces},
  {"groundGridPatchMaxSlopeDeg",
    "ground_segmentation.grid_patch.max_slope_deg",
    kGroundSegmentationSurfaces},
  {"groundGridPatchMaxHeightRange",
    "ground_segmentation.grid_patch.max_height_range",
    kGroundSegmentationSurfaces},
  {"groundGridPatchMaxRoughness",
    "ground_segmentation.grid_patch.max_roughness",
    kGroundSegmentationSurfaces},
  {"groundGridPatchMaxPredictionMargin",
    "ground_segmentation.grid_patch.max_prediction_margin",
    kGroundSegmentationSurfaces},
  {"groundGridPatchMinInlierRatio",
    "ground_segmentation.grid_patch.min_inlier_ratio",
    kGroundSegmentationSurfaces},
  {"groundGridPatchMinNeighborCells",
    "ground_segmentation.grid_patch.min_neighbor_cells",
    kGroundSegmentationSurfaces},
  {"groundGridPatchMinSupportQuadrants",
    "ground_segmentation.grid_patch.min_support_quadrants",
    kGroundSegmentationSurfaces},
  {"edgeThreshold", "feature_extraction.edge_threshold", kMappingSurfaces},
  {"surfThreshold", "feature_extraction.surface_threshold",
    kMappingSurfaces},
  {"edgeFeatureMinValidNum", "feature_extraction.min_edge_features",
    kMappingSurfaces},
  {"surfFeatureMinValidNum", "feature_extraction.min_surface_features",
    kMappingSurfaces},
  {"odometrySurfLeafSize", "voxel_filter.odometry_surface_leaf_size",
    kMappingSurfaces},
  {"mappingCornerLeafSize", "voxel_filter.mapping_corner_leaf_size",
    kMappingSurfaces},
  {"mappingSurfLeafSize", "voxel_filter.mapping_surface_leaf_size",
    kMappingSurfaces},
  {"dynamicKeyframeFilterEnable", "dynamic_filter.enable",
    kMappingSurfaces},
  {"dynamicSurfKnnMaxDist", "dynamic_filter.surface_knn_max_distance",
    kMappingSurfaces},
  {"dynamicSurfPlaneMaxDist", "dynamic_filter.surface_plane_max_distance",
    kMappingSurfaces},
  {"globalStaticMapFilterEnable", "static_map_filter.enable",
    kMappingSurfaces},
  {"globalStaticVoxelSize", "static_map_filter.voxel_size",
    kMappingSurfaces},
  {"globalStaticMinKF", "static_map_filter.min_keyframes",
    kMappingSurfaces},
  {"globalStaticFilterCorners", "static_map_filter.filter_corners",
    kMappingSurfaces},
  {"z_tollerance", "motion_constraint.z_tolerance", kMappingSurfaces},
  {"rotation_tollerance", "motion_constraint.rotation_tolerance",
    kMappingSurfaces},
  {"surroundingkeyframeAddingDistThreshold",
    "local_map.keyframe_distance_threshold", kMappingSurfaces},
  {"surroundingkeyframeAddingAngleThreshold",
    "local_map.keyframe_angle_threshold", kMappingSurfaces},
  {"surroundingKeyframeDensity", "local_map.keyframe_density",
    kMappingSurfaces},
  {"surroundingKeyframeSearchRadius", "local_map.search_radius",
    kMappingSurfaces},
  {"loopClosureEnableFlag", "loop_closure.enable", kSharedSurfaces},
  {"system.enable_loopclosure", "loop_closure.enable",
    kRelocalizationSurfaces},
  {"loopClosureFrequency", "loop_closure.frequency", kMappingSurfaces},
  {"surroundingKeyframeSize", "loop_closure.surrounding_keyframe_size",
    kMappingSurfaces},
  {"historyKeyframeSearchRadius", "loop_closure.history_search_radius",
    kMappingSurfaces},
  {"historyKeyframeSearchTimeDiff",
    "loop_closure.history_search_time_difference", kMappingSurfaces},
  {"historyKeyframeSearchNum", "loop_closure.history_search_count",
    kMappingSurfaces},
  {"historyKeyframeFitnessScore", "loop_closure.fitness_threshold",
    kMappingSurfaces},
  {"loopclosure.candidate_local_map_left_range",
    "loop_closure.candidate_local_map_left_range", kRelocalizationSurfaces},
  {"loopclosure.candidate_local_map_right_range",
    "loop_closure.candidate_local_map_right_range", kRelocalizationSurfaces},
  {"loopclosure.loopclosure_local_map_left_range",
    "loop_closure.loopclosure_local_map_left_range",
    kRelocalizationSurfaces},
  {"loopclosure.near_neighbor_distance_threshold",
    "loop_closure.near_neighbor_distance_threshold",
    kRelocalizationSurfaces},
  {"loopclosure.registration_converge_threshold",
    "loop_closure.registration_converge_threshold",
    kRelocalizationSurfaces},
  {"loopclosure.skip_near_keyframe_threshold",
    "loop_closure.skip_near_keyframe_threshold", kRelocalizationSurfaces},
  {"loopclosure.skip_near_loopclosure_threshold",
    "loop_closure.skip_near_loopclosure_threshold",
    kRelocalizationSurfaces},
  {"globalMapVisualizationSearchRadius",
    "visualization.global_map_search_radius", kMappingSurfaces},
  {"globalMapVisualizationPoseDensity",
    "visualization.global_map_pose_density", kMappingSurfaces},
  {"globalMapVisualizationLeafSize", "visualization.global_map_leaf_size",
    kMappingSurfaces},
};

std::string ReadProjectFile(const std::string & relative_path)
{
  std::ifstream file(std::string(PROJECT_SOURCE_DIR) + "/" + relative_path);
  EXPECT_TRUE(file.is_open()) << "Could not open " << relative_path;
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

std::string StripCppComments(const std::string & text)
{
  enum class State { kCode, kLineComment, kBlockComment, kString, kCharacter };
  State state = State::kCode;
  std::string result;
  result.reserve(text.size());

  for (std::size_t i = 0; i < text.size(); ++i) {
    const char current = text[i];
    const char next = i + 1 < text.size() ? text[i + 1] : '\0';
    if (state == State::kLineComment) {
      if (current == '\n') {
        result += current;
        state = State::kCode;
      }
      continue;
    }
    if (state == State::kBlockComment) {
      if (current == '\n') {
        result += current;
      } else if (current == '*' && next == '/') {
        ++i;
        state = State::kCode;
      }
      continue;
    }

    result += current;
    if (state == State::kString || state == State::kCharacter) {
      if (current == '\\' && next != '\0') {
        result += next;
        ++i;
        continue;
      }
      if ((state == State::kString && current == '"') ||
        (state == State::kCharacter && current == '\''))
      {
        state = State::kCode;
      }
      continue;
    }

    if (current == '"') {
      state = State::kString;
    } else if (current == '\'') {
      state = State::kCharacter;
    } else if (current == '/' && next == '/') {
      result.pop_back();
      ++i;
      state = State::kLineComment;
    } else if (current == '/' && next == '*') {
      result.pop_back();
      ++i;
      state = State::kBlockComment;
    }
  }

  return result;
}

std::string StripPythonComments(const std::string & text)
{
  enum class State { kCode, kComment, kSingleQuoted, kDoubleQuoted };
  State state = State::kCode;
  std::string result;
  result.reserve(text.size());

  for (std::size_t i = 0; i < text.size(); ++i) {
    const char current = text[i];
    if (state == State::kComment) {
      if (current == '\n') {
        result += current;
        state = State::kCode;
      }
      continue;
    }

    result += current;
    if (state == State::kSingleQuoted || state == State::kDoubleQuoted) {
      if (current == '\\' && i + 1 < text.size()) {
        result += text[++i];
        continue;
      }
      if ((state == State::kSingleQuoted && current == '\'') ||
        (state == State::kDoubleQuoted && current == '"'))
      {
        state = State::kCode;
      }
      continue;
    }

    if (current == '#') {
      result.pop_back();
      state = State::kComment;
    } else if (current == '\'') {
      state = State::kSingleQuoted;
    } else if (current == '"') {
      state = State::kDoubleQuoted;
    }
  }

  return result;
}

bool HasQuotedString(const std::string & text, const std::string & value)
{
  return text.find("\"" + value + "\"") != std::string::npos ||
         text.find("'" + value + "'") != std::string::npos;
}

std::set<std::string> ReadCppParameterNamesFromText(
  const std::string & source_text)
{
  const std::string text = StripCppComments(source_text);
  const std::array<std::string, 5> parameter_functions = {
    "declare_parameter",
    "get_parameter",
    "util::param",
    "ReadParam",
    "ReadDerivedLegacy",
  };
  std::set<std::string> parameter_names;

  for (const auto & function : parameter_functions) {
    std::size_t search_from = 0;
    while (true) {
      const std::size_t call = text.find(function, search_from);
      if (call == std::string::npos) {
        break;
      }
      const std::size_t open = text.find('(', call + function.size());
      const std::size_t statement_end = text.find(';', call + function.size());
      const std::size_t quote = text.find('"', open);
      if (open != std::string::npos && quote != std::string::npos &&
        (statement_end == std::string::npos || quote < statement_end))
      {
        const std::size_t quote_end = text.find('"', quote + 1);
        if (quote_end != std::string::npos) {
          parameter_names.insert(
            text.substr(quote + 1, quote_end - quote - 1));
        }
      }
      search_from = call + function.size();
    }
  }

  return parameter_names;
}

std::set<std::string> ReadCppParameterNames(
  const std::string & relative_path)
{
  return ReadCppParameterNamesFromText(ReadProjectFile(relative_path));
}

// Canonical keys P2.2 introduced that have no legacy counterpart: they used to
// be hardcoded in the node constructors rather than exposed as parameters.
struct CanonicalTopicContract
{
  std::string canonical;
  unsigned surfaces;
};

const std::vector<CanonicalTopicContract> kCanonicalTopicContracts = {
  // mapping pipeline topics
  {"pipeline_topic.cloud_info", kMappingSurfaces},
  {"pipeline_topic.feature_cloud_info", kMappingSurfaces},
  {"pipeline_topic.deskewed_cloud", kMappingSurfaces},
  {"pipeline_topic.ground_cloud", kMappingSurfaces},
  {"pipeline_topic.corner_cloud", kMappingSurfaces},
  {"pipeline_topic.surface_cloud", kMappingSurfaces},
  {"pipeline_topic.loop_closure_hint", kMappingSurfaces},
  {"pipeline_topic.mapping_odometry", kMappingSurfaces},
  {"pipeline_topic.mapping_odometry_incremental", kMappingSurfaces},
  {"pipeline_topic.imu_odometry_incremental", kMappingSurfaces},
  // mapping outputs
  {"output_topic.path", kMappingSurfaces},
  {"output_topic.global_map", kMappingSurfaces},
  {"output_topic.local_map", kMappingSurfaces},
  {"output_topic.registered_cloud", kMappingSurfaces},
  {"output_topic.registered_cloud_raw", kMappingSurfaces},
  {"output_topic.ground_map", kMappingSurfaces},
  {"output_topic.trajectory", kMappingSurfaces},
  {"output_topic.imu_path", kMappingSurfaces},
  {"output_topic.loop_history", kMappingSurfaces},
  {"output_topic.loop_constraints", kMappingSurfaces},
  // localization outputs and its one external command input
  {"output_topic.path", kRelocalizationSurfaces},
  {"output_topic.global_map", kRelocalizationSurfaces},
  {"output_topic.local_map", kRelocalizationSurfaces},
  {"output_topic.current_lidar", kRelocalizationSurfaces},
  {"command_topic.initial_pose", kRelocalizationSurfaces},
  // shared downstream-facing output
  {"output_topic.diagnostics",
    kMappingSurfaces | kRelocalizationSurfaces},
};

std::set<std::string> CanonicalNamesForSurface(unsigned surface)
{
  std::set<std::string> names;
  for (const auto & contract : kParameterContracts) {
    if ((contract.surfaces & surface) != 0U) {
      names.insert(contract.canonical);
    }
  }
  for (const auto & contract : kCanonicalTopicContracts) {
    if ((contract.surfaces & surface) != 0U) {
      names.insert(contract.canonical);
    }
  }
  return names;
}

std::set<std::string> LegacyNames()
{
  std::set<std::string> names;
  for (const auto & contract : kParameterContracts) {
    names.insert(contract.legacy);
  }
  return names;
}

std::set<std::string> Union(
  const std::set<std::string> & lhs,
  const std::set<std::string> & rhs)
{
  std::set<std::string> result = lhs;
  result.insert(rhs.begin(), rhs.end());
  return result;
}

void ExpectContainsAll(
  const std::set<std::string> & actual,
  const std::set<std::string> & expected,
  const std::string & relative_path)
{
  for (const auto & name : expected) {
    EXPECT_TRUE(actual.find(name) != actual.end())
      << name << " is missing from " << relative_path;
  }
}

struct YamlParameterSurface
{
  std::set<std::string> paths;
  std::set<std::string> raw_keys;
};

YamlParameterSurface ReadYamlParameterSurfaceFromText(
  const std::string & text)
{
  std::istringstream stream(text);
  std::vector<std::pair<std::size_t, std::string>> parents;
  YamlParameterSurface surface;
  std::string line;
  bool inside_parameters = false;
  std::size_t parameters_indentation = 0;

  while (std::getline(stream, line)) {
    const std::size_t first = line.find_first_not_of(' ');
    if (first == std::string::npos || line[first] == '#') {
      continue;
    }

    if (inside_parameters && first <= parameters_indentation) {
      inside_parameters = false;
      parents.clear();
    }

    const std::size_t colon = line.find(':', first);
    if (colon == std::string::npos) {
      continue;
    }

    const std::string key = line.substr(first, colon - first);
    if (key.empty() ||
      !(std::isalpha(static_cast<unsigned char>(key.front())) ||
      key.front() == '_'))
    {
      continue;
    }
    if (key.find_first_not_of(
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.") !=
      std::string::npos)
    {
      continue;
    }

    if (key == "ros__parameters") {
      parents.clear();
      inside_parameters = true;
      parameters_indentation = first;
      continue;
    }
    if (!inside_parameters) {
      continue;
    }

    while (!parents.empty() && parents.back().first >= first) {
      parents.pop_back();
    }

    std::string path;
    for (const auto & parent : parents) {
      if (!path.empty()) {
        path += '.';
      }
      path += parent.second;
    }
    if (!path.empty()) {
      path += '.';
    }
    path += key;
    surface.paths.insert(path);
    surface.raw_keys.insert(key);

    std::string value = line.substr(colon + 1);
    const std::size_t comment = value.find('#');
    if (comment != std::string::npos) {
      value.erase(comment);
    }
    if (value.find_first_not_of(" \t\r") == std::string::npos) {
      parents.emplace_back(first, key);
    }
  }

  return surface;
}

YamlParameterSurface ReadYamlParameterSurface(
  const std::string & relative_path)
{
  return ReadYamlParameterSurfaceFromText(ReadProjectFile(relative_path));
}

void ExpectYamlContainsAll(
  const YamlParameterSurface & actual,
  const std::set<std::string> & expected,
  const std::string & relative_path)
{
  for (const auto & name : expected) {
    EXPECT_TRUE(actual.paths.find(name) != actual.paths.end())
      << name << " is missing from " << relative_path;
    EXPECT_TRUE(actual.raw_keys.find(name) == actual.raw_keys.end())
      << name << " must use nested YAML mappings in " << relative_path;
  }
}

bool HasLegacyYamlName(
  const YamlParameterSurface & surface,
  const std::string & legacy_name)
{
  if (legacy_name == "gravity") {
    return surface.paths.find(legacy_name) != surface.paths.end();
  }
  if (legacy_name.find('.') != std::string::npos) {
    return surface.paths.find(legacy_name) != surface.paths.end() ||
           surface.raw_keys.find(legacy_name) != surface.raw_keys.end();
  }
  return surface.raw_keys.find(legacy_name) != surface.raw_keys.end();
}

}  // namespace

TEST(UnifiedParameterSchema, MappingReadersUseCanonicalNames) {
  // Mapping nodes no longer declare parameters themselves; the shared loader
  // declares and validates every one of them.
  const std::set<std::string> actual =
    ReadCppParameterNames("src/common/slam_config.cpp");
  ExpectContainsAll(
    actual, CanonicalNamesForSurface(kMappingReader),
    "src/common/slam_config.cpp");
}

TEST(UnifiedParameterSchema, RelocalizationReaderUsesCanonicalNames) {
  const std::set<std::string> actual =
    Union(
    Union(
      ReadCppParameterNames("src/common/slam_config.cpp"),
      ReadCppParameterNames("src/slam/system.cpp")),
    ReadCppParameterNames("src/slam/localization.cpp"));
  ExpectContainsAll(
    actual, CanonicalNamesForSurface(kRelocalizationReader),
    "relocalization C++ readers");
}

TEST(UnifiedParameterSchema, MappingYamlUsesCanonicalNames) {
  ExpectYamlContainsAll(
    ReadYamlParameterSurface("config/mapping.yaml"),
    CanonicalNamesForSurface(kMappingYaml),
    "config/mapping.yaml");
}

TEST(UnifiedParameterSchema, RelocalizationYamlUsesCanonicalNames) {
  ExpectYamlContainsAll(
    ReadYamlParameterSurface("config/relocalization.yaml"),
    CanonicalNamesForSurface(kRelocalizationYaml),
    "config/relocalization.yaml");
}

TEST(UnifiedParameterSchema, TiledMapParametersUseCanonicalNames) {
  const std::set<std::string> localization_names = {
    "localization.map_mode",
    "localization.tiled_map.active_radius_tiles",
    "localization.tiled_map.prefetch_radius_tiles",
    "localization.tiled_map.max_cached_tiles",
    "localization.tiled_map.max_cache_mib",
    "localization.tiled_map.load_timeout_ms",
  };
  const std::set<std::string> map_save_names = {
    "map_save.save_tiled_map",
    "map_save.tiled_map.tile_size_m",
    "map_save.tiled_map.output_subdirectory",
    "map_save.tiled_map.overview_resolution_m",
  };

  ExpectContainsAll(ReadCppParameterNames("src/common/slam_config.cpp"),
                    localization_names, "src/common/slam_config.cpp");
  ExpectYamlContainsAll(ReadYamlParameterSurface("config/relocalization.yaml"),
                        localization_names, "config/relocalization.yaml");
  ExpectContainsAll(ReadCppParameterNames("src/common/slam_config.cpp"),
                    map_save_names, "src/common/slam_config.cpp");
  ExpectYamlContainsAll(ReadYamlParameterSurface("config/mapping.yaml"),
                        map_save_names, "config/mapping.yaml");
  const std::string mapping_yaml = ReadProjectFile("config/mapping.yaml");
  const std::string relocalization_yaml =
      ReadProjectFile("config/relocalization.yaml");
  EXPECT_NE(mapping_yaml.find("save_tiled_map: false"),
            std::string::npos);
  EXPECT_NE(mapping_yaml.find("tile_size_m: 100.0"), std::string::npos);
  EXPECT_NE(mapping_yaml.find("output_subdirectory: \"tiled\""),
            std::string::npos);
  EXPECT_NE(mapping_yaml.find("overview_resolution_m: 5.0"),
            std::string::npos);
  EXPECT_NE(relocalization_yaml.find("map_mode: auto"), std::string::npos);
  EXPECT_NE(relocalization_yaml.find("active_radius_tiles: 1"),
            std::string::npos);
  EXPECT_NE(relocalization_yaml.find("prefetch_radius_tiles: 2"),
            std::string::npos);
  EXPECT_NE(relocalization_yaml.find("max_cached_tiles: 25"),
            std::string::npos);
  EXPECT_NE(relocalization_yaml.find("max_cache_mib: 1024"),
            std::string::npos);
  EXPECT_NE(relocalization_yaml.find("load_timeout_ms: 1000"),
            std::string::npos);
}

TEST(UnifiedParameterSchema, LaunchOverridesUseCanonicalNames) {
  const std::string launch = StripPythonComments(
    ReadProjectFile("launch/localization_with_nonground.launch.py"));
  for (const auto & name :
    CanonicalNamesForSurface(kRelocalizationLaunch))
  {
    EXPECT_TRUE(HasQuotedString(launch, name))
      << name << " is missing from localization launch overrides";
  }
}

TEST(UnifiedParameterSchema, ProjectSurfaceRejectsLegacyExternalNames) {
  const std::set<std::string> legacy_names = LegacyNames();
  for (const auto & relative_path : kParameterFiles) {
    const bool is_yaml =
      relative_path.size() >= 5 &&
      relative_path.substr(relative_path.size() - 5) == ".yaml";
    const bool is_python =
      relative_path.size() >= 3 &&
      relative_path.substr(relative_path.size() - 3) == ".py";
    if (is_yaml) {
      const YamlParameterSurface surface =
        ReadYamlParameterSurface(relative_path);
      for (const auto & legacy_name : legacy_names) {
        EXPECT_FALSE(HasLegacyYamlName(surface, legacy_name))
          << "legacy external parameter " << legacy_name
          << " is still present in " << relative_path;
      }
      continue;
    }
    if (is_python) {
      const std::string text =
        StripPythonComments(ReadProjectFile(relative_path));
      for (const auto & legacy_name : legacy_names) {
        EXPECT_FALSE(HasQuotedString(text, legacy_name))
          << "legacy external parameter " << legacy_name
          << " is still present in " << relative_path;
      }
      continue;
    }
    const std::set<std::string> parameter_names =
      ReadCppParameterNames(relative_path);
    for (const auto & legacy_name : legacy_names) {
      EXPECT_TRUE(
        parameter_names.find(legacy_name) ==
        parameter_names.end())
        << "legacy external parameter " << legacy_name
        << " is still present in " << relative_path;
    }
  }
}

TEST(UnifiedParameterSchema, CppScannerIgnoresCommentedParameterNames) {
  const std::string source =
    "// declare_parameter(\"sensor_topic.lidar_topic\", \"points\");\n"
    "/* get_parameter(\"pointCloudTopic\", topic); */\n"
    "declare_parameter(\"actual.parameter\", true);\n";
  const std::set<std::string> names =
    ReadCppParameterNamesFromText(source);

  EXPECT_TRUE(names.find("actual.parameter") != names.end());
  EXPECT_TRUE(names.find("sensor_topic.lidar_topic") == names.end());
  EXPECT_TRUE(names.find("pointCloudTopic") == names.end());
}

TEST(UnifiedParameterSchema, LaunchScannerIgnoresCommentedParameterNames) {
  const std::string launch =
    "# \"ground_segmentation.only_mode\"\n"
    "parameters=[{\"actual.parameter\": True}] "
    "# \"segmentation_only_mode\"\n";
  const std::string stripped = StripPythonComments(launch);

  EXPECT_TRUE(HasQuotedString(stripped, "actual.parameter"));
  EXPECT_FALSE(
    HasQuotedString(stripped, "ground_segmentation.only_mode"));
  EXPECT_FALSE(HasQuotedString(stripped, "segmentation_only_mode"));
}

TEST(UnifiedParameterSchema, YamlScannerStopsOutsideRosParameters) {
  const std::string yaml =
    "/node:\n"
    "  ros__parameters:\n"
    "    sensor_topic:\n"
    "      lidar_topic: /points\n"
    "/outside:\n"
    "  pointCloudTopic: ignored\n";
  const YamlParameterSurface surface =
    ReadYamlParameterSurfaceFromText(yaml);

  EXPECT_TRUE(
    surface.paths.find("sensor_topic.lidar_topic") !=
    surface.paths.end());
  EXPECT_TRUE(
    surface.raw_keys.find("pointCloudTopic") ==
    surface.raw_keys.end());
}
