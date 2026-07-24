#include <gtest/gtest.h>

#include <array>
#include <cctype>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

const std::array<std::string, 7> kParameterFiles = {
    "include/lio_sam/utility.hpp",
    "src/imageProjection.cpp",
    "src/slam/system.cpp",
    "config/mapping.yaml",
    "config/relocalization.yaml",
    "launch/mapping.launch.py",
    "launch/localization_with_nonground.launch.py",
};

const std::vector<std::string> kLegacyParameterNames = {
    "pointCloudTopic",
    "imuTopic",
    "odomTopic",
    "gpsTopic",
    "nonGroundCloudTopic",
    "mapFrame",
    "odometryFrame",
    "baselinkFrame",
    "lidarFrame",
    "mappingMode",
    "numberOfCores",
    "mappingProcessInterval",
    "useImuHeadingInitialization",
    "useGpsElevation",
    "gpsCovThreshold",
    "poseCovThreshold",
    "sensor",
    "lidar.lidar_sensor_type",
    "N_SCAN",
    "lidar.lidar_scan",
    "Horizon_SCAN",
    "lidar.lidar_horizon_scan",
    "downsampleRate",
    "lidarMinRange",
    "lidar.lidar_use_min_distance",
    "lidarMaxRange",
    "lidar.lidar_use_max_distance",
    "lidar.lidar_point_jump_span",
    "lidar.lidar_point_time_scale",
    "lidar.lidar_rotation_noise_std",
    "lidar.lidar_position_noise_std",
    "lidar.lidar_lower_angle",
    "lidar.lidar_vertical_resolution",
    "imuType",
    "imuAccNoise",
    "imuGyrNoise",
    "imuAccBiasN",
    "imu.acc_rw_noise_std",
    "imuGyrBiasN",
    "imu.gyro_rw_noise_std",
    "imuGravity",
    "gravity",
    "imuRPYWeight",
    "extrinsicTrans",
    "extrinsicRot",
    "extrinsicRPY",
    "calibration.lidar_to_imu",
    "savePCD",
    "generateScDb",
    "savePCDDirectory",
    "savePCDResolution",
    "saveGroundPCD",
    "saveGroundPCDResolution",
    "groundPCDDirectory",
    "groundPCDDirectoryGlobal",
    "groundFinalUseOdom",
    "groundGlobalLeafSize",
    "alignSavedMapGroundToZero",
    "savedMapGroundZOffset",
    "segmentation_only_mode",
    "groundLeafSize",
    "groundScanStartIndex",
    "groundScanEndIndex",
    "groundAngleThreshold",
    "groundPlaneDistance",
    "groundPlaneMaxAngle",
    "groundPlaneMinPoints",
    "groundFlattenZ",
    "groundFlattenValue",
    "groundGridPatchEnable",
    "groundGridPatchResolution",
    "groundGridPatchSearchRadius",
    "groundGridPatchMaxSlopeDeg",
    "groundGridPatchMaxHeightRange",
    "groundGridPatchMaxRoughness",
    "groundGridPatchMaxPredictionMargin",
    "groundGridPatchMinInlierRatio",
    "groundGridPatchMinNeighborCells",
    "groundGridPatchMinSupportQuadrants",
    "edgeThreshold",
    "surfThreshold",
    "edgeFeatureMinValidNum",
    "surfFeatureMinValidNum",
    "odometrySurfLeafSize",
    "mappingCornerLeafSize",
    "mappingSurfLeafSize",
    "dynamicKeyframeFilterEnable",
    "dynamicSurfKnnMaxDist",
    "dynamicSurfPlaneMaxDist",
    "globalStaticMapFilterEnable",
    "globalStaticVoxelSize",
    "globalStaticMinKF",
    "globalStaticFilterCorners",
    "z_tollerance",
    "rotation_tollerance",
    "surroundingkeyframeAddingDistThreshold",
    "surroundingkeyframeAddingAngleThreshold",
    "surroundingKeyframeDensity",
    "surroundingKeyframeSearchRadius",
    "loopClosureEnableFlag",
    "loopClosureFrequency",
    "surroundingKeyframeSize",
    "historyKeyframeSearchRadius",
    "historyKeyframeSearchTimeDiff",
    "historyKeyframeSearchNum",
    "historyKeyframeFitnessScore",
    "loopclosure.candidate_local_map_left_range",
    "loopclosure.candidate_local_map_right_range",
    "loopclosure.loopclosure_local_map_left_range",
    "loopclosure.near_neighbor_distance_threshold",
    "loopclosure.registration_converge_threshold",
    "loopclosure.skip_near_keyframe_threshold",
    "loopclosure.skip_near_loopclosure_threshold",
    "globalMapVisualizationSearchRadius",
    "globalMapVisualizationPoseDensity",
    "globalMapVisualizationLeafSize",
    "system.enable_loopclosure",
    "system.enable_visualize_global_map",
    "system.global_map_visualization_resolution",
};

std::string ReadProjectFile(const std::string& relative_path) {
  std::ifstream file(std::string(PROJECT_SOURCE_DIR) + "/" + relative_path);
  EXPECT_TRUE(file.is_open()) << "Could not open " << relative_path;
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

bool HasQuotedString(const std::string& text, const std::string& value) {
  return text.find("\"" + value + "\"") != std::string::npos ||
         text.find("'" + value + "'") != std::string::npos;
}

std::set<std::string> ReadCppParameterNames(
    const std::string& relative_path) {
  const std::string text = ReadProjectFile(relative_path);
  const std::array<std::string, 3> parameter_functions = {
      "declare_parameter",
      "get_parameter",
      "util::param",
  };
  std::set<std::string> parameter_names;

  for (const auto& function : parameter_functions) {
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
          (statement_end == std::string::npos || quote < statement_end)) {
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

void ExpectCppParameters(
    const std::string& relative_path,
    const std::vector<std::string>& parameter_names) {
  const std::set<std::string> actual_names =
      ReadCppParameterNames(relative_path);
  for (const auto& parameter_name : parameter_names) {
    EXPECT_TRUE(actual_names.find(parameter_name) != actual_names.end())
        << parameter_name << " is missing from " << relative_path;
  }
}

void ExpectNoLegacyCppParameters(
    const std::vector<std::string>& relative_paths) {
  for (const auto& relative_path : relative_paths) {
    const std::set<std::string> parameter_names =
        ReadCppParameterNames(relative_path);
    for (const auto& legacy_name : kLegacyParameterNames) {
      EXPECT_TRUE(parameter_names.find(legacy_name) == parameter_names.end())
          << "legacy external parameter " << legacy_name
          << " is still present in " << relative_path;
    }
  }
}

void ExpectQuotedParameters(
    const std::string& relative_path,
    const std::vector<std::string>& parameter_names) {
  const std::string text = ReadProjectFile(relative_path);
  for (const auto& parameter_name : parameter_names) {
    EXPECT_TRUE(HasQuotedString(text, parameter_name))
        << parameter_name << " is missing from " << relative_path;
  }
}

void ExpectNoLegacyQuotedParameters(
    const std::vector<std::string>& relative_paths) {
  for (const auto& relative_path : relative_paths) {
    const std::string text = ReadProjectFile(relative_path);
    for (const auto& legacy_name : kLegacyParameterNames) {
      EXPECT_FALSE(HasQuotedString(text, legacy_name))
          << "legacy external parameter " << legacy_name
          << " is still present in " << relative_path;
    }
  }
}

std::set<std::string> ReadYamlParameterPaths(
    const std::string& relative_path) {
  std::istringstream stream(ReadProjectFile(relative_path));
  std::vector<std::pair<std::size_t, std::string>> parents;
  std::set<std::string> paths;
  std::string line;
  bool inside_parameters = false;

  while (std::getline(stream, line)) {
    const std::size_t first = line.find_first_not_of(' ');
    if (first == std::string::npos || line[first] == '#') {
      continue;
    }

    const std::size_t colon = line.find(':', first);
    if (colon == std::string::npos) {
      continue;
    }

    const std::string key = line.substr(first, colon - first);
    if (key.empty() ||
        !(std::isalpha(static_cast<unsigned char>(key.front())) ||
          key.front() == '_')) {
      continue;
    }
    if (key.find_first_not_of(
            "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.") !=
        std::string::npos) {
      continue;
    }

    if (key == "ros__parameters") {
      parents.clear();
      inside_parameters = true;
      continue;
    }
    if (!inside_parameters) {
      continue;
    }

    while (!parents.empty() && parents.back().first >= first) {
      parents.pop_back();
    }

    std::string path;
    for (const auto& parent : parents) {
      if (!path.empty()) {
        path += '.';
      }
      path += parent.second;
    }
    if (!path.empty()) {
      path += '.';
    }
    path += key;
    paths.insert(path);

    std::string value = line.substr(colon + 1);
    const std::size_t comment = value.find('#');
    if (comment != std::string::npos) {
      value.erase(comment);
    }
    if (value.find_first_not_of(" \t\r") == std::string::npos) {
      parents.emplace_back(first, key);
    }
  }

  return paths;
}

bool HasRawYamlKey(const std::string& relative_path,
                   const std::string& expected_key) {
  std::istringstream stream(ReadProjectFile(relative_path));
  std::string line;
  while (std::getline(stream, line)) {
    const std::size_t first = line.find_first_not_of(' ');
    if (first == std::string::npos || line[first] == '#') {
      continue;
    }
    const std::size_t colon = line.find(':', first);
    if (colon != std::string::npos &&
        line.substr(first, colon - first) == expected_key) {
      return true;
    }
  }
  return false;
}

void ExpectYamlParameters(
    const std::string& relative_path,
    const std::vector<std::string>& parameter_names) {
  const std::set<std::string> paths = ReadYamlParameterPaths(relative_path);
  for (const auto& parameter_name : parameter_names) {
    EXPECT_TRUE(paths.find(parameter_name) != paths.end())
        << parameter_name << " is missing from " << relative_path;
    EXPECT_FALSE(HasRawYamlKey(relative_path, parameter_name))
        << parameter_name << " must use nested YAML mappings in "
        << relative_path;
  }
}

void ExpectNoLegacyYamlParameters(const std::string& relative_path) {
  const std::set<std::string> paths = ReadYamlParameterPaths(relative_path);
  for (const auto& legacy_name : kLegacyParameterNames) {
    bool found = false;
    for (const auto& path : paths) {
      const std::size_t last_dot = path.rfind('.');
      const std::string leaf =
          last_dot == std::string::npos ? path : path.substr(last_dot + 1);
      if (path == legacy_name ||
          (legacy_name.find('.') == std::string::npos &&
           legacy_name != "gravity" && leaf == legacy_name)) {
        found = true;
        break;
      }
    }
    EXPECT_FALSE(found)
        << "legacy external parameter " << legacy_name
        << " is still present in " << relative_path;
  }
}

}  // namespace

TEST(UnifiedParameterSchema, MappingReadersUseCanonicalNames) {
  ExpectCppParameters(
      "include/lio_sam/utility.hpp",
      {
          "sensor_topic.lidar_topic",
          "lidar.sensor_type",
          "imu.gravity",
          "calibration.translation",
          "calibration.rotation",
          "calibration.rpy",
          "map_save.generate_scan_context_db",
          "map_save.align_ground_to_zero",
          "map_save.ground_z_offset",
          "ground_segmentation.grid_patch.max_slope_deg",
          "loop_closure.enable",
      });
  ExpectCppParameters(
      "src/imageProjection.cpp",
      {
          "sensor_topic.non_ground_cloud_topic",
          "ground_segmentation.only_mode",
      });
  ExpectNoLegacyCppParameters(
      {"include/lio_sam/utility.hpp", "src/imageProjection.cpp"});
}

TEST(UnifiedParameterSchema, RelocalizationReaderUsesCanonicalNames) {
  ExpectCppParameters(
      "src/slam/system.cpp",
      {
          "sensor_topic.lidar_topic",
          "lidar.sensor_type",
          "imu.gravity",
          "calibration.translation",
          "calibration.rotation",
          "calibration.rpy",
          "loop_closure.enable",
      });
  ExpectNoLegacyCppParameters({"src/slam/system.cpp"});
}

TEST(UnifiedParameterSchema, MappingYamlUsesCanonicalNames) {
  ExpectYamlParameters(
      "config/mapping.yaml",
      {
          "sensor_topic.lidar_topic",
          "lidar.sensor_type",
          "imu.gravity",
          "calibration.translation",
          "map_save.generate_scan_context_db",
          "map_save.align_ground_to_zero",
          "map_save.ground_z_offset",
          "ground_segmentation.grid_patch.max_slope_deg",
          "loop_closure.enable",
      });
  ExpectNoLegacyYamlParameters("config/mapping.yaml");
}

TEST(UnifiedParameterSchema, RelocalizationYamlUsesCanonicalNames) {
  ExpectYamlParameters(
      "config/relocalization.yaml",
      {
          "sensor_topic.lidar_topic",
          "lidar.sensor_type",
          "imu.gravity",
          "calibration.translation",
          "ground_segmentation.grid_patch.max_slope_deg",
          "loop_closure.enable",
      });
  ExpectNoLegacyYamlParameters("config/relocalization.yaml");
}

TEST(UnifiedParameterSchema, LaunchOverridesUseCanonicalNames) {
  ExpectQuotedParameters(
      "launch/localization_with_nonground.launch.py",
      {"ground_segmentation.only_mode"});
  ExpectNoLegacyQuotedParameters(
      {"launch/mapping.launch.py",
       "launch/localization_with_nonground.launch.py"});
}

TEST(UnifiedParameterSchema, ProjectSurfaceRejectsLegacyExternalNames) {
  for (const auto& relative_path : kParameterFiles) {
    if (relative_path.size() >= 5 &&
        relative_path.substr(relative_path.size() - 5) == ".yaml") {
      ExpectNoLegacyYamlParameters(relative_path);
    } else if (relative_path.size() >= 3 &&
               relative_path.substr(relative_path.size() - 3) == ".py") {
      ExpectNoLegacyQuotedParameters({relative_path});
    } else {
      ExpectNoLegacyCppParameters({relative_path});
    }
  }
}
