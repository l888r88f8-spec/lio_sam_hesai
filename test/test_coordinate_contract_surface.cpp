#include <gtest/gtest.h>

#include <array>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace
{

std::string ReadProjectFile(const std::string & relative_path)
{
  std::ifstream file(std::string(PROJECT_SOURCE_DIR) + "/" + relative_path);
  EXPECT_TRUE(file.is_open()) << "Could not open " << relative_path;
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

}  // namespace

TEST(CoordinateContractSurface, MappingKeepsMapOdomBaseTopology)
{
  const auto launch = ReadProjectFile("launch/mapping.launch.py");
  const auto fusion = ReadProjectFile("src/imuPreintegration.cpp");
  EXPECT_NE(launch.find("\"map\", \"odom\""), std::string::npos);
  EXPECT_NE(fusion.find("ts.child_frame_id = config_.frames.base_link"),
            std::string::npos);
  EXPECT_NE(fusion.find("tfBroadcaster->sendTransform(ts)"),
            std::string::npos);
}

TEST(CoordinateContractSurface, LocalizationHasNoOdomTfDependency)
{
  const auto system = ReadProjectFile("src/slam/system.cpp");
  EXPECT_EQ(system.find("frames.odometry"), std::string::npos);
  EXPECT_EQ(system.find("LookupExternalOdom"), std::string::npos);
  EXPECT_NE(system.find("frames.map"), std::string::npos);
  EXPECT_NE(system.find("frames.base_link"), std::string::npos);
}

TEST(CoordinateContractSurface, MappingTopicsComeFromCanonicalGroups)
{
  const std::array<std::string, 4> sources = {
    "src/imageProjection.cpp",
    "src/featureExtraction.cpp",
    "src/imuPreintegration.cpp",
    "src/mapOptimization.cpp",
  };
  const std::array<std::string, 19> literals = {
    "lio_sam/deskew/cloud_info",
    "lio_sam/deskew/cloud_deskewed",
    "lio_sam/deskew/ground_cloud_global",
    "lio_sam/deskew/cloud_nonground",
    "lio_sam/feature/cloud_info",
    "lio_sam/feature/cloud_corner",
    "lio_sam/feature/cloud_surface",
    "lio_sam/mapping/odometry",
    "lio_sam/mapping/odometry_incremental",
    "lio_sam/mapping/path",
    "lio_sam/mapping/map_global",
    "lio_sam/mapping/map_local",
    "lio_sam/mapping/cloud_registered",
    "lio_sam/mapping/cloud_registered_raw",
    "lio_sam/mapping/ground_cloud_global",
    "lio_sam/mapping/trajectory",
    "lio_sam/imu/path",
    "lio_sam/mapping/icp_loop_closure_history_cloud",
    "/lio_sam/mapping/loop_closure_constraints",
  };

  for (const auto & source : sources) {
    const auto text = ReadProjectFile(source);
    for (const auto & literal : literals) {
      EXPECT_EQ(text.find("\"" + literal + "\""), std::string::npos)
        << source << " must take " << literal
        << " from pipeline_topics/outputs instead of a literal";
    }
  }

  // The strings still resolve to the same ROS names: the shared configuration
  // owns them, so moving them out of the nodes changes no runtime graph.
  const auto config_sources =
    ReadProjectFile("include/common/slam_config.h") +
    ReadProjectFile("src/common/slam_config.cpp");
  for (const auto & literal : literals) {
    EXPECT_NE(config_sources.find(literal), std::string::npos)
      << literal << " must come from the shared configuration";
  }
}

TEST(CoordinateContractSurface, LocalizationTopicLiteralsAreFrozen)
{
  // Localization topics now resolve from the shared configuration loader rather
  // than from hardcoded literals in src/slam/system.cpp.
  const auto loader = ReadProjectFile("src/common/slam_config.cpp");
  const std::array<std::string, 4> topics = {
    "lio_sam/localization/path",
    "lio_sam/localization/global_map",
    "lio_sam/localization/local_map",
    "lio_sam/localization/current_lidar",
  };
  for (const auto & topic : topics) {
    EXPECT_NE(loader.find(topic), std::string::npos)
      << topic << " is missing from src/common/slam_config.cpp";
  }

  const auto yaml = ReadProjectFile("config/relocalization.yaml");
  EXPECT_NE(yaml.find("lio_sam/localization/odometry"), std::string::npos)
    << "lio_sam/localization/odometry is missing from config/relocalization.yaml";
}

TEST(CoordinateContractSurface, LocalizationBindsTransformsWithoutSingletonOrOdom)
{
  const std::array<std::string, 6> files = {
    "include/slam/system.h",
    "include/slam/localization.h",
    "include/slam/preprocessing.h",
    "src/slam/system.cpp",
    "src/slam/localization.cpp",
    "src/slam/preprocessing.cpp",
  };
  for (const auto & file : files) {
    const auto text = ReadProjectFile(file);
    EXPECT_EQ(text.find("ConfigParameters::Instance"), std::string::npos)
      << file << " must read injected configuration, not the singleton";
    EXPECT_EQ(text.find("slam/config_parameters.h"), std::string::npos)
      << file << " must not include the removed singleton header";
    EXPECT_EQ(text.find("last_odom_state_"), std::string::npos)
      << file << " must name the internal state last_local_state_";
    EXPECT_EQ(text.find("frames.odometry"), std::string::npos)
      << file << " must not depend on an odom frame";
  }
}

TEST(CoordinateContractSurface, LaunchStaticTranslationsAreFrozen)
{
  const std::array<std::string, 3> launch_files = {
    "launch/mapping.launch.py",
    "launch/localization.launch.py",
    "launch/localization_with_nonground.launch.py",
  };
  for (const auto & file : launch_files) {
    const auto text = ReadProjectFile(file);
    EXPECT_NE(text.find("\"0.02\""), std::string::npos) << file;
    EXPECT_NE(text.find("\"0.32\""), std::string::npos) << file;
    EXPECT_NE(text.find("\"0.22\""), std::string::npos) << file;
  }
}

TEST(CoordinateContractSurface, RepositoryYamlDropsLegacyOdometryKeys)
{
  // The canonical paths are exercised by default; the legacy aliases stay
  // supported by the loader but are no longer shipped in the repository YAML.
  const std::array<std::string, 2> legacy_keys = {
    "odom_topic",
    "mapping_odom_topic",
  };
  const std::array<std::string, 2> yaml_files = {
    "config/mapping.yaml",
    "config/relocalization.yaml",
  };
  for (const auto & file : yaml_files) {
    const auto text = ReadProjectFile(file);
    for (const auto & key : legacy_keys) {
      EXPECT_EQ(text.find(key + ":"), std::string::npos)
        << file << " must use output_topic.odometry, not " << key;
    }
  }
}

TEST(CoordinateContractSurface, RvizTopicsResolveToCanonicalConfigValues)
{
  const auto rviz = ReadProjectFile("config/rviz2.rviz");
  const auto config_text =
    ReadProjectFile("config/mapping.yaml") +
    ReadProjectFile("config/relocalization.yaml");

  std::size_t checked = 0;
  std::istringstream stream(rviz);
  std::string line;
  while (std::getline(stream, line)) {
    const std::size_t key = line.find("Value:");
    if (key == std::string::npos) {
      continue;
    }
    const std::size_t start = line.find_first_not_of(" \t", key + 6);
    if (start == std::string::npos) {
      continue;
    }
    const std::string value = line.substr(start);
    if (value.empty() || value.front() != '/') {
      continue;  // Only topic-valued entries are part of the contract.
    }

    // ROS treats "lio_sam/x" and "/lio_sam/x" as the same topic, so accept
    // either spelling as long as it is a configured value.
    const std::string bare = value.substr(1);
    EXPECT_TRUE(config_text.find("\"" + bare + "\"") != std::string::npos ||
                config_text.find("\"/" + bare + "\"") != std::string::npos)
      << "RViz topic " << value << " is not a canonical configuration value";
    ++checked;
  }
  EXPECT_GT(checked, 0U) << "no topic entries were checked in config/rviz2.rviz";
}

TEST(CoordinateContractSurface, LaunchFilesUseCanonicalUseSimTime)
{
  const std::array<std::string, 3> launches = {
    "launch/mapping.launch.py",
    "launch/localization.launch.py",
    "launch/localization_with_nonground.launch.py",
  };
  for (const auto & file : launches) {
    const auto text = ReadProjectFile(file);
    // Canonical argument name, plus the legacy alias accepted for one cycle.
    EXPECT_NE(text.find("\"use_sim_time\","), std::string::npos) << file;
    EXPECT_NE(text.find("\"use_sim\","), std::string::npos) << file;
    EXPECT_NE(text.find("LEGACY_USE_SIM_UNSET"), std::string::npos) << file;
    // Every node is wired to the resolved value, not to either raw argument.
    EXPECT_NE(text.find("_effective_use_sim_time()"), std::string::npos) << file;
    EXPECT_NE(text.find("\"use_sim_time\": use_sim"), std::string::npos) << file;
    EXPECT_EQ(text.find("\"use_sim_time\": use_sim_time"), std::string::npos)
      << file << " must not read the raw canonical argument directly";
  }
}

namespace
{

// Finds the end of a YAML block that starts at `block_start` and whose keys are
// indented by `indentation` spaces.
std::size_t FindYamlBlockEnd(const std::string & text,
                             std::size_t block_start,
                             std::size_t indentation)
{
  std::size_t line_start = text.find('\n', block_start + 1);
  while (line_start != std::string::npos && line_start + 1 < text.size()) {
    ++line_start;
    const std::size_t line_end = text.find('\n', line_start);
    const std::size_t first = text.find_first_not_of(' ', line_start);
    if (first != std::string::npos &&
      (line_end == std::string::npos || first < line_end) &&
      text[first] != '#' && first - line_start <= indentation)
    {
      return line_start;
    }
    line_start = line_end;
  }
  return text.size();
}

void ExpectYamlGroupValue(const std::string & file,
                          const std::string & group,
                          const std::string & leaf,
                          const std::string & value)
{
  const auto text = ReadProjectFile(file);
  const std::size_t block = text.find("\n    " + group + ":");
  ASSERT_NE(block, std::string::npos) << group << " is missing from " << file;
  const std::size_t block_end = FindYamlBlockEnd(text, block, 4);
  const std::size_t entry = text.find("\n      " + leaf + ": \"" + value + "\"",
                                      block);
  EXPECT_TRUE(entry != std::string::npos && entry < block_end)
    << file << ": " << group << "." << leaf << " must be \"" << value << "\"";
}

}  // namespace

TEST(CoordinateContractSurface, MappingYamlResolvesToFrozenTopicStrings)
{
  const std::string file = "config/mapping.yaml";
  const std::array<std::array<std::string, 3>, 21> entries = {{
    {"pipeline_topic", "cloud_info", "lio_sam/deskew/cloud_info"},
    {"pipeline_topic", "feature_cloud_info", "lio_sam/feature/cloud_info"},
    {"pipeline_topic", "deskewed_cloud", "lio_sam/deskew/cloud_deskewed"},
    {"pipeline_topic", "ground_cloud", "lio_sam/deskew/ground_cloud_global"},
    {"pipeline_topic", "corner_cloud", "lio_sam/feature/cloud_corner"},
    {"pipeline_topic", "surface_cloud", "lio_sam/feature/cloud_surface"},
    {"pipeline_topic", "loop_closure_hint", "lio_loop/loop_closure_detection"},
    {"pipeline_topic", "mapping_odometry", "lio_sam/mapping/odometry"},
    {"pipeline_topic", "mapping_odometry_incremental",
      "lio_sam/mapping/odometry_incremental"},
    {"pipeline_topic", "imu_odometry_incremental", "odometry/imu_incremental"},
    {"output_topic", "odometry", "odometry/imu"},
    {"output_topic", "path", "lio_sam/mapping/path"},
    {"output_topic", "global_map", "lio_sam/mapping/map_global"},
    {"output_topic", "local_map", "lio_sam/mapping/map_local"},
    {"output_topic", "registered_cloud", "lio_sam/mapping/cloud_registered"},
    {"output_topic", "registered_cloud_raw",
      "lio_sam/mapping/cloud_registered_raw"},
    {"output_topic", "ground_map", "lio_sam/mapping/ground_cloud_global"},
    {"output_topic", "trajectory", "lio_sam/mapping/trajectory"},
    {"output_topic", "imu_path", "lio_sam/imu/path"},
    {"output_topic", "loop_history",
      "lio_sam/mapping/icp_loop_closure_history_cloud"},
    {"output_topic", "loop_constraints",
      "/lio_sam/mapping/loop_closure_constraints"},
  }};
  for (const auto & entry : entries) {
    ExpectYamlGroupValue(file, entry[0], entry[1], entry[2]);
  }
}

TEST(CoordinateContractSurface, LocalizationYamlResolvesToFrozenTopicStrings)
{
  const std::string file = "config/relocalization.yaml";
  const std::array<std::array<std::string, 3>, 6> entries = {{
    {"command_topic", "initial_pose", "/initialpose"},
    {"output_topic", "odometry", "lio_sam/localization/odometry"},
    {"output_topic", "path", "lio_sam/localization/path"},
    {"output_topic", "global_map", "lio_sam/localization/global_map"},
    {"output_topic", "local_map", "lio_sam/localization/local_map"},
    {"output_topic", "current_lidar", "lio_sam/localization/current_lidar"},
  }};
  for (const auto & entry : entries) {
    ExpectYamlGroupValue(file, entry[0], entry[1], entry[2]);
  }
}
