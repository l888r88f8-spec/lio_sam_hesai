#include <gtest/gtest.h>

#include <array>
#include <fstream>
#include <sstream>
#include <string>

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
