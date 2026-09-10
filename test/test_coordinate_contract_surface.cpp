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
  EXPECT_NE(fusion.find("ts.child_frame_id = baselinkFrame"),
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

TEST(CoordinateContractSurface, MappingTopicLiteralsAreFrozen)
{
  const auto optimization = ReadProjectFile("src/mapOptimization.cpp");
  const std::array<std::string, 5> topics = {
    "lio_sam/mapping/odometry",
    "lio_sam/mapping/path",
    "lio_sam/mapping/map_global",
    "lio_sam/mapping/map_local",
    "lio_sam/mapping/cloud_registered",
  };
  for (const auto & topic : topics) {
    EXPECT_NE(optimization.find(topic), std::string::npos)
      << topic << " is missing from src/mapOptimization.cpp";
  }
}

TEST(CoordinateContractSurface, LocalizationTopicLiteralsAreFrozen)
{
  const auto system = ReadProjectFile("src/slam/system.cpp");
  const std::array<std::string, 4> topics = {
    "lio_sam/localization/path",
    "lio_sam/localization/global_map",
    "lio_sam/localization/local_map",
    "lio_sam/localization/current_lidar",
  };
  for (const auto & topic : topics) {
    EXPECT_NE(system.find(topic), std::string::npos)
      << topic << " is missing from src/slam/system.cpp";
  }

  const auto yaml = ReadProjectFile("config/relocalization.yaml");
  EXPECT_NE(yaml.find("lio_sam/localization/odometry"), std::string::npos)
    << "lio_sam/localization/odometry is missing from config/relocalization.yaml";
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
