#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>

namespace {

std::string ReadProjectFile(const std::string& relative_path) {
  std::ifstream file(std::string(PROJECT_SOURCE_DIR) + "/" + relative_path);
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

void ExpectToken(const std::string& file, const std::string& token) {
  EXPECT_NE(ReadProjectFile(file).find(token), std::string::npos)
      << token << " is missing from " << file;
}

void ExpectNoToken(const std::string& file, const std::string& token) {
  EXPECT_EQ(ReadProjectFile(file).find(token), std::string::npos)
      << token << " must not remain in " << file;
}

}  // namespace

TEST(RelocalizationEfficiencySurface, MatchersKeepUpdateStatePerInstance) {
  const std::string ivox = "include/registration/loam_point_to_plane_ivox.h";
  ExpectToken(ivox, "bool is_first_add_cloud_ = true;");
  ExpectNoToken(ivox, "static bool is_first");
  ExpectNoToken(ivox, "static Eigen::Matrix<Type, 5, 1> b");

  const std::string icp = "include/registration/icp_optimized.h";
  ExpectToken(icp, "last_add_cloud_T_");
  ExpectToken(icp, "has_last_add_cloud_T_");
  ExpectNoToken(icp, "static Eigen::Matrix<Type, 4, 4> last_T");

  const std::string kdtree =
      "include/registration/loam_point_to_plane_kdtree.h";
  ExpectToken(kdtree, "last_add_cloud_T_");
  ExpectToken(kdtree, "has_last_add_cloud_T_");
  ExpectNoToken(kdtree, "static Eigen::Matrix<Type, 4, 4> last_T");
  ExpectNoToken(kdtree, "static Eigen::Matrix<Type, 5, 1> b");
}

TEST(RelocalizationEfficiencySurface, ExposesHighConfidenceEarlyAccept) {
  ExpectToken("include/slam/config_parameters.h",
              "localization_sc_accept_fitness_");
  ExpectToken("src/slam/system.cpp", "localization.sc_accept_fitness");
  ExpectToken("config/relocalization.yaml", "sc_accept_fitness:");
  ExpectToken("src/slam/localization.cpp",
              "ShouldEarlyAcceptRelocalization");
}

TEST(RelocalizationEfficiencySurface, ExposesDedicatedGroundMap) {
  ExpectToken("include/slam/config_parameters.h",
              "localization_ground_map_path_");
  ExpectToken("src/slam/system.cpp", "localization.ground_map_path");
  ExpectToken("config/relocalization.yaml", "ground_map_path:");
  ExpectToken("include/slam/localization.h", "ground_map_cloud_ptr_");
  ExpectToken("include/slam/localization.h", "ground_map_xy_cloud_ptr_");
  ExpectToken("include/slam/localization.h", "ground_map_xy_kdtree_");
}
