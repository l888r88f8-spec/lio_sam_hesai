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
  ExpectToken("include/common/slam_config.h", "sc_accept_fitness");
  ExpectToken("src/common/slam_config.cpp", "\"localization.sc_accept_fitness\"");
  ExpectToken("config/relocalization.yaml", "sc_accept_fitness:");
  ExpectToken("src/slam/localization.cpp",
              "ShouldEarlyAcceptRelocalization");
  ExpectToken("src/slam/localization.cpp",
              "config_->localization.localization.sc_accept_fitness");
}

TEST(RelocalizationEfficiencySurface, ExposesDedicatedGroundMap) {
  ExpectToken("include/common/slam_config.h", "ground_map_path");
  ExpectToken("src/common/slam_config.cpp", "\"localization.ground_map_path\"");
  ExpectToken("src/slam/localization.cpp", "map_config.ground_map_path");
  ExpectToken("config/relocalization.yaml", "ground_map_path:");
  ExpectToken("src/slam/localization.cpp", "provider_config.ground_map_path");
  ExpectToken("src/localization_map/global_map_provider.cpp",
              "impl_->ground_tree");
  ExpectToken("include/slam/localization.h", "active_ground_xy_kdtree_");
}

TEST(RelocalizationEfficiencySurface, CandidateLocalMapUsesFiftyMeterExtent) {
  ExpectToken("src/localization_map/global_map_provider.cpp",
              "constexpr double kCandidateHalfExtent = 50.0;");
  ExpectNoToken("src/localization_map/global_map_provider.cpp",
                "constexpr double kCandidateHalfExtent = 100.0;");
}

TEST(RelocalizationEfficiencySurface,
     EvaluatesCandidatesSeriallyWithEarlyExit) {
  const std::string localization =
      ReadProjectFile("src/slam/localization.cpp");
  EXPECT_EQ(localization.find("RunBoundedAsync("), std::string::npos);

  const std::size_t ordered_loop = localization.find(
      "for (std::size_t candidate_index = 0;");
  ASSERT_NE(ordered_loop, std::string::npos);

  const std::size_t isolated_preparation = localization.find(
      "try {\n      Timer preparation_timer;", ordered_loop);
  const std::size_t lazy_local_map =
      localization.find("map_provider_->LoadCandidate(", isolated_preparation);
  const std::size_t serial_match =
      localization.find("matcher->Match(", lazy_local_map);
  const std::size_t early_accept = localization.find(
      "ShouldEarlyAcceptRelocalization(", serial_match);
  const std::size_t fitness_stop = localization.find(
      "stop_reason = \"fitness\";", early_accept);
  const std::size_t isolated_catch =
      localization.find("} catch (const std::exception& error)", fitness_stop);
  const std::size_t candidate_summary = localization.find(
      "LOG(INFO) << \"Auto relocalization prepared \"", isolated_catch);

  ASSERT_NE(isolated_preparation, std::string::npos);
  ASSERT_NE(lazy_local_map, std::string::npos);
  ASSERT_NE(serial_match, std::string::npos);
  ASSERT_NE(early_accept, std::string::npos);
  ASSERT_NE(fitness_stop, std::string::npos);
  ASSERT_NE(isolated_catch, std::string::npos);
  EXPECT_NE(candidate_summary, std::string::npos);
  EXPECT_NE(localization.find("candidate_index + 1U", ordered_loop),
            std::string::npos);
}
