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
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

template<std::size_t FileCount, std::size_t TokenCount>
void ExpectTokens(
  const std::array<std::string, FileCount> & files,
  const std::array<std::string, TokenCount> & tokens)
{
  for (const auto & token : tokens) {
    for (const auto & file : files) {
      EXPECT_NE(ReadProjectFile(file).find(token), std::string::npos)
        << token << " is missing from " << file;
    }
  }
}

void ExpectToken(const std::string & file, const std::string & token)
{
  EXPECT_NE(ReadProjectFile(file).find(token), std::string::npos)
    << token << " is missing from " << file;
}

void ExpectQuotedParameter(
  const std::string & file,
  const std::string & parameter_name)
{
  EXPECT_NE(
    ReadProjectFile(file).find("\"" + parameter_name + "\""),
    std::string::npos)
    << parameter_name << " is missing from " << file;
}

std::size_t FindYamlBlockEnd(
  const std::string & text,
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

void ExpectMappingYamlParameter(const std::string & leaf)
{
  const std::string text = ReadProjectFile("config/mapping.yaml");
  const std::size_t map_save = text.find("\n    map_save:");
  ASSERT_NE(map_save, std::string::npos)
    << "map_save is missing from config/mapping.yaml";
  const std::size_t map_save_end = FindYamlBlockEnd(text, map_save, 4);
  const std::size_t yaml_leaf = text.find("\n      " + leaf + ":", map_save);
  EXPECT_TRUE(yaml_leaf != std::string::npos && yaml_leaf < map_save_end)
    << "map_save." << leaf << " is missing from config/mapping.yaml";
}

void ExpectLocalizationHealthYamlParameter(const std::string & leaf)
{
  const std::string text = ReadProjectFile("config/relocalization.yaml");
  const std::size_t health = text.find("\n    localization_health:");
  ASSERT_NE(health, std::string::npos)
    << "localization_health is missing from config/relocalization.yaml";
  const std::size_t health_end = FindYamlBlockEnd(text, health, 4);
  const std::size_t yaml_leaf = text.find("\n      " + leaf + ":", health);
  EXPECT_TRUE(yaml_leaf != std::string::npos && yaml_leaf < health_end)
    << "localization_health." << leaf
    << " is missing from config/relocalization.yaml";
}

}  // namespace

TEST(AutoRelocalizationParameterSurface, ExposesAllRuntimeParameters) {
  const std::array<std::string, 3> files = {
    "include/slam/config_parameters.h",
    "src/slam/system.cpp",
    "config/relocalization.yaml",
  };
  const std::array<std::string, 7> tokens = {
    "auto_relocalization",
    "sc_db_path",
    "sc_top_n",
    "sc_fitness_thres",
    "sc_consistency_frames",
    "sc_consistency_trans_thres",
    "sc_consistency_yaw_thres",
  };

  ExpectTokens(files, tokens);
}

TEST(AutoRelocalizationParameterSurface, ExposesMappingDatabaseExport) {
  const std::array<std::string, 3> canonical_parameters = {
    "map_save.generate_scan_context_db",
    "map_save.align_ground_to_zero",
    "map_save.ground_z_offset",
  };
  const std::array<std::string, 3> yaml_leaves = {
    "generate_scan_context_db",
    "align_ground_to_zero",
    "ground_z_offset",
  };

  for (const auto & parameter : canonical_parameters) {
    ExpectQuotedParameter("include/lio_sam/utility.hpp", parameter);
  }
  for (const auto & leaf : yaml_leaves) {
    ExpectMappingYamlParameter(leaf);
  }

  const std::array<std::string, 9> tokens = {
    "generateScDb",
    "relocalization/scan_context.h",
    "ScanContext",
    "sc_db.bin",
    "SaveDB",
    "alignSavedMapGroundToZero",
    "savedMapGroundZOffset",
    "pclPointToAffine3f",
    "keyFramePose(2, 3) += savedMapGroundZOffset",
  };

  for (const auto & token : tokens) {
    ExpectToken("src/mapOptimization.cpp", token);
  }
}

TEST(AutoRelocalizationParameterSurface, ExposesLocalizationHealthParameters) {
  struct HealthParameter {
    std::string leaf;
    std::string member;
  };
  const std::array<HealthParameter, 13> parameters = {{
    {"enable", "health_enable_"},
    {"degrade_enter_frames", "health_degrade_enter_frames_"},
    {"recover_frames", "health_recover_frames_"},
    {"lost_enter_frames", "health_lost_enter_frames_"},
    {"degraded_fitness_thres", "health_degraded_fitness_thres_"},
    {"lost_fitness_thres", "health_lost_fitness_thres_"},
    {"nominal_lidar_rate_hz", "health_nominal_lidar_rate_hz_"},
    {"nominal_imu_rate_hz", "health_nominal_imu_rate_hz_"},
    {"lidar_stall_timeout", "health_lidar_stall_timeout_"},
    {"imu_stall_timeout", "health_imu_stall_timeout_"},
    {"result_timeout", "health_result_timeout_"},
    {"base_covariance", "health_base_covariance_"},
    {"max_covariance_scale", "health_max_covariance_scale_"},
  }};

  for (const auto & parameter : parameters) {
    ExpectToken("include/slam/config_parameters.h", parameter.member);
    ExpectQuotedParameter(
      "src/slam/system.cpp", "localization_health." + parameter.leaf);
    ExpectLocalizationHealthYamlParameter(parameter.leaf);
  }
}
