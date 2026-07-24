#include <gtest/gtest.h>

#include <array>
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

void ExpectTokens(const std::array<std::string, 3>& files,
                  const std::array<std::string, 7>& tokens) {
  for (const auto& token : tokens) {
    for (const auto& file : files) {
      EXPECT_NE(ReadProjectFile(file).find(token), std::string::npos)
          << token << " is missing from " << file;
    }
  }
}

void ExpectToken(const std::string& file, const std::string& token) {
  EXPECT_NE(ReadProjectFile(file).find(token), std::string::npos)
      << token << " is missing from " << file;
}

void ExpectQuotedParameter(const std::string& file,
                           const std::string& parameter_name) {
  EXPECT_NE(ReadProjectFile(file).find("\"" + parameter_name + "\""),
            std::string::npos)
      << parameter_name << " is missing from " << file;
}

void ExpectMappingYamlParameter(const std::string& leaf) {
  const std::string text = ReadProjectFile("config/mapping.yaml");
  EXPECT_NE(text.find("\n      " + leaf + ":"), std::string::npos)
      << "map_save." << leaf << " is missing from config/mapping.yaml";
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

  for (const auto& parameter : canonical_parameters) {
    ExpectQuotedParameter("include/lio_sam/utility.hpp", parameter);
  }
  EXPECT_NE(ReadProjectFile("config/mapping.yaml").find("\n    map_save:"),
            std::string::npos)
      << "map_save is missing from config/mapping.yaml";
  for (const auto& leaf : yaml_leaves) {
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

  for (const auto& token : tokens) {
    ExpectToken("src/mapOptimization.cpp", token);
  }
}
