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
  ExpectToken("include/lio_sam/utility.hpp", "generateScDb");
  ExpectToken("config/mapping.yaml", "generateScDb");

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
