#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {

std::filesystem::path ProjectPath(const std::string& relative_path) {
  return std::filesystem::path(PROJECT_SOURCE_DIR) / relative_path;
}

std::string ReadProjectFile(const std::string& relative_path) {
  std::ifstream file(ProjectPath(relative_path));
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

}  // namespace

TEST(CodeHygiene, PublicUtilityHeaderDoesNotImportStdNamespace) {
  const auto utility = ProjectPath("include/lio_sam/utility.hpp");
  ASSERT_TRUE(std::filesystem::exists(utility));
  EXPECT_EQ(
      ReadProjectFile("include/lio_sam/utility.hpp")
          .find("using namespace std;"),
      std::string::npos);
}

TEST(CodeHygiene, UnbuiltLegacySlamSourcesAreAbsent) {
  EXPECT_FALSE(std::filesystem::exists(
      ProjectPath("src/slam/frontend.cpp")));
  EXPECT_FALSE(std::filesystem::exists(
      ProjectPath("src/slam/loop_closure.cpp")));
}
