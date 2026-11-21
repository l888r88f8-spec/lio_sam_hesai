#include "common/dufomap_export.h"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include <unistd.h>

namespace
{

namespace fs = std::filesystem;
using lio_sam_hesai::DufomapFrameRecord;
using lio_sam_hesai::DufomapManifest;

class TempDirectory
{
 public:
  TempDirectory()
  {
    path_ = fs::temp_directory_path() /
            ("lio_sam_dufomap_contract_" + std::to_string(::getpid()) + "_" +
             std::to_string(counter_++));
    std::error_code error;
    fs::remove_all(path_, error);
    fs::create_directories(path_);
  }

  ~TempDirectory()
  {
    std::error_code error;
    fs::remove_all(path_, error);
  }

  [[nodiscard]] const fs::path& path() const { return path_; }

 private:
  static inline unsigned counter_{0};
  fs::path path_;
};

TEST(DufomapExportContract, DefaultConfigurationIsValid)
{
  DufomapExportConfig config;
  std::string error;
  EXPECT_TRUE(lio_sam_hesai::ValidateDufomapExportConfig(config, &error));
  EXPECT_TRUE(error.empty());
}

TEST(DufomapExportContract, RejectsInvalidParameters)
{
  for (const auto& mutate :
       std::vector<std::function<void(DufomapExportConfig&)>>{
           [](DufomapExportConfig& c) { c.resolution = 0.0; },
           [](DufomapExportConfig& c) { c.d_s = -0.1; },
           [](DufomapExportConfig& c) { c.d_p = -1; },
           [](DufomapExportConfig& c) { c.d_p = 5; },
           [](DufomapExportConfig& c) { c.num_threads = 1; },
           [](DufomapExportConfig& c) { c.num_threads = -1; },
           [](DufomapExportConfig& c) { c.min_range = -0.1; },
           [](DufomapExportConfig& c) { c.max_range = -1.0; },
           [](DufomapExportConfig& c) { c.timeout_seconds = -1.0; }}) {
    DufomapExportConfig config;
    mutate(config);
    std::string error;
    EXPECT_FALSE(lio_sam_hesai::ValidateDufomapExportConfig(config, &error));
    EXPECT_FALSE(error.empty());
  }
}

TEST(DufomapExportContract, RejectsCombiningWithLegacyStaticVoxelFilter)
{
  StaticMapFilterConfig legacy;
  legacy.enable = true;
  legacy.voxel_size = 0.4;
  legacy.min_keyframes = 3;
  std::string error;
  EXPECT_FALSE(lio_sam_hesai::ValidateDufomapFilterCompatibility(
      true, legacy, &error));
  EXPECT_NE(error.find("static_map_filter"), std::string::npos);

  // A false request remains valid even when the legacy filter is enabled.
  EXPECT_TRUE(lio_sam_hesai::ValidateDufomapFilterCompatibility(
      false, legacy, &error));

  legacy.enable = false;
  EXPECT_TRUE(lio_sam_hesai::ValidateDufomapFilterCompatibility(
      true, legacy, &error));
}

TEST(DufomapExportContract, ManifestCarriesFinalPoseAndOriginalTargets)
{
  DufomapManifest manifest;
  manifest.parameters.resolution = 0.1;
  manifest.parameters.d_s = 0.2;
  manifest.parameters.d_p = 2;
  manifest.backend = "native-cpp-upstream-core";
  manifest.backend_source =
      "KTH-RPL/dufomap@9e239ddd5995136e14f5212f33382a6ebc59e518";
  manifest.ufomap_revision = "fcb20f445d257280c039c07ab756c9d5b37ebd78";
  DufomapFrameRecord frame;
  frame.index = 7;
  frame.scan = "scans/000007.pcd";
  frame.pose = {1.0, 0.0, 0.0, 1.5,
                0.0, 1.0, 0.0, -2.0,
                0.0, 0.0, 1.0, 3.0,
                0.0, 0.0, 0.0, 1.0};
  manifest.frames.push_back(frame);
  manifest.targets.push_back({"corner", "targets/corner_raw.pcd",
                              "corner.pcd"});

  const auto parsed = YAML::Load(
      lio_sam_hesai::SerializeDufomapManifest(manifest));
  EXPECT_EQ(parsed["format"].as<std::string>(), "lio_sam_dufomap_export");
  EXPECT_EQ(parsed["version"].as<int>(), 1);
  EXPECT_EQ(parsed["backend"].as<std::string>(), "native-cpp-upstream-core");
  EXPECT_EQ(parsed["ufomap_revision"].as<std::string>(),
            "fcb20f445d257280c039c07ab756c9d5b37ebd78");
  ASSERT_EQ(parsed["frames"].size(), 1U);
  EXPECT_EQ(parsed["frames"][0]["scan"].as<std::string>(), "scans/000007.pcd");
  EXPECT_DOUBLE_EQ(parsed["frames"][0]["pose"][3].as<double>(), 1.5);
  EXPECT_DOUBLE_EQ(parsed["frames"][0]["pose"][7].as<double>(), -2.0);
  ASSERT_EQ(parsed["targets"].size(), 1U);
  EXPECT_EQ(parsed["targets"][0]["output"].as<std::string>(), "corner.pcd");
  EXPECT_DOUBLE_EQ(parsed["parameters"]["resolution"].as<double>(), 0.1);
}

TEST(DufomapExportContract, MakesFreshSessionDirectoryBelowConfiguredBase)
{
  TempDirectory temp;
  std::string error;
  const fs::path session = lio_sam_hesai::MakeDufomapSessionDirectory(
      temp.path().string(), ::getpid(), &error);
  ASSERT_FALSE(session.empty()) << error;
  EXPECT_TRUE(session.string().rfind(temp.path().string(), 0) == 0);
  EXPECT_TRUE(fs::is_directory(session));
  EXPECT_TRUE(fs::is_directory(session / "scans"));

  const fs::path second = lio_sam_hesai::MakeDufomapSessionDirectory(
      temp.path().string(), ::getpid(), &error);
  ASSERT_FALSE(second.empty()) << error;
  EXPECT_NE(session, second);
}

TEST(DufomapExportContract, EmptyBaseFallsBackToSystemTempDirectory)
{
  std::string error;
  const fs::path session = lio_sam_hesai::MakeDufomapSessionDirectory(
      "", ::getpid(), &error);
  ASSERT_FALSE(session.empty()) << error;
  EXPECT_TRUE(fs::equivalent(session.parent_path(),
                             fs::temp_directory_path()));
  EXPECT_TRUE(fs::is_directory(session / "scans"));
  std::error_code cleanup_error;
  fs::remove_all(session, cleanup_error);
}

}  // namespace
