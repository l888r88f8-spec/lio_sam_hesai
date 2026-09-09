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

TEST(TiledMapLocalizationSurface, UsesProviderForEveryMapRequest) {
  ExpectToken("include/slam/localization.h",
              "std::unique_ptr<localization_map::MapProvider> map_provider_");
  ExpectToken("src/slam/localization.cpp", "ResolveMapMode");
  ExpectToken("src/slam/localization.cpp", "map_provider_->LoadCandidate");
  ExpectToken("src/slam/localization.cpp",
              "const auto initial_map = map_provider_->LoadCandidate");
  ExpectToken("src/slam/localization.cpp",
              "UpdateActiveMapSnapshot(initial_map.snapshot)");
  ExpectToken("src/slam/localization.cpp", "map_provider_->RequestTracking");
  ExpectToken("src/slam/localization.cpp", "MapRequestState::kPending");
  ExpectNoToken("src/slam/localization.cpp",
                "Scan Context auto relocalization does not support tile maps");
  ExpectNoToken("include/slam/localization.h", "bool use_tile_map_");
  ExpectNoToken("include/slam/localization.h", "local_map_data_");
}

TEST(TiledMapLocalizationSurface, ExportsTilesAndPublishesMapDiagnostics) {
  ExpectToken("src/mapOptimization.cpp", "saveTiledMap");
  ExpectToken("src/mapOptimization.cpp", "GenerateTileMap");
  const std::string system = "src/slam/system.cpp";
  for (const char* key : {
           "map_mode", "map_manifest_version", "map_active_tile_count",
           "map_cached_tile_count", "map_cache_mib", "map_cache_hits",
           "map_cache_misses", "map_tile_load_ms", "map_tile_load_failures",
           "map_tile_load_state", "map_tile_load_error"}) {
    ExpectToken(system, key);
  }
  ExpectToken("include/slam/system.h",
              "localization_local_cloud_map_pub_");
  ExpectToken(system, "lio_sam/localization/local_map");
  ExpectToken(system, "localization_ptr_->GetLocalCloudMap()");
  ExpectToken(system, "localization_ptr_->GetGlobalCloudMap()");
  ExpectToken("config/rviz2.rviz", "/lio_sam/localization/local_map");
}
