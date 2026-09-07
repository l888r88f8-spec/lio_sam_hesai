#include <gtest/gtest.h>

#include <chrono>
#include <array>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <stdexcept>
#include <string>

#include "localization_map/tile_manifest.h"

namespace {

namespace fs = std::filesystem;
using localization_map::TileIndex;
using localization_map::TileManifest;

#define EXPECT_RUNTIME_ERROR_CONTAINING(statement, fragment)                 \
  do {                                                                        \
    try {                                                                     \
      statement;                                                              \
      FAIL() << "expected std::runtime_error containing: " << fragment;     \
    } catch (const std::runtime_error& error) {                              \
      EXPECT_NE(std::string(error.what()).find(fragment), std::string::npos) \
          << error.what();                                                    \
    }                                                                         \
  } while (false)

class TemporaryMapDirectory {
 public:
  TemporaryMapDirectory() {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = fs::temp_directory_path() / ("tile_manifest_" + std::to_string(stamp));
    fs::create_directories(root_);
  }

  ~TemporaryMapDirectory() { fs::remove_all(root_); }

  const fs::path& root() const { return root_; }

  void Write(const fs::path& relative_path, const std::string& content) const {
    fs::create_directories((root_ / relative_path).parent_path());
    std::ofstream output(root_ / relative_path, std::ios::binary);
    output << content;
  }

  void WriteManifest(const std::string& text) const { Write("manifest.yaml", text); }

  std::string Read(const fs::path& relative_path) const {
    std::ifstream input(root_ / relative_path, std::ios::binary);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  }

 private:
  fs::path root_;
};

std::string ValidManifest(const std::string& global_path = "global/-1_0.pcd",
                          const std::string& global_bytes = "2") {
  return "schema_version: 1\n"
         "frame_id: \"map\"\n"
         "point_type: \"XYZI\"\n"
         "tile_size_m: 100.0\n"
         "origin_xy: [0.0, 0.0]\n"
         "source:\n"
         "  global_map_name: \"GlobalMap.pcd\"\n"
         "  global_map_bytes: 2855470\n"
         "  ground_map_name: \"GroundMap.pcd\"\n"
         "  ground_map_bytes: 1367788\n"
         "tiles:\n"
         "  global:\n"
         "    - index: [-1, 0]\n"
         "      path: \"" + global_path + "\"\n"
         "      point_count: 2\n"
         "      bytes: " + global_bytes + "\n"
         "  ground:\n"
         "    - index: [0, 0]\n"
         "      path: \"ground/0_0.pcd\"\n"
         "      point_count: 3\n"
         "      bytes: 3\n"
         "overview:\n"
         "  path: \"overview.pcd\"\n"
         "  resolution_m: 5.0\n";
}

void PopulateValidMap(const TemporaryMapDirectory& directory) {
  directory.Write("global/-1_0.pcd", "ab");
  directory.Write("ground/0_0.pcd", "xyz");
  directory.Write("overview.pcd", "overview");
  directory.WriteManifest(ValidManifest());
}

TEST(TileManifest, LoadsValidVersionOneManifest) {
  const TemporaryMapDirectory directory;
  PopulateValidMap(directory);

  const TileManifest manifest = TileManifest::Load(directory.root());

  EXPECT_EQ(manifest.schema_version, 1);
  EXPECT_EQ(manifest.frame_id, "map");
  EXPECT_DOUBLE_EQ(manifest.tile_size_m, 100.0);
  EXPECT_EQ(manifest.origin, (std::array<double, 2>{0.0, 0.0}));
  EXPECT_EQ(manifest.global_tiles.size(), 1U);
  EXPECT_EQ(manifest.global_tiles.at(TileIndex{-1, 0}).point_count, 2U);
  ASSERT_TRUE(manifest.overview.has_value());
  EXPECT_EQ(manifest.overview->path, fs::path("overview.pcd"));
}

TEST(TileManifest, RejectsManifestSymlinkWithoutReadingTarget) {
  TemporaryMapDirectory directory;
  TemporaryMapDirectory outside;
  PopulateValidMap(directory);
  outside.WriteManifest(ValidManifest());
  fs::remove(directory.root() / "manifest.yaml");
  fs::create_symlink(outside.root() / "manifest.yaml",
                     directory.root() / "manifest.yaml");

  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()),
                                  "manifest.yaml must be a regular file");
}

TEST(TileManifest, RejectsNonRegularManifestWithoutBlocking) {
  TemporaryMapDirectory directory;
  PopulateValidMap(directory);
  fs::remove(directory.root() / "manifest.yaml");
  fs::create_directory(directory.root() / "manifest.yaml");

  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()),
                                  "manifest.yaml must be a regular file");
}

TEST(TileManifest, IgnoresUnknownOptionalFields) {
  TemporaryMapDirectory directory;
  PopulateValidMap(directory);
  directory.WriteManifest(ValidManifest() + "future_optional: true\n");

  EXPECT_NO_THROW(TileManifest::Load(directory.root()));
}

TEST(TileManifest, RejectsUnsupportedSchemaVersion) {
  TemporaryMapDirectory directory;
  PopulateValidMap(directory);
  directory.WriteManifest(ValidManifest().replace(16, 1, "2"));
  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()),
                                  "unsupported schema_version");
}

TEST(TileManifest, RejectsDuplicateGlobalTileIndex) {
  TemporaryMapDirectory directory;
  PopulateValidMap(directory);
  std::string manifest = ValidManifest();
  manifest.replace(manifest.find("  ground:"), 0,
                   "    - index: [-1, 0]\n"
                   "      path: \"global/-1_0.pcd\"\n"
                   "      point_count: 2\n"
                   "      bytes: 2\n");
  directory.WriteManifest(manifest);
  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()),
                                  "duplicate global tile index");
}

TEST(TileManifest, RejectsDuplicateGroundTileIndex) {
  TemporaryMapDirectory directory;
  PopulateValidMap(directory);
  std::string manifest = ValidManifest();
  manifest.replace(manifest.find("overview:"), 0,
                   "    - index: [0, 0]\n"
                   "      path: \"ground/0_0.pcd\"\n"
                   "      point_count: 3\n"
                   "      bytes: 3\n");
  directory.WriteManifest(manifest);
  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()),
                                  "duplicate ground tile index");
}

TEST(TileManifest, RejectsPathThatEscapesMapRoot) {
  TemporaryMapDirectory directory;
  PopulateValidMap(directory);
  directory.WriteManifest(ValidManifest("../outside.pcd"));
  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()),
                                  "escapes tiled map root");
}

TEST(TileManifest, RejectsSymlinkPathThatEscapesMapRoot) {
  TemporaryMapDirectory directory;
  PopulateValidMap(directory);
  fs::remove(directory.root() / "global/-1_0.pcd");
  fs::create_symlink(directory.root().parent_path(),
                     directory.root() / "global/-1_0.pcd");

  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()),
                                  "escapes tiled map root");
}

TEST(TileManifest, RejectsAbsoluteAndEmptyTilePaths) {
  TemporaryMapDirectory directory;
  PopulateValidMap(directory);
  directory.WriteManifest(ValidManifest("/tmp/tile.pcd"));
  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()),
                                  "must be relative");
  directory.WriteManifest(ValidManifest(""));
  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()),
                                  "must be nonempty");
}

TEST(TileManifest, RejectsMissingTile) {
  TemporaryMapDirectory directory;
  PopulateValidMap(directory);
  directory.WriteManifest(ValidManifest("global/missing.pcd"));
  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()),
                                  "tile file is missing");
}

TEST(TileManifest, ValidatesAllPathsBeforeCheckingTileExistence) {
  TemporaryMapDirectory directory;
  PopulateValidMap(directory);
  std::string manifest = ValidManifest("global/missing.pcd");
  manifest.replace(manifest.find("  ground:"), 0,
                   "    - index: [0, 0]\n"
                   "      path: \"../outside.pcd\"\n"
                   "      point_count: 0\n"
                   "      bytes: 0\n");
  directory.WriteManifest(manifest);

  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()),
                                  "escapes tiled map root");
}

TEST(TileManifest, RejectsDeclaredTileByteMismatch) {
  TemporaryMapDirectory directory;
  PopulateValidMap(directory);
  directory.WriteManifest(ValidManifest("global/-1_0.pcd", "9"));
  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()),
                                  "tile byte size mismatch");
}

TEST(TileManifest, RejectsInvalidScalarFields) {
  TemporaryMapDirectory directory;
  PopulateValidMap(directory);
  std::string manifest = ValidManifest();
  manifest.replace(manifest.find("XYZI"), 4, "XYZ");
  directory.WriteManifest(manifest);
  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()), "point_type");

  manifest = ValidManifest();
  manifest.replace(manifest.find("100.0"), 5, "0");
  directory.WriteManifest(manifest);
  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()), "tile_size_m");

  manifest = ValidManifest();
  manifest.replace(manifest.find("100.0"), 5, ".nan");
  directory.WriteManifest(manifest);
  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()), "tile_size_m");
}

TEST(TileManifest, RejectsInvalidOriginAndOverviewResolution) {
  TemporaryMapDirectory directory;
  PopulateValidMap(directory);
  std::string manifest = ValidManifest();
  manifest.replace(manifest.find("[0.0, 0.0]"), 10, "[0.0]");
  directory.WriteManifest(manifest);
  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()), "origin_xy");

  manifest = ValidManifest();
  manifest.replace(manifest.rfind("5.0"), 3, "0");
  directory.WriteManifest(manifest);
  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()), "resolution_m");

  manifest = ValidManifest();
  manifest.replace(manifest.rfind("5.0"), 3, ".nan");
  directory.WriteManifest(manifest);
  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()), "resolution_m");

  manifest = ValidManifest();
  manifest.replace(manifest.find("[0.0, 0.0]"), 10, "[.nan, 0.0]");
  directory.WriteManifest(manifest);
  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()), "origin_xy");
}

TEST(TileManifest, RejectsMalformedScalarValues) {
  TemporaryMapDirectory directory;
  PopulateValidMap(directory);
  std::string manifest = ValidManifest();
  manifest.replace(manifest.find("point_count: 2") + 13, 1, "wrong");
  directory.WriteManifest(manifest);
  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()), "point_count");
}

TEST(TileManifest, RejectsNonRegularTileFile) {
  TemporaryMapDirectory directory;
  PopulateValidMap(directory);
  fs::remove(directory.root() / "global/-1_0.pcd");
  fs::create_directory(directory.root() / "global/-1_0.pcd");
  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()),
                                  "tile file is not regular");
}

TEST(TileManifest, RejectsMissingOverviewFile) {
  TemporaryMapDirectory directory;
  PopulateValidMap(directory);
  fs::remove(directory.root() / "overview.pcd");

  EXPECT_RUNTIME_ERROR_CONTAINING(TileManifest::Load(directory.root()),
                                  "overview file is missing");
}

TEST(TileManifest, SavesManifestThatRoundTripsThroughLoad) {
  TemporaryMapDirectory directory;
  PopulateValidMap(directory);
  const TileManifest original = TileManifest::Load(directory.root());

  original.Save(directory.root());

  const TileManifest reloaded = TileManifest::Load(directory.root());
  EXPECT_EQ(reloaded.global_tiles.at(TileIndex{-1, 0}).bytes, 2U);
  ASSERT_TRUE(reloaded.overview.has_value());
  EXPECT_EQ(reloaded.overview->path, fs::path("overview.pcd"));
}

TEST(TileManifest, SaveDoesNotFollowExistingManifestSymlink) {
  TemporaryMapDirectory directory;
  TemporaryMapDirectory outside;
  PopulateValidMap(directory);
  outside.Write("sentinel.yaml", "outside sentinel\n");
  const TileManifest manifest = TileManifest::Load(directory.root());
  fs::remove(directory.root() / "manifest.yaml");
  fs::create_symlink(outside.root() / "sentinel.yaml",
                     directory.root() / "manifest.yaml");

  manifest.Save(directory.root());

  EXPECT_EQ(outside.Read("sentinel.yaml"), "outside sentinel\n");
  EXPECT_FALSE(fs::is_symlink(directory.root() / "manifest.yaml"));
}

TEST(TileManifest, SaveIsDeterministicAndUsesNaturalTileOrder) {
  TemporaryMapDirectory directory;
  PopulateValidMap(directory);
  directory.Write("global/-2_0.pcd", "c");
  std::string text = ValidManifest();
  text.replace(text.find("  ground:"), 0,
               "    - index: [-2, 0]\n"
               "      path: \"global/-2_0.pcd\"\n"
               "      point_count: 1\n"
               "      bytes: 1\n");
  directory.WriteManifest(text);
  const TileManifest manifest = TileManifest::Load(directory.root());

  manifest.Save(directory.root());
  const std::string first = directory.Read("manifest.yaml");
  manifest.Save(directory.root());
  const std::string second = directory.Read("manifest.yaml");

  EXPECT_EQ(first, second);
  EXPECT_LT(first.find("index: [-2, 0]"), first.find("index: [-1, 0]"));
}

}  // namespace
