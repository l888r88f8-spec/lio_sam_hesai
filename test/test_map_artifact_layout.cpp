#include "common/map_artifact_layout.h"

#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include <unistd.h>

namespace {

namespace fs = std::filesystem;

// Restores the process working directory when it goes out of scope, so a
// relative-path test cannot leak its cwd into the rest of the binary.
class ScopedCurrentPath {
 public:
  explicit ScopedCurrentPath(const fs::path& path)
      : previous_(fs::current_path()) {
    fs::current_path(path);
  }
  ~ScopedCurrentPath() { fs::current_path(previous_); }
  ScopedCurrentPath(const ScopedCurrentPath&) = delete;
  ScopedCurrentPath& operator=(const ScopedCurrentPath&) = delete;

 private:
  fs::path previous_;
};

}  // namespace

TEST(MapArtifactLayout, ResolvesEveryArtifactFromTheRoot)
{
  const MapArtifactLayout layout("/maps/site");

  EXPECT_EQ(layout.Root(), "/maps/site");
  EXPECT_EQ(layout.GlobalMap(), "/maps/site/GlobalMap.pcd");
  EXPECT_EQ(layout.GroundMap(), "/maps/site/GroundMap.pcd");
  EXPECT_EQ(layout.ScanContextDb(), "/maps/site/sc_db.bin");
  EXPECT_EQ(layout.Trajectory(), "/maps/site/trajectory.pcd");
  EXPECT_EQ(layout.Transformations(), "/maps/site/transformations.pcd");
  EXPECT_EQ(layout.CornerMap(), "/maps/site/CornerMap.pcd");
  EXPECT_EQ(layout.SurfaceMap(), "/maps/site/SurfMap.pcd");
  EXPECT_EQ(layout.TiledRoot(), "/maps/site/tiled");
  EXPECT_EQ(layout.TiledManifest(), "/maps/site/tiled/manifest.yaml");
}

TEST(MapArtifactLayout, HonoursAConfiguredTileSubdirectory)
{
  const MapArtifactLayout layout("/maps/site", "tiles_v2");

  EXPECT_EQ(layout.TiledRoot(), "/maps/site/tiles_v2");
  EXPECT_EQ(layout.TiledManifest(), "/maps/site/tiles_v2/manifest.yaml");
  // Only the tiled artifacts move; the root artifacts do not.
  EXPECT_EQ(layout.GlobalMap(), "/maps/site/GlobalMap.pcd");
}

TEST(MapArtifactLayout, DerivesTheRootFromAGlobalMapPath)
{
  const MapArtifactLayout layout =
    MapArtifactLayout::FromGlobalMap("/maps/site/GlobalMap.pcd");

  EXPECT_EQ(layout.Root(), "/maps/site");
  EXPECT_EQ(layout.GroundMap(), "/maps/site/GroundMap.pcd");
  EXPECT_EQ(layout.ScanContextDb(), "/maps/site/sc_db.bin");
}

TEST(MapArtifactLayout, DerivesTheRootFromATiledRoot)
{
  const MapArtifactLayout layout =
    MapArtifactLayout::FromTiledRoot("/maps/site/tiled");

  EXPECT_EQ(layout.Root(), "/maps/site");
  EXPECT_EQ(layout.TiledRoot(), "/maps/site/tiled");
  EXPECT_EQ(layout.TiledManifest(), "/maps/site/tiled/manifest.yaml");
}

TEST(MapArtifactLayout, KeepsANonDefaultTileBasename)
{
  // The configured basename is validated, not forced to "tiled".
  const MapArtifactLayout layout =
    MapArtifactLayout::FromTiledRoot("/maps/site/tiles_v2");

  EXPECT_EQ(layout.Root(), "/maps/site");
  EXPECT_EQ(layout.TiledManifest(), "/maps/site/tiles_v2/manifest.yaml");
}

TEST(MapArtifactLayout, ValidatesTheConfiguredTileBasename)
{
  EXPECT_NO_THROW(
    MapArtifactLayout::FromTiledRoot("/maps/site/tiled", "tiled"));
  EXPECT_THROW(
    MapArtifactLayout::FromTiledRoot("/maps/site/tiled", "tiles_v2"),
    std::invalid_argument);
}

TEST(MapArtifactLayout, RejectsEmptyPaths)
{
  EXPECT_THROW(MapArtifactLayout(""), std::invalid_argument);
  EXPECT_THROW(MapArtifactLayout::FromGlobalMap(""), std::invalid_argument);
  EXPECT_THROW(MapArtifactLayout::FromTiledRoot(""), std::invalid_argument);
}

TEST(MapArtifactLayout, RejectsAForeignGlobalMapFileName)
{
  // Rejection happens before any I/O touches a wrongly named file.
  EXPECT_THROW(
    MapArtifactLayout::FromGlobalMap("/maps/site/other.pcd"),
    std::invalid_argument);
  EXPECT_THROW(
    MapArtifactLayout::FromGlobalMap("/maps/site/map.pcd"),
    std::invalid_argument);
}

TEST(MapArtifactLayout, NormalizesTheStoredRoot)
{
  EXPECT_EQ(MapArtifactLayout("/maps/./site").Root(), "/maps/site");
  EXPECT_EQ(MapArtifactLayout("/maps/site").TiledManifest(),
            fs::path("/maps/site") / "tiled" / "manifest.yaml");
}

TEST(MapArtifactLayout, AcceptsRootlessRelativePaths)
{
  // A bare relative file name has no parent path. The root is then the current
  // directory, not an empty -- and therefore unusable -- path.
  const MapArtifactLayout global =
    MapArtifactLayout::FromGlobalMap("GlobalMap.pcd");

  EXPECT_EQ(global.Root(), ".");
  EXPECT_EQ(global.GlobalMap(), "./GlobalMap.pcd");
  EXPECT_EQ(global.GroundMap(), "./GroundMap.pcd");
  EXPECT_EQ(global.ScanContextDb(), "./sc_db.bin");
}

TEST(MapArtifactLayout, AcceptsExplicitCurrentDirectoryPrefix)
{
  const MapArtifactLayout global =
    MapArtifactLayout::FromGlobalMap("./GlobalMap.pcd");

  EXPECT_EQ(global.Root(), ".");
  EXPECT_EQ(global.GlobalMap(), "./GlobalMap.pcd");
}

TEST(MapArtifactLayout, AcceptsRootlessRelativeTiledRoot)
{
  const MapArtifactLayout tiled = MapArtifactLayout::FromTiledRoot("tiled");

  EXPECT_EQ(tiled.Root(), ".");
  EXPECT_EQ(tiled.TiledRoot(), "./tiled");
  EXPECT_EQ(tiled.TiledManifest(), "./tiled/manifest.yaml");
}

TEST(MapArtifactLayout, AcceptsRelativeTiledRootSpellings)
{
  const std::array<std::string, 3> spellings = {
    "tiled", "tiled/", "./tiled",
  };
  for (const auto & spelling : spellings) {
    const MapArtifactLayout tiled =
      MapArtifactLayout::FromTiledRoot(spelling);
    EXPECT_EQ(tiled.Root(), ".") << spelling;
    EXPECT_EQ(tiled.TiledManifest(), "./tiled/manifest.yaml") << spelling;
  }

  const MapArtifactLayout renamed =
    MapArtifactLayout::FromTiledRoot("./tiles_v2");
  EXPECT_EQ(renamed.Root(), ".");
  EXPECT_EQ(renamed.TiledManifest(), "./tiles_v2/manifest.yaml");
}

TEST(MapArtifactLayout, KeepsRelativeParentDirectories)
{
  const MapArtifactLayout global =
    MapArtifactLayout::FromGlobalMap("maps/site/GlobalMap.pcd");
  EXPECT_EQ(global.Root(), "maps/site");
  EXPECT_EQ(global.GroundMap(), "maps/site/GroundMap.pcd");

  const MapArtifactLayout tiled =
    MapArtifactLayout::FromTiledRoot("maps/site/tiled");
  EXPECT_EQ(tiled.Root(), "maps/site");
  EXPECT_EQ(tiled.TiledManifest(), "maps/site/tiled/manifest.yaml");
  EXPECT_EQ(tiled.ScanContextDb(), "maps/site/sc_db.bin");
}

TEST(MapArtifactLayout, StillRejectsEmptyInputs)
{
  EXPECT_THROW(MapArtifactLayout(""), std::invalid_argument);
  EXPECT_THROW(MapArtifactLayout::FromGlobalMap(""), std::invalid_argument);
  EXPECT_THROW(MapArtifactLayout::FromTiledRoot(""), std::invalid_argument);
}

namespace {

// A minimal but genuinely loadable PCD, so the test proves the derived paths
// land on real map artifacts rather than on files that merely exist.
void WriteValidPcd(const fs::path& path) {
  std::ofstream out(path);
  out << "# .PCD v0.7 - Point Cloud Data file format\n"
         "VERSION 0.7\n"
         "FIELDS x y z intensity\n"
         "SIZE 4 4 4 4\n"
         "TYPE F F F F\n"
         "COUNT 1 1 1 1\n"
         "WIDTH 1\n"
         "HEIGHT 1\n"
         "VIEWPOINT 0 0 0 1 0 0 0\n"
         "POINTS 1\n"
         "DATA ascii\n"
         "1.0 2.0 3.0 4.0\n";
}

}  // namespace

TEST(MapArtifactLayout, ResolvesRelativeSpellingsToTheSameRealArtifacts)
{
  // String comparison alone cannot show that a configured map path resolves to
  // the right files, and the Scan Context and ground paths are derived through
  // RootDirectoryOf() -- the entry point that used to disagree with
  // FromTiledRoot() about a trailing separator.
  const fs::path directory =
    fs::temp_directory_path() /
    ("lio_sam_map_layout_" + std::to_string(::getpid()));
  fs::remove_all(directory);
  fs::create_directories(directory / "tiled");
  WriteValidPcd(directory / "GlobalMap.pcd");
  WriteValidPcd(directory / "GroundMap.pcd");
  WriteValidPcd(directory / "sc_db.bin");
  WriteValidPcd(directory / "tiled" / "overview.pcd");
  {
    std::ofstream manifest(directory / "tiled" / "manifest.yaml");
    manifest << "schema_version: 1\n"
                "frame_id: map\n"
                "point_type: XYZI\n"
                "tile_size_m: 15.0\n"
                "origin_xy: [0.0, 0.0]\n"
                "source:\n"
                "  global_map_name: GlobalMap.pcd\n"
                "  global_map_bytes: 0\n"
                "  ground_map_name: GroundMap.pcd\n"
                "  ground_map_bytes: 0\n"
                "tiles:\n"
                "  global: []\n"
                "  ground: []\n";
  }

  const ScopedCurrentPath scoped(directory);
  const fs::path expected_sc = fs::path(".") / "sc_db.bin";
  const fs::path expected_ground = fs::path(".") / "GroundMap.pcd";

  // Every spelling of the tiled root must derive the same artifacts, and the
  // derived paths must be the files that are actually on disk.
  const std::array<std::string, 4> spellings = {
    "tiled", "tiled/", "./tiled", "tiled//",
  };
  for (const auto & spelling : spellings) {
    const MapArtifactLayout derived(
        MapArtifactLayout::RootDirectoryOf(spelling));
    EXPECT_EQ(derived.Root(), ".") << spelling;
    EXPECT_EQ(derived.ScanContextDb(), expected_sc) << spelling;
    EXPECT_EQ(derived.GroundMap(), expected_ground) << spelling;
    EXPECT_TRUE(fs::is_regular_file(derived.ScanContextDb())) << spelling;
    EXPECT_TRUE(fs::is_regular_file(derived.GroundMap())) << spelling;

    const MapArtifactLayout parsed =
        MapArtifactLayout::FromTiledRoot(spelling);
    EXPECT_EQ(parsed.TiledManifest(), derived.TiledRoot() / "manifest.yaml")
      << spelling;
    EXPECT_TRUE(fs::is_regular_file(parsed.TiledManifest())) << spelling;
  }

  // The global-map spelling derives the same root artifacts.
  const MapArtifactLayout global =
    MapArtifactLayout::FromGlobalMap("GlobalMap.pcd");
  EXPECT_TRUE(fs::is_regular_file(global.GlobalMap()));
  EXPECT_EQ(global.ScanContextDb(), expected_sc);
  EXPECT_EQ(global.GroundMap(), expected_ground);

  fs::remove_all(directory);
}
