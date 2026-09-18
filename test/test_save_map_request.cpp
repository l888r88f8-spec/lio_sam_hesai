// Focused contract tests for the shared save-map request/export options.
//
// Both /lio_sam/save_map (legacy, HOME-relative destination) and
// /lio_sam/save_map_options (absolute destination, per-save toggles) resolve
// their request into the same lio_sam_hesai::SaveMapExportOptions struct before
// the mapOptimization export body touches the filesystem. These tests pin the
// path resolution, the unsafe-destination rejection, and the artifact cleanup
// rules without needing a running mapping node.

#include "common/map_artifact_layout.h"
#include "common/save_map_request.h"
#include "lio_sam_hesai/srv/save_map.hpp"
#include "lio_sam_hesai/srv/save_map_options.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <string>

#include <unistd.h>

namespace {

namespace fs = std::filesystem;

fs::path UniqueTempRoot()
{
  static std::atomic<unsigned> counter{0};
  return fs::temp_directory_path() /
         ("lio_sam_save_map_request_" + std::to_string(::getpid()) + "_" +
          std::to_string(counter.fetch_add(1)));
}

// Owns a fresh directory below the system temp directory and removes it again,
// so a failing assertion cannot leak state into later test binaries.
class TempDirectory
{
 public:
  TempDirectory() : path_(UniqueTempRoot())
  {
    std::error_code error;
    fs::remove_all(path_, error);
    fs::create_directories(path_);
  }

  ~TempDirectory()
  {
    std::error_code error;
    fs::remove_all(path_, error);
  }

  TempDirectory(const TempDirectory&) = delete;
  TempDirectory& operator=(const TempDirectory&) = delete;

  [[nodiscard]] const fs::path& path() const { return path_; }

 private:
  fs::path path_;
};

void WriteFile(const fs::path& path)
{
  fs::create_directories(path.parent_path());
  std::ofstream file(path);
  file << "map artifact\n";
}

}  // namespace

TEST(SaveMapRequest, LegacyDestinationIsHomeRelative)
{
  std::string error;
  const auto options = lio_sam_hesai::MakeLegacySaveMapExportOptions(
      "/map/site", 0.25F, 0.1F, "/home/robot", "/maps/default",
      /*configured_save_ground_map=*/true,
      /*configured_save_tiled_map=*/true, &error);

  ASSERT_TRUE(options.has_value()) << error;
  EXPECT_EQ(options->directory, fs::path("/home/robot/map/site"));
  EXPECT_FLOAT_EQ(options->resolution, 0.25F);
  EXPECT_FLOAT_EQ(options->ground_resolution, 0.1F);
  EXPECT_TRUE(options->save_ground_map);
  EXPECT_TRUE(options->save_tiled_map);
  EXPECT_TRUE(error.empty());
}

TEST(SaveMapRequest, LegacyEmptyDestinationFallsBackToConfiguredDirectory)
{
  std::string error;
  const auto options = lio_sam_hesai::MakeLegacySaveMapExportOptions(
      "", 0.0F, 0.0F, "/home/robot", "/maps/default",
      /*configured_save_ground_map=*/false,
      /*configured_save_tiled_map=*/false, &error);

  ASSERT_TRUE(options.has_value()) << error;
  EXPECT_EQ(options->directory, fs::path("/home/robot/maps/default"));
  EXPECT_FALSE(options->save_ground_map);
  EXPECT_FALSE(options->save_tiled_map);
}

TEST(SaveMapRequest, LegacyRejectsUnresolvableOrHomeDestination)
{
  std::string error;
  EXPECT_FALSE(lio_sam_hesai::MakeLegacySaveMapExportOptions(
                   "", 0.0F, 0.0F, "", "", true, true, &error)
                   .has_value());
  EXPECT_FALSE(error.empty());

  error.clear();
  // An empty request with an empty configured directory resolves to HOME
  // itself, which the shared validation rejects.
  EXPECT_FALSE(lio_sam_hesai::MakeLegacySaveMapExportOptions(
                   "", 0.0F, 0.0F, "/home/robot", "", true, true, &error)
                   .has_value());
  EXPECT_FALSE(error.empty());
}

TEST(SaveMapRequest, ValidationRejectsEmptyRelativeRootAndHomeDestinations)
{
  std::string error;
  EXPECT_FALSE(lio_sam_hesai::ValidateSaveMapDestination(
      fs::path(""), "/home/robot", &error));
  EXPECT_FALSE(error.empty());

  error.clear();
  EXPECT_FALSE(lio_sam_hesai::ValidateSaveMapDestination(
      fs::path("maps/site"), "/home/robot", &error));
  EXPECT_FALSE(error.empty());

  error.clear();
  EXPECT_FALSE(lio_sam_hesai::ValidateSaveMapDestination(
      fs::path("/"), "/home/robot", &error));
  EXPECT_FALSE(error.empty());

  error.clear();
  EXPECT_FALSE(lio_sam_hesai::ValidateSaveMapDestination(
      fs::path("/home/robot"), "/home/robot", &error));
  EXPECT_FALSE(error.empty());

  error.clear();
  EXPECT_FALSE(lio_sam_hesai::ValidateSaveMapDestination(
      fs::path("/home/robot/"), "/home/robot", &error));
  EXPECT_FALSE(error.empty());

  error.clear();
  EXPECT_FALSE(lio_sam_hesai::ValidateSaveMapDestination(
      fs::path("/home/robot/."), "/home/robot", &error));
  EXPECT_FALSE(error.empty());

  error.clear();
  EXPECT_TRUE(lio_sam_hesai::ValidateSaveMapDestination(
      fs::path("/home/robot/maps"), "/home/robot", &error));
  EXPECT_TRUE(error.empty());
}

TEST(SaveMapRequest, ValidationRejectsSymlinkDestination)
{
  TempDirectory temp;
  const fs::path target = temp.path() / "real";
  const fs::path link = temp.path() / "link";
  fs::create_directories(target);
  fs::create_symlink(target, link);

  std::string error;
  EXPECT_FALSE(
      lio_sam_hesai::ValidateSaveMapDestination(link, "/home/robot", &error));
  EXPECT_FALSE(error.empty());

  // Only the selected destination itself is rejected; a new child below a
  // symlinked parent is still a legitimate, explicitly chosen directory.
  error.clear();
  EXPECT_TRUE(lio_sam_hesai::ValidateSaveMapDestination(
      link / "child", "/home/robot", &error));
  EXPECT_TRUE(error.empty());
}

TEST(SaveMapRequest, ValidationRejectsSymlinkDestinationWithTrailingSeparatorOrDot)
{
  TempDirectory temp;
  const fs::path target = temp.path() / "real";
  const fs::path link = temp.path() / "link";
  fs::create_directories(target);
  fs::create_symlink(target, link);

  // "link/" and "link/." still denote the symlink itself, but stat() resolves
  // them to its target directory, so the symlink check must normalize them
  // before deciding.
  std::string error;
  EXPECT_FALSE(lio_sam_hesai::ValidateSaveMapDestination(
      fs::path(link.string() + "/"), "/home/robot", &error));
  EXPECT_FALSE(error.empty());

  error.clear();
  EXPECT_FALSE(lio_sam_hesai::ValidateSaveMapDestination(
      link / ".", "/home/robot", &error));
  EXPECT_FALSE(error.empty());
}

TEST(SaveMapRequest, OptionsRequestUsesAbsoluteDestinationExactly)
{
  std::string error;
  const auto options = lio_sam_hesai::MakeSaveMapExportOptions(
      "/tmp/site/out/", 0.3F, -1.0F,
      /*save_ground_map=*/true, /*save_tiled_map=*/false, "/home/robot",
      &error);

  ASSERT_TRUE(options.has_value()) << error;
  EXPECT_EQ(options->directory.string(), "/tmp/site/out/");
  EXPECT_FLOAT_EQ(options->resolution, 0.3F);
  EXPECT_FLOAT_EQ(options->ground_resolution, -1.0F);
  EXPECT_TRUE(options->save_ground_map);
  EXPECT_FALSE(options->save_tiled_map);
  EXPECT_TRUE(error.empty());
}

TEST(SaveMapRequest, OptionsRequestRejectsMissingRelativeAndUnsafeDestination)
{
  std::string error;
  EXPECT_FALSE(lio_sam_hesai::MakeSaveMapExportOptions(
                   "", 0.0F, 0.0F, true, true, "/home/robot", &error)
                   .has_value());
  EXPECT_FALSE(error.empty());

  error.clear();
  EXPECT_FALSE(lio_sam_hesai::MakeSaveMapExportOptions(
                   "maps/site", 0.0F, 0.0F, true, true, "/home/robot", &error)
                   .has_value());
  EXPECT_FALSE(error.empty());

  error.clear();
  EXPECT_FALSE(lio_sam_hesai::MakeSaveMapExportOptions(
                   "/", 0.0F, 0.0F, true, true, "/home/robot", &error)
                   .has_value());
  EXPECT_FALSE(error.empty());

  error.clear();
  EXPECT_FALSE(lio_sam_hesai::MakeSaveMapExportOptions(
                   "/home/robot", 0.0F, 0.0F, true, true, "/home/robot", &error)
                   .has_value());
  EXPECT_FALSE(error.empty());
}

TEST(SaveMapRequest, RemovePriorArtifactsDeletesOnlyKnownArtifacts)
{
  TempDirectory temp;
  const fs::path root = temp.path() / "map";
  const MapArtifactLayout layout(root, "tiled");

  WriteFile(layout.GlobalMap());
  WriteFile(layout.GroundMap());
  WriteFile(layout.ScanContextDb());
  WriteFile(layout.Trajectory());
  WriteFile(layout.Transformations());
  WriteFile(layout.CornerMap());
  WriteFile(layout.SurfaceMap());
  WriteFile(layout.TiledManifest());
  WriteFile(root / "tiled.incomplete" / "partial.pcd");
  WriteFile(root / "notes.txt");
  WriteFile(root / "unrelated" / "keep.pcd");

  std::string error;
  ASSERT_TRUE(lio_sam_hesai::RemovePriorMapArtifacts(layout, &error)) << error;

  EXPECT_FALSE(fs::exists(layout.GlobalMap()));
  EXPECT_FALSE(fs::exists(layout.GroundMap()));
  EXPECT_FALSE(fs::exists(layout.ScanContextDb()));
  EXPECT_FALSE(fs::exists(layout.Trajectory()));
  EXPECT_FALSE(fs::exists(layout.Transformations()));
  EXPECT_FALSE(fs::exists(layout.CornerMap()));
  EXPECT_FALSE(fs::exists(layout.SurfaceMap()));
  EXPECT_FALSE(fs::exists(layout.TiledRoot()));
  EXPECT_FALSE(fs::exists(root / "tiled.incomplete"));
  EXPECT_TRUE(fs::exists(root / "notes.txt"));
  EXPECT_TRUE(fs::exists(root / "unrelated" / "keep.pcd"));
  EXPECT_TRUE(error.empty());
}

TEST(SaveMapRequest, RemovePriorArtifactsHonoursConfiguredTiledSubdirectory)
{
  TempDirectory temp;
  const fs::path root = temp.path() / "map";
  const MapArtifactLayout layout(root, "tiles_v2");

  WriteFile(layout.TiledManifest());
  WriteFile(root / "tiles_v2.incomplete" / "partial.pcd");
  // A differently named tiled directory is unrelated and must survive.
  WriteFile(root / "tiled" / "manifest.yaml");

  std::string error;
  ASSERT_TRUE(lio_sam_hesai::RemovePriorMapArtifacts(layout, &error)) << error;

  EXPECT_FALSE(fs::exists(layout.TiledRoot()));
  EXPECT_FALSE(fs::exists(root / "tiles_v2.incomplete"));
  EXPECT_TRUE(fs::exists(root / "tiled" / "manifest.yaml"));
}

TEST(SaveMapRequest, RemovePriorArtifactsRejectsEscapingTiledSubdirectory)
{
  TempDirectory temp;
  const fs::path root = temp.path() / "map";
  WriteFile(root / "GlobalMap.pcd");
  WriteFile(temp.path() / "keep.pcd");

  for (const std::string name : {"..", "../outside", "/outside", "."})
  {
    const MapArtifactLayout layout(root, name);
    std::string error;
    EXPECT_FALSE(lio_sam_hesai::RemovePriorMapArtifacts(layout, &error));
    EXPECT_FALSE(error.empty());
    EXPECT_TRUE(fs::exists(root / "GlobalMap.pcd"));
    EXPECT_TRUE(fs::exists(temp.path() / "keep.pcd"));
  }
}

TEST(SaveMapRequest, RemovePriorArtifactsRejectsDirectoryAtFixedArtifactPath)
{
  TempDirectory temp;
  const fs::path root = temp.path() / "map";
  WriteFile(root / "GlobalMap.pcd" / "keep.txt");

  std::string error;
  EXPECT_FALSE(lio_sam_hesai::RemovePriorMapArtifacts(
      MapArtifactLayout(root, "tiled"), &error));
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(fs::exists(root / "GlobalMap.pcd" / "keep.txt"));
}

TEST(SaveMapRequest, RemovePriorArtifactsRemovesSymlinkedTiledDirectoryWithoutFollowing)
{
  TempDirectory temp;
  const fs::path root = temp.path() / "map";
  const MapArtifactLayout layout(root, "tiled");
  const fs::path target = temp.path() / "real_tiles";

  WriteFile(target / "keep.pcd");
  fs::create_directories(root);
  fs::create_directory_symlink(target, layout.TiledRoot());

  std::string error;
  ASSERT_TRUE(lio_sam_hesai::RemovePriorMapArtifacts(layout, &error)) << error;

  EXPECT_FALSE(fs::exists(layout.TiledRoot()));
  EXPECT_TRUE(fs::exists(target / "keep.pcd"));
}

TEST(SaveMapRequest, OptionsServiceRequestDefaultsAreInert)
{
  lio_sam_hesai::srv::SaveMapOptions::Request request;
  EXPECT_TRUE(request.destination.empty());
  EXPECT_FLOAT_EQ(request.resolution, 0.0F);
  EXPECT_FLOAT_EQ(request.ground_resolution, 0.0F);
  EXPECT_FALSE(request.save_ground_map);
  EXPECT_FALSE(request.save_tiled_map);

  lio_sam_hesai::srv::SaveMapOptions::Response response;
  EXPECT_FALSE(response.success);
  EXPECT_TRUE(response.message.empty());
}

TEST(SaveMapRequest, LegacyServiceRequestShapeIsUnchanged)
{
  lio_sam_hesai::srv::SaveMap::Request request;
  EXPECT_TRUE(request.destination.empty());
  EXPECT_FLOAT_EQ(request.resolution, 0.0F);
  EXPECT_FLOAT_EQ(request.ground_resolution, 0.0F);

  lio_sam_hesai::srv::SaveMap::Response response;
  EXPECT_FALSE(response.success);
}
