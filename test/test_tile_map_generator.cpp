#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "localization_map/tile_index.h"
#include "localization_map/tile_manifest.h"
#include "localization_map/tile_map_generator.h"

namespace {

namespace fs = std::filesystem;
using localization_map::GenerateTileMap;
using localization_map::TileIndex;
using localization_map::TileIndexForPosition;
using localization_map::TileManifest;
using localization_map::TileMapGeneratorOptions;

class TemporaryDirectory {
 public:
  explicit TemporaryDirectory(const std::string& label = "tile_map_generator") {
    static std::atomic<std::uint64_t> next_id{0};
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = fs::temp_directory_path() /
            (label + "_" + std::to_string(stamp) + "_" +
             std::to_string(next_id.fetch_add(1)));
    fs::create_directories(root_);
  }

  ~TemporaryDirectory() {
    std::error_code error;
    fs::remove_all(root_, error);
  }

  const fs::path& root() const { return root_; }

 private:
  fs::path root_;
};

pcl::PointXYZI Point(float x, float y, float z = 0.0F,
                     float intensity = 1.0F) {
  pcl::PointXYZI point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  return point;
}

void SaveCloud(const fs::path& path,
               const std::vector<pcl::PointXYZI>& points) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.points.assign(points.begin(), points.end());
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.height = 1;
  cloud.is_dense = true;
  ASSERT_EQ(pcl::io::savePCDFileBinary(path.string(), cloud), 0);
}

void WriteAsciiPcd(const fs::path& path, const std::string& fields,
                   const std::string& sizes, const std::string& types,
                   const std::string& counts, const std::string& point) {
  std::ofstream output(path);
  output << "# .PCD v0.7\nVERSION 0.7\n"
         << "FIELDS " << fields << '\n'
         << "SIZE " << sizes << '\n'
         << "TYPE " << types << '\n'
         << "COUNT " << counts << '\n'
         << "WIDTH 1\nHEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1\nDATA ascii\n"
         << point << '\n';
}

const std::vector<pcl::PointXYZI>& GlobalPoints() {
  static const std::vector<pcl::PointXYZI> points{
      Point(-100.0F, -100.0F, 1.0F, 10.0F),
      Point(-0.001F, 0.0F, 2.0F, 20.0F),
      Point(0.0F, 0.0F, 3.0F, 30.0F),
      Point(99.999F, 100.0F, 4.0F, 40.0F),
      Point(100.0F, 99.0F, 5.0F, 50.0F),
      Point(200.0F, -100.001F, 6.0F, 60.0F),
  };
  return points;
}

const std::vector<pcl::PointXYZI>& GroundPoints() {
  static const std::vector<pcl::PointXYZI> points{
      Point(1.0F, 1.0F, -1.0F, 11.0F),
      Point(-1.0F, 50.0F, -2.0F, 22.0F),
      Point(150.0F, 150.0F, -3.0F, 33.0F),
  };
  return points;
}

void PopulateInputs(const fs::path& root, bool include_ground = true) {
  SaveCloud(root / "GlobalMap.pcd", GlobalPoints());
  if (include_ground) {
    SaveCloud(root / "GroundMap.pcd", GroundPoints());
  }
}

TileMapGeneratorOptions OptionsFor(const fs::path& root,
                                   const fs::path& output = "tiled") {
  TileMapGeneratorOptions options;
  options.map_directory = root;
  options.output_subdirectory = output;
  options.tile_size_m = 100.0;
  options.origin = {0.0, 0.0};
  options.overview_resolution_m = 5.0;
  options.frame_id = "map";
  return options;
}

std::string ReadFile(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("unable to read test file: " + path.string());
  }
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}

std::vector<fs::path> RelativeRegularFiles(const fs::path& root) {
  std::vector<fs::path> files;
  for (const fs::directory_entry& entry : fs::recursive_directory_iterator(root)) {
    if (entry.is_regular_file()) {
      files.push_back(fs::relative(entry.path(), root));
    }
  }
  std::sort(files.begin(), files.end());
  return files;
}

void ExpectNoOutput(const fs::path& root,
                    const fs::path& output = "tiled") {
  EXPECT_FALSE(fs::exists(root / output));
  EXPECT_FALSE(fs::exists(root / (output.string() + ".incomplete")));
}

TEST(TileMapGenerator, PartitionsAndPublishesAValidatedBundle) {
  TemporaryDirectory directory;
  PopulateInputs(directory.root());

  const auto result = GenerateTileMap(OptionsFor(directory.root()));

  EXPECT_EQ(result.global_input_points, 6U);
  EXPECT_EQ(result.global_output_points, 6U);
  EXPECT_EQ(result.ground_input_points, 3U);
  EXPECT_EQ(result.ground_output_points, 3U);
  // tile_count is the union of logical indices represented by global or ground.
  EXPECT_EQ(result.tile_count, 7U);
  EXPECT_EQ(result.output_directory, directory.root() / "tiled");
  EXPECT_TRUE(fs::is_regular_file(directory.root() / "tiled/manifest.yaml"));
  EXPECT_FALSE(fs::exists(directory.root() / "tiled.incomplete"));

  const TileManifest manifest = TileManifest::Load(result.output_directory);
  EXPECT_EQ(manifest.source.global_map_bytes,
            fs::file_size(directory.root() / "GlobalMap.pcd"));
  EXPECT_EQ(manifest.source.ground_map_bytes,
            fs::file_size(directory.root() / "GroundMap.pcd"));
  ASSERT_TRUE(manifest.overview.has_value());

  std::uint64_t global_count = 0;
  std::uint64_t ground_count = 0;
  const auto verify_tiles = [&](const auto& tiles, std::uint64_t* total) {
    for (const auto& [index, metadata] : tiles) {
      const fs::path path = result.output_directory / metadata.path;
      EXPECT_EQ(metadata.bytes, fs::file_size(path));
      pcl::PointCloud<pcl::PointXYZI> cloud;
      ASSERT_EQ(pcl::io::loadPCDFile(path.string(), cloud), 0);
      EXPECT_EQ(metadata.point_count, cloud.size());
      *total += cloud.size();
      for (const pcl::PointXYZI& point : cloud) {
        EXPECT_EQ(TileIndexForPosition(point.x, point.y, 100.0, 0.0, 0.0),
                  index);
      }
    }
  };
  verify_tiles(manifest.global_tiles, &global_count);
  verify_tiles(manifest.ground_tiles, &ground_count);
  EXPECT_EQ(global_count, 6U);
  EXPECT_EQ(ground_count, 3U);
}

TEST(TileMapGenerator, ProducesByteIdenticalBundlesForIdenticalInputs) {
  TemporaryDirectory first("tile_map_determinism_first");
  TemporaryDirectory second("tile_map_determinism_second");
  PopulateInputs(first.root());
  PopulateInputs(second.root());

  GenerateTileMap(OptionsFor(first.root()));
  GenerateTileMap(OptionsFor(second.root()));

  const fs::path first_output = first.root() / "tiled";
  const fs::path second_output = second.root() / "tiled";
  const auto first_files = RelativeRegularFiles(first_output);
  EXPECT_EQ(first_files, RelativeRegularFiles(second_output));
  for (const fs::path& relative : first_files) {
    EXPECT_EQ(ReadFile(first_output / relative),
              ReadFile(second_output / relative))
        << relative;
  }
}

TEST(TileMapGenerator, RefusesToOverwriteAnExistingFinalOutput) {
  TemporaryDirectory directory;
  PopulateInputs(directory.root());
  fs::create_directories(directory.root() / "tiled");
  const fs::path sentinel = directory.root() / "tiled/manifest.yaml";
  {
    std::ofstream output(sentinel);
    output << "sentinel manifest\n";
  }

  EXPECT_THROW(GenerateTileMap(OptionsFor(directory.root())), std::exception);
  EXPECT_EQ(ReadFile(sentinel), "sentinel manifest\n");
}

TEST(TileMapGenerator, MissingOrInvalidGlobalMapLeavesNoArtifacts) {
  TemporaryDirectory missing("tile_map_missing_global");
  EXPECT_THROW(GenerateTileMap(OptionsFor(missing.root())), std::exception);
  ExpectNoOutput(missing.root());

  TemporaryDirectory invalid("tile_map_invalid_global");
  {
    std::ofstream output(invalid.root() / "GlobalMap.pcd");
    output << "not a PCD file\n";
  }
  EXPECT_THROW(GenerateTileMap(OptionsFor(invalid.root())), std::exception);
  ExpectNoOutput(invalid.root());
}

TEST(TileMapGenerator, RejectsAnEmptyGlobalMap) {
  TemporaryDirectory directory;
  {
    std::ofstream output(directory.root() / "GlobalMap.pcd");
    output << "# .PCD v0.7\nVERSION 0.7\n"
              "FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
              "COUNT 1 1 1 1\nWIDTH 0\nHEIGHT 1\n"
              "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 0\nDATA ascii\n";
  }

  try {
    GenerateTileMap(OptionsFor(directory.root()));
    FAIL() << "expected empty GlobalMap.pcd to be rejected";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("at least one point"),
              std::string::npos)
        << error.what();
  }
  ExpectNoOutput(directory.root());
}

TEST(TileMapGenerator, RejectsANonFiniteInputPointWithoutArtifacts) {
  TemporaryDirectory directory;
  SaveCloud(directory.root() / "GlobalMap.pcd",
            {Point(std::numeric_limits<float>::quiet_NaN(), 1.0F)});

  try {
    GenerateTileMap(OptionsFor(directory.root()));
    FAIL() << "expected non-finite input point to be rejected";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("non-finite XYZI point"),
              std::string::npos)
        << error.what();
  }
  ExpectNoOutput(directory.root());
}

TEST(TileMapGenerator, RejectsPcdSchemasOtherThanCanonicalPointXYZI) {
  struct SchemaCase {
    std::string fields;
    std::string sizes;
    std::string types;
    std::string counts;
    std::string point;
  };
  const std::vector<SchemaCase> cases{
      {"x y z", "4 4 4", "F F F", "1 1 1", "1 2 3"},
      {"x y z intensity ring", "4 4 4 4 2", "F F F F U", "1 1 1 1 1",
       "1 2 3 4 5"},
      {"y x z intensity", "4 4 4 4", "F F F F", "1 1 1 1", "2 1 3 4"},
      {"x y z intensity intensity", "4 4 4 4 4", "F F F F F",
       "1 1 1 1 1", "1 2 3 4 5"},
      {"x y z intensity", "4 4 4 4", "F F F U", "1 1 1 1", "1 2 3 4"},
      {"x y z intensity", "4 4 4 4", "F F F F", "1 1 1 2", "1 2 3 4 5"},
  };

  for (const SchemaCase& schema : cases) {
    TemporaryDirectory directory("tile_map_wrong_schema");
    WriteAsciiPcd(directory.root() / "GlobalMap.pcd", schema.fields,
                  schema.sizes, schema.types, schema.counts, schema.point);
    try {
      GenerateTileMap(OptionsFor(directory.root()));
      FAIL() << "expected non-PointXYZI schema to be rejected: "
             << schema.fields;
    } catch (const std::runtime_error& error) {
      EXPECT_NE(std::string(error.what()).find(
                    "exactly four float32 fields in order: x, y, z, intensity"),
                std::string::npos)
          << error.what();
    }
    ExpectNoOutput(directory.root());
  }
}

TEST(TileMapGenerator, AllowsAnAbsentGroundMap) {
  TemporaryDirectory directory;
  PopulateInputs(directory.root(), false);

  const auto result = GenerateTileMap(OptionsFor(directory.root()));

  EXPECT_EQ(result.global_input_points, 6U);
  EXPECT_EQ(result.global_output_points, 6U);
  EXPECT_EQ(result.ground_input_points, 0U);
  EXPECT_EQ(result.ground_output_points, 0U);
  EXPECT_EQ(result.tile_count, 6U);
  const TileManifest manifest = TileManifest::Load(result.output_directory);
  EXPECT_EQ(manifest.source.global_map_name, "GlobalMap.pcd");
  EXPECT_GT(manifest.source.global_map_bytes, 0U);
  EXPECT_EQ(manifest.source.ground_map_name, "GroundMap.pcd");
  EXPECT_EQ(manifest.source.ground_map_bytes, 0U);
  EXPECT_TRUE(manifest.ground_tiles.empty());
}

TEST(TileMapGenerator, RejectsInvalidNumericAndFrameOptions) {
  const auto expect_invalid = [](const auto& mutate) {
    TemporaryDirectory directory("tile_map_invalid_option");
    PopulateInputs(directory.root());
    TileMapGeneratorOptions options = OptionsFor(directory.root());
    mutate(options);
    EXPECT_THROW(GenerateTileMap(options), std::exception);
    ExpectNoOutput(directory.root());
  };

  expect_invalid([](auto& options) { options.tile_size_m = 0.0; });
  expect_invalid([](auto& options) { options.tile_size_m = -1.0; });
  expect_invalid([](auto& options) {
    options.tile_size_m = std::numeric_limits<double>::infinity();
  });
  expect_invalid([](auto& options) {
    options.overview_resolution_m = std::numeric_limits<double>::quiet_NaN();
  });
  expect_invalid([](auto& options) { options.overview_resolution_m = 0.0; });
  expect_invalid([](auto& options) {
    options.origin[0] = std::numeric_limits<double>::infinity();
  });
  expect_invalid([](auto& options) { options.frame_id.clear(); });
}

TEST(TileMapGenerator, RejectsUnsafeOutputSubdirectories) {
  const std::vector<fs::path> unsafe_outputs{
      fs::path{}, fs::path{"."}, fs::path{".."}, fs::path{"nested/tiled"},
      fs::temp_directory_path() / "absolute_tiled_output"};
  for (const fs::path& output : unsafe_outputs) {
    TemporaryDirectory directory("tile_map_unsafe_output");
    PopulateInputs(directory.root());
    EXPECT_THROW(GenerateTileMap(OptionsFor(directory.root(), output)),
                 std::exception)
        << output;
  }

  TemporaryDirectory directory("tile_map_output_symlink");
  TemporaryDirectory outside("tile_map_output_symlink_outside");
  PopulateInputs(directory.root());
  {
    std::ofstream output(outside.root() / "sentinel");
    output << "outside remains\n";
  }
  fs::create_directory_symlink(outside.root(), directory.root() / "escape");
  EXPECT_THROW(GenerateTileMap(OptionsFor(directory.root(), "escape")),
               std::exception);
  EXPECT_EQ(ReadFile(outside.root() / "sentinel"), "outside remains\n");
}

TEST(TileMapGenerator, RejectsEmbeddedNulBeforeFilesystemWork) {
  TemporaryDirectory directory("tile_map_nul_output");
  PopulateInputs(directory.root());
  TileMapGeneratorOptions options = OptionsFor(directory.root());
  options.output_subdirectory = fs::path(std::string("nul\0suffix", 10));

  try {
    GenerateTileMap(options);
    FAIL() << "expected embedded NUL output name to be rejected";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find("output_subdirectory"),
              std::string::npos)
        << error.what();
  }
  EXPECT_FALSE(fs::exists(directory.root() / "nul"));
  EXPECT_FALSE(fs::exists(directory.root() / "nul.incomplete"));
  EXPECT_FALSE(fs::exists(directory.root() / "tiled"));
  EXPECT_FALSE(fs::exists(directory.root() / "tiled.incomplete"));
}

TEST(TileMapGenerator, RemovesAValidatedStaleIncompleteDirectory) {
  TemporaryDirectory directory;
  PopulateInputs(directory.root());
  fs::create_directories(directory.root() / "tiled.incomplete/old");
  {
    std::ofstream output(directory.root() / "tiled.incomplete/old/sentinel");
    output << "stale\n";
  }

  EXPECT_NO_THROW(GenerateTileMap(OptionsFor(directory.root())));
  EXPECT_TRUE(fs::is_regular_file(directory.root() / "tiled/manifest.yaml"));
  EXPECT_FALSE(fs::exists(directory.root() / "tiled.incomplete"));
}

TEST(TileMapGenerator, PreservesARegularFileAtTheIncompletePath) {
  TemporaryDirectory directory;
  PopulateInputs(directory.root());
  const fs::path sentinel = directory.root() / "tiled.incomplete";
  {
    std::ofstream output(sentinel);
    output << "do not delete\n";
  }

  EXPECT_THROW(GenerateTileMap(OptionsFor(directory.root())), std::exception);
  EXPECT_EQ(ReadFile(sentinel), "do not delete\n");
  EXPECT_FALSE(fs::exists(directory.root() / "tiled"));
}

TEST(TileMapGenerator, CleansIncompleteOutputAfterPostCreateFailure) {
  TemporaryDirectory directory;
  PopulateInputs(directory.root());
  TileMapGeneratorOptions options = OptionsFor(directory.root());
  options.overview_resolution_m = std::numeric_limits<double>::denorm_min();

  EXPECT_THROW(GenerateTileMap(options), std::exception);
  ExpectNoOutput(directory.root());
}

TEST(TileMapGenerator, NeverFollowsAnIncompleteSymlinkDuringCleanup) {
  TemporaryDirectory directory;
  TemporaryDirectory outside("tile_map_incomplete_symlink_outside");
  PopulateInputs(directory.root());
  {
    std::ofstream output(outside.root() / "sentinel");
    output << "outside remains\n";
  }
  fs::create_directory_symlink(outside.root(),
                               directory.root() / "tiled.incomplete");

  EXPECT_THROW(GenerateTileMap(OptionsFor(directory.root())), std::exception);
  EXPECT_EQ(ReadFile(outside.root() / "sentinel"), "outside remains\n");
  EXPECT_TRUE(fs::is_symlink(directory.root() / "tiled.incomplete"));
  EXPECT_FALSE(fs::exists(directory.root() / "tiled"));
}

}  // namespace
