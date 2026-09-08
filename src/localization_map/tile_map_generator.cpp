#include "localization_map/tile_map_generator.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#ifdef __linux__
#include <fcntl.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

#include "localization_map/tile_index.h"
#include "localization_map/tile_manifest.h"

namespace localization_map {
namespace {

namespace fs = std::filesystem;
using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
using TileClouds = std::map<TileIndex, PointCloud>;

[[noreturn]] void Fail(const std::string& message) {
  throw std::runtime_error("tile map generation failed: " + message);
}

fs::file_status SymlinkStatus(const fs::path& path, const char* label) {
  std::error_code error;
  const fs::file_status status = fs::symlink_status(path, error);
  if (error == std::errc::no_such_file_or_directory) {
    return fs::file_status(fs::file_type::not_found);
  }
  if (error) {
    Fail(std::string("unable to inspect ") + label + ": " + error.message());
  }
  return status;
}

bool IsMissing(const fs::file_status& status) {
  return status.type() == fs::file_type::not_found;
}

void ValidateOutputName(const fs::path& output) {
  if (output.native().find(fs::path::value_type{}) != fs::path::string_type::npos ||
      output.empty() || output.is_absolute() || output.has_root_path() ||
      output.parent_path() != fs::path{} || output.filename() != output ||
      output == "." || output == "..") {
    throw std::invalid_argument(
        "output_subdirectory must be exactly one safe relative filename component");
  }
}

fs::path ValidateAndResolveMapDirectory(const fs::path& map_directory) {
  if (map_directory.empty()) {
    throw std::invalid_argument("map_directory must be nonempty");
  }
  const fs::file_status status = SymlinkStatus(map_directory, "map_directory");
  if (IsMissing(status) || !fs::is_directory(status) || fs::is_symlink(status)) {
    throw std::invalid_argument(
        "map_directory must name an existing non-symlink directory");
  }
  std::error_code error;
  const fs::path root = fs::canonical(map_directory, error);
  if (error) {
    Fail("unable to resolve map_directory: " + error.message());
  }
  return root;
}

void ValidateOptions(const TileMapGeneratorOptions& options) {
  ValidateOutputName(options.output_subdirectory);
  if (!std::isfinite(options.tile_size_m) || options.tile_size_m <= 0.0) {
    throw std::invalid_argument("tile_size_m must be finite and positive");
  }
  if (!std::isfinite(options.overview_resolution_m) ||
      options.overview_resolution_m <= 0.0) {
    throw std::invalid_argument(
        "overview_resolution_m must be finite and positive");
  }
  if (!std::isfinite(options.origin[0]) ||
      !std::isfinite(options.origin[1])) {
    throw std::invalid_argument("origin values must be finite");
  }
  if (options.frame_id.empty()) {
    throw std::invalid_argument("frame_id must be nonempty");
  }
}

void RefuseExistingFinal(const fs::path& final_directory) {
  if (!IsMissing(SymlinkStatus(final_directory, "final output"))) {
    Fail("final output already exists: " + final_directory.string());
  }
}

void RemoveStaleIncomplete(const fs::path& incomplete_directory) {
  const fs::file_status status =
      SymlinkStatus(incomplete_directory, "incomplete output");
  if (IsMissing(status)) {
    return;
  }
  if (fs::is_symlink(status)) {
    Fail("incomplete output must not be a symlink");
  }
  if (!fs::is_directory(status)) {
    Fail("incomplete output must be a directory");
  }
  std::error_code error;
  fs::remove_all(incomplete_directory, error);
  if (error) {
    Fail("unable to remove stale incomplete output: " + error.message());
  }
}

class IncompleteGuard {
 public:
  explicit IncompleteGuard(fs::path path) : path_(std::move(path)) {}

  ~IncompleteGuard() {
    if (!active_) {
      return;
    }
    std::error_code error;
    fs::remove_all(path_, error);
  }

  void Dismiss() { active_ = false; }

 private:
  fs::path path_;
  bool active_{true};
};

std::uintmax_t RequiredRegularFileSize(const fs::path& path,
                                       const char* label) {
  const fs::file_status status = SymlinkStatus(path, label);
  if (IsMissing(status)) {
    Fail(std::string(label) + " is missing");
  }
  if (!fs::is_regular_file(status) || fs::is_symlink(status)) {
    Fail(std::string(label) + " must be a regular non-symlink file");
  }
  std::error_code error;
  const std::uintmax_t bytes = fs::file_size(path, error);
  if (error) {
    Fail(std::string("unable to inspect ") + label + " size: " +
         error.message());
  }
  return bytes;
}

bool OptionalRegularFileSize(const fs::path& path, const char* label,
                             std::uintmax_t* bytes) {
  const fs::file_status status = SymlinkStatus(path, label);
  if (IsMissing(status)) {
    *bytes = 0;
    return false;
  }
  *bytes = RequiredRegularFileSize(path, label);
  return true;
}

void ValidateCloud(const PointCloud& cloud, const char* label,
                   bool require_nonempty) {
  if (require_nonempty && cloud.empty()) {
    Fail(std::string(label) + " must contain at least one point");
  }
  for (const pcl::PointXYZI& point : cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
      Fail(std::string(label) + " contains a non-finite XYZI point");
    }
  }
}

PointCloud::Ptr LoadCloud(const fs::path& path, const char* label,
                          bool require_nonempty) {
  pcl::PCLPointCloud2 encoded;
  if (pcl::io::loadPCDFile(path.string(), encoded) != 0) {
    if (require_nonempty && !encoded.fields.empty() &&
        (encoded.width == 0U || encoded.height == 0U)) {
      Fail(std::string(label) + " must contain at least one point");
    }
    Fail(std::string("unable to load ") + label);
  }
  static constexpr const char* kPointXYZIFields[]{"x", "y", "z",
                                                  "intensity"};
  if (encoded.fields.size() != std::size(kPointXYZIFields)) {
    Fail(std::string(label) +
         " must have exactly four float32 fields in order: x, y, z, intensity");
  }
  for (std::size_t index = 0; index < std::size(kPointXYZIFields); ++index) {
    const pcl::PCLPointField& field = encoded.fields[index];
    if (field.name != kPointXYZIFields[index] || field.count != 1U ||
        field.datatype != pcl::PCLPointField::FLOAT32) {
      Fail(std::string(label) +
           " must have exactly four float32 fields in order: x, y, z, intensity");
    }
  }
  PointCloud::Ptr cloud(new PointCloud);
  pcl::fromPCLPointCloud2(encoded, *cloud);
  ValidateCloud(*cloud, label, require_nonempty);
  return cloud;
}

std::uint64_t PointCount(const PointCloud& cloud, const char* label) {
  if (cloud.size() > std::numeric_limits<std::uint64_t>::max()) {
    Fail(std::string(label) + " point count is not representable");
  }
  return static_cast<std::uint64_t>(cloud.size());
}

TileClouds Partition(const PointCloud& cloud,
                     const TileMapGeneratorOptions& options) {
  TileClouds tiles;
  for (const pcl::PointXYZI& point : cloud) {
    const TileIndex index = TileIndexForPosition(
        point.x, point.y, options.tile_size_m, options.origin[0],
        options.origin[1]);
    PointCloud& tile = tiles[index];
    tile.points.push_back(point);
  }
  for (auto& [index, tile] : tiles) {
    (void)index;
    tile.width = static_cast<std::uint32_t>(tile.size());
    tile.height = 1;
    tile.is_dense = true;
  }
  return tiles;
}

std::uint64_t SaveTiles(const fs::path& incomplete_directory,
                        const char* category, const TileClouds& tiles,
                        std::map<TileIndex, TileFileMetadata>* metadata) {
  const fs::path directory = incomplete_directory / category;
  std::error_code error;
  fs::create_directory(directory, error);
  if (error) {
    Fail(std::string("unable to create ") + category +
         " tile directory: " + error.message());
  }

  std::uint64_t output_points = 0;
  for (const auto& [index, cloud] : tiles) {
    const fs::path relative =
        fs::path(category) / (TileFileStem(index) + ".pcd");
    const fs::path path = incomplete_directory / relative;
    if (pcl::io::savePCDFileBinary(path.string(), cloud) != 0) {
      Fail("unable to save tile: " + relative.generic_string());
    }
    const std::uint64_t count = PointCount(cloud, "tile");
    if (output_points > std::numeric_limits<std::uint64_t>::max() - count) {
      Fail("tile point count overflow");
    }
    output_points += count;
    metadata->emplace(
        index, TileFileMetadata{relative, count,
                                RequiredRegularFileSize(path, "saved tile")});
  }
  return output_points;
}

void SaveOverview(const fs::path& incomplete_directory,
                  const PointCloud::ConstPtr& global_cloud,
                  double resolution_m) {
  const float leaf_size = static_cast<float>(resolution_m);
  if (!std::isfinite(leaf_size) || leaf_size <= 0.0F) {
    throw std::invalid_argument(
        "overview_resolution_m must be representable as a positive float");
  }
  pcl::VoxelGrid<pcl::PointXYZI> filter;
  filter.setInputCloud(global_cloud);
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  PointCloud overview;
  filter.filter(overview);
  if (overview.empty()) {
    Fail("overview filtering produced an empty cloud");
  }
  const fs::path path = incomplete_directory / "overview.pcd";
  if (pcl::io::savePCDFileBinary(path.string(), overview) != 0) {
    Fail("unable to save overview.pcd");
  }
  RequiredRegularFileSize(path, "saved overview");
}

void PublishNoReplace(const fs::path& incomplete_directory,
                      const fs::path& final_directory) {
#if defined(__linux__) && defined(SYS_renameat2)
  if (::syscall(SYS_renameat2, AT_FDCWD, incomplete_directory.c_str(),
                AT_FDCWD, final_directory.c_str(), RENAME_NOREPLACE) == 0) {
    return;
  }
  const int rename_error = errno;
  if (rename_error == EEXIST) {
    Fail("final output appeared during publication");
  }
  if (rename_error == ENOSYS || rename_error == EINVAL) {
    Fail("atomic no-replace publication is unsupported by this filesystem");
  }
  Fail("unable to publish tiled map: " +
       std::string(std::strerror(rename_error)));
#else
  (void)incomplete_directory;
  (void)final_directory;
  Fail("atomic no-replace publication is unsupported on this platform");
#endif
}

}  // namespace

TileMapGeneratorResult GenerateTileMap(const TileMapGeneratorOptions& options) {
  ValidateOptions(options);
  const fs::path map_directory =
      ValidateAndResolveMapDirectory(options.map_directory);
  const fs::path final_directory =
      map_directory / options.output_subdirectory;
  const fs::path incomplete_directory =
      map_directory /
      (options.output_subdirectory.filename().string() + ".incomplete");

  RefuseExistingFinal(final_directory);
  RemoveStaleIncomplete(incomplete_directory);
  IncompleteGuard incomplete_guard(incomplete_directory);

  const fs::path global_path = map_directory / "GlobalMap.pcd";
  const fs::path ground_path = map_directory / "GroundMap.pcd";
  const std::uintmax_t global_bytes =
      RequiredRegularFileSize(global_path, "GlobalMap.pcd");
  std::uintmax_t ground_bytes = 0;
  const bool has_ground =
      OptionalRegularFileSize(ground_path, "GroundMap.pcd", &ground_bytes);

  std::error_code error;
  fs::create_directory(incomplete_directory, error);
  if (error) {
    Fail("unable to create incomplete output: " + error.message());
  }

  TileManifest manifest;
  manifest.schema_version = TileManifest::kSchemaVersion;
  manifest.frame_id = options.frame_id;
  manifest.point_type = "XYZI";
  manifest.tile_size_m = options.tile_size_m;
  manifest.origin = options.origin;
  manifest.source.global_map_name = "GlobalMap.pcd";
  manifest.source.global_map_bytes = global_bytes;
  manifest.source.ground_map_name = "GroundMap.pcd";
  manifest.source.ground_map_bytes = ground_bytes;

  TileMapGeneratorResult result;
  std::set<TileIndex> logical_tiles;
  {
    PointCloud::Ptr global_cloud =
        LoadCloud(global_path, "GlobalMap.pcd", true);
    result.global_input_points = PointCount(*global_cloud, "GlobalMap.pcd");

    SaveOverview(incomplete_directory, global_cloud,
                 options.overview_resolution_m);
    manifest.overview =
        OverviewMetadata{fs::path("overview.pcd"),
                         options.overview_resolution_m};

    TileClouds global_tiles = Partition(*global_cloud, options);
    global_cloud.reset();
    result.global_output_points =
        SaveTiles(incomplete_directory, "global", global_tiles,
                  &manifest.global_tiles);
    for (const auto& [index, cloud] : global_tiles) {
      (void)cloud;
      logical_tiles.insert(index);
    }
  }

  {
    TileClouds ground_tiles;
    if (has_ground) {
      PointCloud::Ptr ground_cloud =
          LoadCloud(ground_path, "GroundMap.pcd", false);
      result.ground_input_points =
          PointCount(*ground_cloud, "GroundMap.pcd");
      ground_tiles = Partition(*ground_cloud, options);
      ground_cloud.reset();
    }
    result.ground_output_points =
        SaveTiles(incomplete_directory, "ground", ground_tiles,
                  &manifest.ground_tiles);
    for (const auto& [index, cloud] : ground_tiles) {
      (void)cloud;
      logical_tiles.insert(index);
    }
  }

  if (result.global_input_points != result.global_output_points ||
      result.ground_input_points != result.ground_output_points) {
    Fail("unfiltered tile point-count conservation check failed");
  }

  manifest.Save(incomplete_directory);
  (void)TileManifest::Load(incomplete_directory);

  result.tile_count = logical_tiles.size();

  PublishNoReplace(incomplete_directory, final_directory);
  incomplete_guard.Dismiss();
  result.output_directory = final_directory;
  return result;
}

}  // namespace localization_map
