#include "common/save_map_request.h"

#include <array>
#include <system_error>
#include <utility>

namespace lio_sam_hesai
{

namespace
{

namespace fs = std::filesystem;

void SetError(std::string* error, std::string message)
{
  if (error != nullptr)
  {
    *error = std::move(message);
  }
}

// Comparison key for the root/home checks: resolves symlinked prefixes when
// possible and drops the trailing separator that forms such as "dir/." keep.
// Falls back to the lexical form when the path cannot be canonicalized.
fs::path ComparisonKey(const fs::path& path)
{
  std::error_code error;
  fs::path key = fs::weakly_canonical(path, error);
  if (error)
  {
    key = path;
  }
  key = key.lexically_normal();
  if (key != key.root_path() && !key.empty() && key.filename().empty())
  {
    key = key.parent_path();
  }
  return key;
}

// Path used for the symlink check. A trailing separator or a trailing "."
// makes lstat() report the symlink's target instead of the symlink itself, so
// "link/" and "link/." would otherwise slip past the rejection. Folding the
// lexical form and dropping the trailing separator keeps the check on the
// component the caller actually named.
fs::path SymlinkCheckPath(const fs::path& path)
{
  fs::path check = path.lexically_normal();
  while (check.filename().empty() && check.has_parent_path() &&
         check != check.parent_path())
  {
    check = check.parent_path();
  }
  return check;
}

}  // namespace

bool ValidateSaveMapDestination(const fs::path& destination,
                                const fs::path& home_directory,
                                std::string* error)
{
  if (destination.empty())
  {
    SetError(error, "save destination must not be empty");
    return false;
  }
  if (!destination.is_absolute())
  {
    SetError(error, "save destination must be an absolute path: " +
                        destination.string());
    return false;
  }

  if (ComparisonKey(destination) == ComparisonKey(fs::path("/")))
  {
    SetError(error, "save destination must not be the filesystem root");
    return false;
  }
  if (!home_directory.empty() &&
      ComparisonKey(destination) == ComparisonKey(home_directory))
  {
    SetError(error, "save destination must not be the home directory: " +
                        destination.string());
    return false;
  }

  std::error_code status_error;
  const fs::file_status status =
      fs::symlink_status(SymlinkCheckPath(destination), status_error);
  if (!status_error && fs::is_symlink(status))
  {
    SetError(error, "save destination must not be a symlink: " +
                        destination.string());
    return false;
  }

  return true;
}

std::optional<SaveMapExportOptions> MakeLegacySaveMapExportOptions(
    const std::string& requested_destination, float resolution,
    float ground_resolution, const std::string& home_directory,
    const std::string& configured_directory, bool configured_save_ground_map,
    bool configured_save_tiled_map, std::string* error)
{
  const std::string& suffix =
      requested_destination.empty() ? configured_directory
                                    : requested_destination;
  // Historical semantics: the destination is concatenated onto HOME, so a
  // caller-provided "/map/site" lands in "$HOME/map/site".
  const std::string resolved = home_directory + suffix;
  if (resolved.empty())
  {
    SetError(error, "save destination is not configured");
    return std::nullopt;
  }

  SaveMapExportOptions options;
  options.directory = resolved;
  options.resolution = resolution;
  options.ground_resolution = ground_resolution;
  options.save_ground_map = configured_save_ground_map;
  options.save_tiled_map = configured_save_tiled_map;

  if (!ValidateSaveMapDestination(options.directory, home_directory, error))
  {
    return std::nullopt;
  }
  return options;
}

std::optional<SaveMapExportOptions> MakeSaveMapExportOptions(
    const std::string& requested_destination, float resolution,
    float ground_resolution, bool save_ground_map, bool save_tiled_map,
    const std::string& home_directory, std::string* error)
{
  const fs::path destination(requested_destination);
  if (!ValidateSaveMapDestination(destination, home_directory, error))
  {
    return std::nullopt;
  }

  SaveMapExportOptions options;
  options.directory = destination;
  options.resolution = resolution;
  options.ground_resolution = ground_resolution;
  options.save_ground_map = save_ground_map;
  options.save_tiled_map = save_tiled_map;
  return options;
}

bool RemovePriorMapArtifacts(const MapArtifactLayout& layout, std::string* error)
{
  const fs::path tiled_name = layout.TiledRoot().lexically_relative(layout.Root());
  if (tiled_name.empty() || tiled_name.is_absolute() ||
      tiled_name.parent_path() != fs::path{} ||
      tiled_name.filename() != tiled_name ||
      tiled_name == "." || tiled_name == ".." ||
      tiled_name.native().find(fs::path::value_type{}) !=
          fs::path::string_type::npos)
  {
    SetError(error, "tiled output subdirectory must be one safe relative filename");
    return false;
  }

  // The tile generator stages into "<output_subdirectory>.incomplete" next to
  // the map root, keyed by the configured subdirectory's basename.
  const fs::path incomplete_directory =
      layout.Root() /
      (layout.TiledRoot().filename().string() + ".incomplete");
  const std::array<fs::path, 7> files{
      layout.GlobalMap(),       layout.GroundMap(),
      layout.ScanContextDb(),   layout.Trajectory(),
      layout.Transformations(), layout.CornerMap(),
      layout.SurfaceMap(),
  };

  // Fixed artifact names are files. Reject a directory at one of those names
  // before deleting anything, so its contents cannot be mistaken for map data.
  for (const fs::path& artifact : files)
  {
    std::error_code status_error;
    const fs::file_status status = fs::symlink_status(artifact, status_error);
    if (status_error && status_error != std::errc::no_such_file_or_directory)
    {
      SetError(error, "unable to inspect map artifact " + artifact.string() +
                          ": " + status_error.message());
      return false;
    }
    if (!status_error && fs::is_directory(status))
    {
      SetError(error, "map artifact path is a directory: " + artifact.string());
      return false;
    }
  }

  for (const fs::path& artifact : files)
  {
    std::error_code remove_error;
    fs::remove(artifact, remove_error);
    if (remove_error)
    {
      SetError(error, "unable to remove prior map artifact " +
                          artifact.string() + ": " + remove_error.message());
      return false;
    }
  }
  for (const fs::path& artifact : {layout.TiledRoot(), incomplete_directory})
  {
    std::error_code remove_error;
    // remove_all never follows a symlink: a symlinked tiled directory is
    // unlinked, not deleted through.
    fs::remove_all(artifact, remove_error);
    if (remove_error)
    {
      SetError(error, "unable to remove prior map artifact " +
                          artifact.string() + ": " + remove_error.message());
      return false;
    }
  }
  return true;
}

}  // namespace lio_sam_hesai
