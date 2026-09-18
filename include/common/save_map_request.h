#ifndef LIO_SAM_HESAI_COMMON_SAVE_MAP_REQUEST_H
#define LIO_SAM_HESAI_COMMON_SAVE_MAP_REQUEST_H

#include <filesystem>
#include <optional>
#include <string>

#include "common/map_artifact_layout.h"

namespace lio_sam_hesai
{

// One resolved map export request. The legacy /lio_sam/save_map service and the
// /lio_sam/save_map_options service both build this struct and hand it to the
// same export body in mapOptimization, so destination handling and per-save
// toggles cannot drift apart between the two services.
struct SaveMapExportOptions
{
  // Absolute map directory. The legacy service resolves it from HOME; the
  // options service uses its absolute request value exactly as given.
  std::filesystem::path directory;
  // Raw request values. A negative value selects the configured map_save
  // resolution; zero keeps the historical meaning of no additional
  // downsampling. The export body resolves them against the active
  // configuration.
  float resolution{0.0F};
  float ground_resolution{0.0F};
  bool save_ground_map{true};
  bool save_tiled_map{false};
};

// Rejects a destination that a map save must never modify: an empty or
// relative path, the filesystem root, the user's home directory, or a symlink.
// `home_directory` may be empty, in which case only the root check applies.
// Fills `error` with a human-readable reason on every rejection.
bool ValidateSaveMapDestination(const std::filesystem::path& destination,
                                const std::filesystem::path& home_directory,
                                std::string* error);

// Legacy /lio_sam/save_map semantics: an empty `requested_destination` uses
// `configured_directory`, and either value is appended to HOME exactly as the
// historical implementation did. Ground and tiled exports follow the
// configured flags. Returns nullopt and fills `error` when the destination
// cannot be resolved or is rejected by ValidateSaveMapDestination.
std::optional<SaveMapExportOptions> MakeLegacySaveMapExportOptions(
    const std::string& requested_destination, float resolution,
    float ground_resolution, const std::string& home_directory,
    const std::string& configured_directory, bool configured_save_ground_map,
    bool configured_save_tiled_map, std::string* error = nullptr);

// /lio_sam/save_map_options semantics: `requested_destination` is required,
// must be absolute, and is used exactly as given. The ground and tiled toggles
// are honoured per request. Returns nullopt and fills `error` on rejection.
std::optional<SaveMapExportOptions> MakeSaveMapExportOptions(
    const std::string& requested_destination, float resolution,
    float ground_resolution, bool save_ground_map, bool save_tiled_map,
    const std::string& home_directory, std::string* error = nullptr);

// Removes only the artifacts a map save can write into `layout`: the fixed
// artifact file names, the configured tiled directory, and the matching
// `<tiled>.incomplete` staging directory. Rejects unsafe tiled names and
// directories at fixed file names before removing anything. Unrelated files
// survive, and a symlinked tiled directory is unlinked without touching its
// target. Returns false and fills `error` on the first filesystem failure.
bool RemovePriorMapArtifacts(const MapArtifactLayout& layout, std::string* error);

}  // namespace lio_sam_hesai

#endif  // LIO_SAM_HESAI_COMMON_SAVE_MAP_REQUEST_H
