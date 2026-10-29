#ifndef LIO_SAM_HESAI_COMMON_SAVE_MAP_REQUEST_H
#define LIO_SAM_HESAI_COMMON_SAVE_MAP_REQUEST_H

#include <filesystem>
#include <optional>
#include <string>

#include "common/map_artifact_layout.h"

namespace lio_sam_hesai {

// One resolved map export request. The /lio_sam/save_map_options service
// builds this struct and hands it to the export body in mapOptimization, so
// destination handling and per-save toggles stay in one place.
struct SaveMapExportOptions {
  // Absolute map directory, used exactly as given in the request.
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

// /lio_sam/save_map_options semantics: `requested_destination` is required,
// must be absolute, and is used exactly as given. The ground, tiled, and
// per-request toggles are honoured. DUFOMap is configured online, not here.
// Returns nullopt and fills `error`
// on rejection.
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
bool RemovePriorMapArtifacts(const MapArtifactLayout& layout,
                             std::string* error);

}  // namespace lio_sam_hesai

#endif  // LIO_SAM_HESAI_COMMON_SAVE_MAP_REQUEST_H
