#ifndef LIO_SAM_HESAI_COMMON_DUFOMAP_EXPORT_H
#define LIO_SAM_HESAI_COMMON_DUFOMAP_EXPORT_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "common/slam_config.h"

namespace lio_sam_hesai {

// One cached full scan participating in DUFOMap's ray evidence. The pose is
// the final keyframe pose (after loop closure), expressed as a row-major 4x4
// odometry_T_lidar matrix. The scan file itself stays in the lidar frame.
struct DufomapFrameRecord {
  std::size_t index{0};
  std::string scan;  // relative to the dataset directory
  std::array<double, 16> pose{};
};

// A point cloud the exporter wants labelled. `input` is a raw point cloud in
// the odometry frame; `output` is the filtered point cloud, relative to the
// tool output directory.
struct DufomapTargetRecord {
  std::string name;
  std::string input;
  std::string output;
};

struct DufomapManifest {
  DufomapExportConfig parameters;
  std::string backend;
  std::string backend_source;
  std::string ufomap_revision;
  std::vector<DufomapFrameRecord> frames;
  std::vector<DufomapTargetRecord> targets;
  std::string summary{"summary.json"};
};

// Rejects parameter values the native backend cannot use safely. The native
// backend validates its configuration again when a save actually requests it.
bool ValidateDufomapExportConfig(const DufomapExportConfig& config,
                                 std::string* error);

// The legacy static-map filter aggregates points into voxel centroids before
// export. Combining it with DUFOMap would discard the original point evidence
// and coordinates the new path must preserve, so the combination is rejected.
// Callers pass the switch of one save request: a false request remains valid
// even when the legacy filter is enabled.
bool ValidateDufomapFilterCompatibility(
    bool use_dufomap, const StaticMapFilterConfig& static_filter,
    std::string* error);

// Deterministic JSON consumed by scripts/dufomap_export_filter.py.
std::string SerializeDufomapManifest(const DufomapManifest& manifest);

// Creates a fresh per-session directory below the configured cache root. The
// caller owns the directory and keeps it after the session ends. An empty
// `configured_base` selects the system temp directory. A leading `~` path
// component is expanded using HOME.
std::filesystem::path MakeDufomapSessionDirectory(
    const std::string& configured_base, std::uint64_t process_id,
    std::string* error);

}  // namespace lio_sam_hesai

#endif  // LIO_SAM_HESAI_COMMON_DUFOMAP_EXPORT_H
