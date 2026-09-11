#include "common/map_artifact_layout.h"

#include <stdexcept>
#include <utility>

MapArtifactLayout::MapArtifactLayout(std::filesystem::path root,
                                     std::string tile_subdirectory)
    : root_(root.lexically_normal()),
      tile_subdirectory_(std::move(tile_subdirectory)) {
  if (root_.empty()) {
    throw std::invalid_argument("map artifact root must not be empty");
  }
  if (tile_subdirectory_.empty()) {
    throw std::invalid_argument("tile subdirectory must not be empty");
  }
}

MapArtifactLayout MapArtifactLayout::FromGlobalMap(
    const std::filesystem::path& global_map) {
  if (global_map.empty()) {
    throw std::invalid_argument("global map path must not be empty");
  }
  // Checked before any I/O so a wrongly named file cannot silently retarget
  // the ground map and Scan Context artifacts to its own directory.
  if (global_map.filename() != kGlobalMapName) {
    throw std::invalid_argument("global map path must be named " +
                                std::string(kGlobalMapName) + ": " +
                                global_map.string());
  }
  return MapArtifactLayout(global_map.parent_path());
}

MapArtifactLayout MapArtifactLayout::FromTiledRoot(
    const std::filesystem::path& tiled_root,
    const std::string& expected_subdirectory) {
  if (tiled_root.empty()) {
    throw std::invalid_argument("tiled map root must not be empty");
  }
  // Tolerate a trailing separator so "/maps/site/tiled/" behaves like
  // "/maps/site/tiled".
  std::filesystem::path normalized = tiled_root.lexically_normal();
  while (normalized.filename().empty() && normalized.has_parent_path() &&
         normalized != normalized.parent_path()) {
    normalized = normalized.parent_path();
  }
  const std::string basename = normalized.filename().string();
  if (basename.empty()) {
    throw std::invalid_argument("tiled map root must name a directory: " +
                                tiled_root.string());
  }
  if (!expected_subdirectory.empty() && basename != expected_subdirectory) {
    throw std::invalid_argument("tiled map root is named '" + basename +
                                "' but the configuration expects '" +
                                expected_subdirectory + "'");
  }
  return MapArtifactLayout(normalized.parent_path(), basename);
}

std::filesystem::path MapArtifactLayout::GlobalMap() const {
  return root_ / kGlobalMapName;
}

std::filesystem::path MapArtifactLayout::GroundMap() const {
  return root_ / kGroundMapName;
}

std::filesystem::path MapArtifactLayout::ScanContextDb() const {
  return root_ / kScanContextDbName;
}

std::filesystem::path MapArtifactLayout::Trajectory() const {
  return root_ / kTrajectoryName;
}

std::filesystem::path MapArtifactLayout::Transformations() const {
  return root_ / kTransformationsName;
}

std::filesystem::path MapArtifactLayout::CornerMap() const {
  return root_ / kCornerMapName;
}

std::filesystem::path MapArtifactLayout::SurfaceMap() const {
  return root_ / kSurfaceMapName;
}

std::filesystem::path MapArtifactLayout::TiledRoot() const {
  return root_ / tile_subdirectory_;
}

std::filesystem::path MapArtifactLayout::TiledManifest() const {
  return TiledRoot() / kManifestName;
}

std::filesystem::path MapArtifactLayout::SavedCornerCloud() const {
  return root_ / kSavedCornerCloudName;
}

std::filesystem::path MapArtifactLayout::SavedSurfaceCloud() const {
  return root_ / kSavedSurfaceCloudName;
}

std::filesystem::path MapArtifactLayout::SavedGlobalCloud() const {
  return root_ / kSavedGlobalCloudName;
}

std::filesystem::path MapArtifactLayout::SavedGroundCloud() const {
  return root_ / kSavedGroundCloudName;
}
