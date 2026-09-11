#ifndef LIO_SAM_HESAI_COMMON_MAP_ARTIFACT_LAYOUT_H
#define LIO_SAM_HESAI_COMMON_MAP_ARTIFACT_LAYOUT_H

#include <filesystem>
#include <string>

// Single source of truth for the on-disk map artifact layout:
//
//   <root>/GlobalMap.pcd
//   <root>/GroundMap.pcd
//   <root>/sc_db.bin
//   <root>/trajectory.pcd
//   <root>/transformations.pcd
//   <root>/CornerMap.pcd
//   <root>/SurfMap.pcd
//   <root>/<tile_subdirectory>/manifest.yaml
//
// P2.2 does not rename any of these files. The map savers, the full-map
// loader, the tile generator and the Scan Context loader all derive their
// paths from this object rather than concatenating file names themselves, so
// the layout and the manifest content cannot drift apart.
class MapArtifactLayout {
 public:
  static constexpr const char* kGlobalMapName = "GlobalMap.pcd";
  static constexpr const char* kGroundMapName = "GroundMap.pcd";
  static constexpr const char* kScanContextDbName = "sc_db.bin";
  static constexpr const char* kTrajectoryName = "trajectory.pcd";
  static constexpr const char* kTransformationsName = "transformations.pcd";
  static constexpr const char* kCornerMapName = "CornerMap.pcd";
  static constexpr const char* kSurfaceMapName = "SurfMap.pcd";
  static constexpr const char* kManifestName = "manifest.yaml";
  static constexpr const char* kDefaultTileSubdirectory = "tiled";

  // Written only by the shutdown auto-save. Nothing reads these back, but P2.2
  // does not rename on-disk artifacts, so they keep their historical names and
  // still come from here instead of being concatenated at the call site.
  static constexpr const char* kSavedCornerCloudName = "cloudCorner.pcd";
  static constexpr const char* kSavedSurfaceCloudName = "cloudSurf.pcd";
  static constexpr const char* kSavedGlobalCloudName = "cloudGlobal.pcd";
  static constexpr const char* kSavedGroundCloudName = "cloudGround.pcd";

  // Stores a normalized root. Throws std::invalid_argument for an empty root
  // or an empty tile subdirectory, before any I/O happens.
  explicit MapArtifactLayout(
      std::filesystem::path root,
      std::string tile_subdirectory = kDefaultTileSubdirectory);

  // Derives the layout from a global map file. Rejects an empty path or a file
  // not named GlobalMap.pcd rather than silently retargeting the artifacts.
  static MapArtifactLayout FromGlobalMap(
      const std::filesystem::path& global_map);

  // Derives the layout from an existing tiled root. When
  // `expected_subdirectory` is non-empty the configured basename is validated
  // against it; it is never forced to "tiled".
  static MapArtifactLayout FromTiledRoot(
      const std::filesystem::path& tiled_root,
      const std::string& expected_subdirectory = std::string());

  [[nodiscard]] const std::filesystem::path& Root() const { return root_; }
  [[nodiscard]] std::filesystem::path GlobalMap() const;
  [[nodiscard]] std::filesystem::path GroundMap() const;
  [[nodiscard]] std::filesystem::path ScanContextDb() const;
  [[nodiscard]] std::filesystem::path Trajectory() const;
  [[nodiscard]] std::filesystem::path Transformations() const;
  [[nodiscard]] std::filesystem::path CornerMap() const;
  [[nodiscard]] std::filesystem::path SurfaceMap() const;
  [[nodiscard]] std::filesystem::path TiledRoot() const;
  [[nodiscard]] std::filesystem::path TiledManifest() const;
  [[nodiscard]] std::filesystem::path SavedCornerCloud() const;
  [[nodiscard]] std::filesystem::path SavedSurfaceCloud() const;
  [[nodiscard]] std::filesystem::path SavedGlobalCloud() const;
  [[nodiscard]] std::filesystem::path SavedGroundCloud() const;

 private:
  std::filesystem::path root_;
  std::filesystem::path tile_subdirectory_;
};

#endif  // LIO_SAM_HESAI_COMMON_MAP_ARTIFACT_LAYOUT_H
