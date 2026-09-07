#ifndef LOCALIZATION_MAP__TILE_MANIFEST_H_
#define LOCALIZATION_MAP__TILE_MANIFEST_H_

#include <array>
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>

#include "localization_map/tile_index.h"

namespace localization_map {

struct TileFileMetadata {
  std::filesystem::path path;
  std::uint64_t point_count{0};
  std::uintmax_t bytes{0};
};

struct OverviewMetadata {
  std::filesystem::path path;
  double resolution_m{0.0};
};

struct SourceMetadata {
  std::string global_map_name;
  std::uintmax_t global_map_bytes{0};
  std::string ground_map_name;
  std::uintmax_t ground_map_bytes{0};
};

struct TileManifest {
  static constexpr int kSchemaVersion = 1;

  int schema_version{kSchemaVersion};
  std::string frame_id{"map"};
  std::string point_type{"XYZI"};
  double tile_size_m{100.0};
  std::array<double, 2> origin{0, 0};
  SourceMetadata source;
  std::map<TileIndex, TileFileMetadata> global_tiles;
  std::map<TileIndex, TileFileMetadata> ground_tiles;
  std::optional<OverviewMetadata> overview;

  // The published root artifact must remain immutable while this manifest or
  // any provider built from it is in use. On-demand file openers must still
  // revalidate containment and file identity when acquiring tile data.
  static TileManifest Load(const std::filesystem::path& root);
  void Save(const std::filesystem::path& root) const;
};

}  // namespace localization_map

#endif  // LOCALIZATION_MAP__TILE_MANIFEST_H_
