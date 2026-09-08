#ifndef LOCALIZATION_MAP__TILE_MAP_GENERATOR_H_
#define LOCALIZATION_MAP__TILE_MAP_GENERATOR_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>

namespace localization_map {

struct TileMapGeneratorOptions {
  std::filesystem::path map_directory;
  std::filesystem::path output_subdirectory{"tiled"};
  double tile_size_m{100.0};
  std::array<double, 2> origin{0, 0};
  double overview_resolution_m{5.0};
  std::string frame_id{"map"};
};

struct TileMapGeneratorResult {
  std::uint64_t global_input_points{0};
  std::uint64_t global_output_points{0};
  std::uint64_t ground_input_points{0};
  std::uint64_t ground_output_points{0};
  std::size_t tile_count{0};
  std::filesystem::path output_directory;
};

TileMapGeneratorResult GenerateTileMap(const TileMapGeneratorOptions& options);

}  // namespace localization_map

#endif  // LOCALIZATION_MAP__TILE_MAP_GENERATOR_H_
