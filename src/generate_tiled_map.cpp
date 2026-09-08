#include <pcl/console/print.h>

#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "localization_map/tile_map_generator.h"

namespace {

constexpr int kUsageExitCode = 64;

const char* Usage() {
  return "usage: lio_sam_hesai_generate_tiled_map MAP_DIRECTORY [--output-subdirectory tiled] [--tile-size-m 100] [--origin-x 0] [--origin-y 0] [--overview-resolution-m 5] [--frame-id map]";
}

double ParseFiniteDouble(const std::string& text, const std::string& flag) {
  std::size_t consumed = 0;
  double value = 0.0;
  try {
    value = std::stod(text, &consumed);
  } catch (const std::exception&) {
    throw std::invalid_argument(flag + " requires a finite number");
  }
  if (consumed != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument(flag + " requires a finite number");
  }
  return value;
}

localization_map::TileMapGeneratorOptions ParseArguments(int argc,
                                                         char** argv) {
  if (argc < 2) {
    throw std::invalid_argument("MAP_DIRECTORY is required");
  }
  const std::string map_directory = argv[1];
  if (map_directory.empty() || map_directory.rfind("--", 0) == 0) {
    throw std::invalid_argument("MAP_DIRECTORY is required first");
  }

  localization_map::TileMapGeneratorOptions options;
  options.map_directory = std::filesystem::path(map_directory);
  for (int index = 2; index < argc; index += 2) {
    const std::string flag = argv[index];
    if (index + 1 >= argc) {
      throw std::invalid_argument(flag + " requires a value");
    }
    const std::string value = argv[index + 1];
    if (flag == "--output-subdirectory") {
      options.output_subdirectory = value;
    } else if (flag == "--tile-size-m") {
      options.tile_size_m = ParseFiniteDouble(value, flag);
    } else if (flag == "--origin-x") {
      options.origin[0] = ParseFiniteDouble(value, flag);
    } else if (flag == "--origin-y") {
      options.origin[1] = ParseFiniteDouble(value, flag);
    } else if (flag == "--overview-resolution-m") {
      options.overview_resolution_m = ParseFiniteDouble(value, flag);
    } else if (flag == "--frame-id") {
      options.frame_id = value;
    } else {
      throw std::invalid_argument("unknown flag: " + flag);
    }
  }
  return options;
}

}  // namespace

int main(int argc, char** argv) {
  localization_map::TileMapGeneratorOptions options;
  try {
    options = ParseArguments(argc, argv);
  } catch (const std::invalid_argument& error) {
    std::cerr << "error: " << error.what() << "; " << Usage() << '\n';
    return kUsageExitCode;
  }

  try {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    const localization_map::TileMapGeneratorResult result =
        localization_map::GenerateTileMap(options);
    std::cout << "generated " << result.output_directory << " tiles="
              << result.tile_count << " global=" << result.global_input_points
              << '/' << result.global_output_points << " ground="
              << result.ground_input_points << '/' << result.ground_output_points
              << '\n';
    return EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
