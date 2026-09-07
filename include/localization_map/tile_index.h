#ifndef LOCALIZATION_MAP__TILE_INDEX_H_
#define LOCALIZATION_MAP__TILE_INDEX_H_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace localization_map {

struct TileIndex {
  int x{0};
  int y{0};

  friend bool operator==(const TileIndex& lhs, const TileIndex& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y;
  }

  friend bool operator<(const TileIndex& lhs, const TileIndex& rhs) {
    return lhs.x < rhs.x || (lhs.x == rhs.x && lhs.y < rhs.y);
  }
};

struct TileIndexHash {
  std::size_t operator()(const TileIndex& index) const noexcept {
    const std::size_t x = std::hash<int>{}(index.x);
    const std::size_t y = std::hash<int>{}(index.y);
    return x ^ (y + 0x9e3779b9U + (x << 6U) + (x >> 2U));
  }
};

inline TileIndex TileIndexForPosition(double x, double y, double tile_size_m,
                                      double origin_x, double origin_y) {
  if (!std::isfinite(tile_size_m) || tile_size_m <= 0.0) {
    throw std::invalid_argument("tile_size_m must be finite and positive");
  }
  if (!std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(origin_x) || !std::isfinite(origin_y)) {
    throw std::invalid_argument("tile position and origin must be finite");
  }

  const auto tile_coordinate = [tile_size_m](double position, double origin) {
    const double normalized = (position - origin) / tile_size_m;
    const double floored = std::floor(normalized);
    if (!std::isfinite(normalized) || !std::isfinite(floored) ||
        floored < static_cast<double>(std::numeric_limits<int>::min()) ||
        floored > static_cast<double>(std::numeric_limits<int>::max())) {
      throw std::invalid_argument("tile index is outside int range");
    }
    return static_cast<int>(floored);
  };

  return {tile_coordinate(x, origin_x), tile_coordinate(y, origin_y)};
}

inline std::vector<TileIndex> SquareNeighborhood(const TileIndex& center,
                                                 int radius) {
  if (radius < 0) {
    throw std::invalid_argument("radius must be non-negative");
  }

  const std::int64_t radius_wide = static_cast<std::int64_t>(radius);
  const std::int64_t minimum_x =
      static_cast<std::int64_t>(center.x) - radius_wide;
  const std::int64_t maximum_x =
      static_cast<std::int64_t>(center.x) + radius_wide;
  const std::int64_t minimum_y =
      static_cast<std::int64_t>(center.y) - radius_wide;
  const std::int64_t maximum_y =
      static_cast<std::int64_t>(center.y) + radius_wide;
  if (minimum_x < std::numeric_limits<int>::min() ||
      maximum_x > std::numeric_limits<int>::max() ||
      minimum_y < std::numeric_limits<int>::min() ||
      maximum_y > std::numeric_limits<int>::max()) {
    throw std::invalid_argument("tile neighborhood is not representable");
  }

  const std::uint64_t width =
      static_cast<std::uint64_t>(maximum_x - minimum_x + 1);
  if (width > std::numeric_limits<std::uint64_t>::max() / width) {
    throw std::invalid_argument("tile neighborhood is not representable");
  }
  const std::uint64_t count = width * width;
  std::vector<TileIndex> neighborhood;
  if (count > std::numeric_limits<std::size_t>::max() ||
      count > neighborhood.max_size()) {
    throw std::invalid_argument("tile neighborhood is not representable");
  }
  neighborhood.reserve(static_cast<std::size_t>(count));
  for (std::int64_t x = minimum_x; x <= maximum_x; ++x) {
    for (std::int64_t y = minimum_y; y <= maximum_y; ++y) {
      neighborhood.push_back({static_cast<int>(x), static_cast<int>(y)});
    }
  }
  return neighborhood;
}

inline std::string TileFileStem(const TileIndex& index) {
  return std::to_string(index.x) + "_" + std::to_string(index.y);
}

}  // namespace localization_map

#endif  // LOCALIZATION_MAP__TILE_INDEX_H_
