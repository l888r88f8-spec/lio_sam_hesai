#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "localization_map/tile_index.h"

using localization_map::TileIndex;
using localization_map::TileIndexForPosition;

TEST(TileIndex, ComputesCoordinatesAtTileBoundaries) {
  EXPECT_EQ(TileIndexForPosition(0.0, 0.0, 100.0, 0.0, 0.0),
            (TileIndex{0, 0}));
  EXPECT_EQ(TileIndexForPosition(99.999, 100.0, 100.0, 0.0, 0.0),
            (TileIndex{0, 1}));
}

TEST(TileIndex, FloorsNegativeCoordinates) {
  EXPECT_EQ(TileIndexForPosition(-0.001, -100.001, 100.0, 0.0, 0.0),
            (TileIndex{-1, -2}));
}

TEST(TileIndex, AppliesOriginsBeforeFlooring) {
  EXPECT_EQ(TileIndexForPosition(49.0, -51.0, 100.0, 50.0, -50.0),
            (TileIndex{-1, -1}));
}

TEST(TileIndex, RejectsInvalidTileSizesAndCoordinates) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();

  EXPECT_THROW(TileIndexForPosition(0.0, 0.0, nan, 0.0, 0.0),
               std::invalid_argument);
  EXPECT_THROW(TileIndexForPosition(0.0, 0.0, infinity, 0.0, 0.0),
               std::invalid_argument);
  EXPECT_THROW(TileIndexForPosition(0.0, 0.0, 0.0, 0.0, 0.0),
               std::invalid_argument);
  EXPECT_THROW(TileIndexForPosition(0.0, 0.0, -1.0, 0.0, 0.0),
               std::invalid_argument);
  EXPECT_THROW(TileIndexForPosition(nan, 0.0, 100.0, 0.0, 0.0),
               std::invalid_argument);
  EXPECT_THROW(TileIndexForPosition(infinity, 0.0, 100.0, 0.0, 0.0),
               std::invalid_argument);
  EXPECT_THROW(TileIndexForPosition(0.0, nan, 100.0, 0.0, 0.0),
               std::invalid_argument);
  EXPECT_THROW(TileIndexForPosition(0.0, 0.0, 100.0, infinity, 0.0),
               std::invalid_argument);
}

TEST(TileIndex, RejectsOutOfRangeCoordinateCalculations) {
  const double maximum = std::numeric_limits<double>::max();

  EXPECT_THROW(TileIndexForPosition(maximum, 0.0, 1.0, 0.0, 0.0),
               std::invalid_argument);
  EXPECT_THROW(TileIndexForPosition(-maximum, 0.0, 1.0, 0.0, 0.0),
               std::invalid_argument);
  EXPECT_THROW(TileIndexForPosition(maximum, 0.0, 1.0, -maximum, 0.0),
               std::invalid_argument);
}

TEST(TileIndex, BuildsSquareNeighborhoodInAscendingOrder) {
  const std::vector<TileIndex> neighborhood =
      localization_map::SquareNeighborhood(TileIndex{2, -3}, 1);

  EXPECT_EQ(neighborhood, (std::vector<TileIndex>{
    {1, -4}, {1, -3}, {1, -2},
    {2, -4}, {2, -3}, {2, -2},
    {3, -4}, {3, -3}, {3, -2},
  }));
}

TEST(TileIndex, RejectsUnrepresentableSquareNeighborhoods) {
  EXPECT_THROW(localization_map::SquareNeighborhood(
                   TileIndex{std::numeric_limits<int>::max(), 0}, 1),
               std::invalid_argument);
  EXPECT_THROW(localization_map::SquareNeighborhood(
                   TileIndex{std::numeric_limits<int>::min(), 0}, 1),
               std::invalid_argument);
  EXPECT_THROW(localization_map::SquareNeighborhood(
                   TileIndex{0, 0}, std::numeric_limits<int>::max()),
               std::invalid_argument);
  EXPECT_THROW(localization_map::SquareNeighborhood(TileIndex{0, 0}, -1),
               std::invalid_argument);
}

TEST(TileIndex, BuildsAtMaximumCenterWithZeroRadius) {
  EXPECT_EQ(localization_map::SquareNeighborhood(
                TileIndex{std::numeric_limits<int>::max(),
                          std::numeric_limits<int>::max()},
                0),
            (std::vector<TileIndex>{{std::numeric_limits<int>::max(),
                                     std::numeric_limits<int>::max()}}));
}

TEST(TileIndex, FormatsNegativeTileFileStems) {
  EXPECT_EQ(localization_map::TileFileStem(TileIndex{-2, 3}), "-2_3");
}

TEST(TileIndex, HashesEqualIndicesIdentically) {
  const TileIndex index{2, -3};
  EXPECT_EQ(localization_map::TileIndexHash{}(index),
            localization_map::TileIndexHash{}(TileIndex{2, -3}));

  std::unordered_map<TileIndex, int, localization_map::TileIndexHash> tiles;
  tiles.emplace(index, 42);
  EXPECT_EQ(tiles.at(TileIndex{2, -3}), 42);
}
