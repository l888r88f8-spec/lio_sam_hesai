#include "ivox_map/ivox_map.h"

#include <gtest/gtest.h>

#include <vector>

namespace {

PCLPointXYZI MakePoint(float x, float y, float z) {
  PCLPointXYZI point{};
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = 1.0F;
  return point;
}

}  // namespace

TEST(IVoxMapBatchAdd, ContinuesAfterNoDownsamplePoint) {
  IVoxMap::Options options;
  options.resolution_ = 1.0F;
  options.nearby_type_ = IVoxMap::NearbyType::CENTER;
  IVoxMap map(options);

  IVoxMap::PointVector world_points{
      MakePoint(0.0F, 0.0F, 0.0F),
      MakePoint(3.0F, 0.0F, 0.0F),
  };
  std::vector<IVoxMap::PointVector> nearest_points(world_points.size());
  nearest_points.front().push_back(MakePoint(2.0F, 2.0F, 2.0F));

  map.AddPoints(world_points, nearest_points);

  IVoxMap::PointVector closest_points;
  EXPECT_TRUE(map.GetClosestPoint(
      world_points.back(), closest_points, 1U, 0.1F));
}
