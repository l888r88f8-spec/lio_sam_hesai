#include <gtest/gtest.h>

#include "lio_sam/ground_grid_patcher.h"

namespace {

lio_sam_hesai::GroundGridPointType makePoint(float x, float y, float z)
{
    lio_sam_hesai::GroundGridPointType point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = 1.0f;
    return point;
}

lio_sam_hesai::GroundGridPatchOptions testOptions()
{
    lio_sam_hesai::GroundGridPatchOptions options;
    options.resolution = 1.0f;
    options.search_radius = 1.5f;
    options.max_slope_deg = 15.0f;
    options.plane_distance_threshold = 0.03f;
    options.max_height_range = 0.08f;
    options.max_roughness = 0.04f;
    options.max_prediction_margin = 0.05f;
    options.min_neighbor_cells = 6;
    options.min_support_quadrants = 3;
    return options;
}

}  // namespace

TEST(GroundGridPatcher, FillsInteriorHoleOnSmoothGround)
{
    pcl::PointCloud<lio_sam_hesai::GroundGridPointType> ground;
    for (int x = -1; x <= 1; ++x)
    {
        for (int y = -1; y <= 1; ++y)
        {
            if (x == 0 && y == 0)
                continue;
            ground.push_back(makePoint(x + 0.5f, y + 0.5f, 0.1f * x));
        }
    }

    const auto patches =
        lio_sam_hesai::generateGroundGridPatches(ground, testOptions());

    ASSERT_EQ(patches.size(), 1U);
    EXPECT_NEAR(patches.front().x, 0.5f, 1e-4f);
    EXPECT_NEAR(patches.front().y, 0.5f, 1e-4f);
    EXPECT_NEAR(patches.front().z, 0.0f, 1e-4f);
}

TEST(GroundGridPatcher, DoesNotExtendOpenMapBoundary)
{
    pcl::PointCloud<lio_sam_hesai::GroundGridPointType> ground;
    for (int x = -3; x <= -1; ++x)
    {
        for (int y = -1; y <= 1; ++y)
            ground.push_back(makePoint(x + 0.5f, y + 0.5f, 0.0f));
    }

    const auto patches =
        lio_sam_hesai::generateGroundGridPatches(ground, testOptions());

    EXPECT_TRUE(patches.empty());
}
