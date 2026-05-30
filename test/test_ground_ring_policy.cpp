#include <gtest/gtest.h>

#include "lio_sam/ground_ring_policy.h"

TEST(GroundRingPolicy, UsesAllRingsForGroundClassification)
{
    const auto classification_range =
        lio_sam_hesai::fullGroundClassificationRange(16);

    EXPECT_EQ(classification_range.start, 0);
    EXPECT_EQ(classification_range.end, 15);
}

TEST(GroundRingPolicy, KeepsGroundOutputLimitedToConfiguredRange)
{
    const auto output_range =
        lio_sam_hesai::normalizeGroundScanRange(11, 15, 16);

    EXPECT_FALSE(lio_sam_hesai::isGroundOutputRing(10, output_range));
    EXPECT_TRUE(lio_sam_hesai::isGroundOutputRing(11, output_range));
    EXPECT_TRUE(lio_sam_hesai::isGroundOutputRing(15, output_range));
    EXPECT_FALSE(lio_sam_hesai::isGroundOutputRing(16, output_range));
}

TEST(GroundRingPolicy, TreatsNegativeConfiguredEndAsLastScan)
{
    const auto output_range =
        lio_sam_hesai::normalizeGroundScanRange(3, -1, 16);

    EXPECT_EQ(output_range.start, 3);
    EXPECT_EQ(output_range.end, 15);
}
