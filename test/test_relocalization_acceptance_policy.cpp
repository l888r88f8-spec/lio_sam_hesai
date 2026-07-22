#include "relocalization/acceptance_policy.h"

#include <gtest/gtest.h>

#include <limits>

TEST(RelocalizationAcceptancePolicy, AcceptsOnlyFiniteFitnessBelowThreshold) {
  EXPECT_TRUE(ShouldEarlyAcceptRelocalization(0.2F, 0.3));
  EXPECT_FALSE(ShouldEarlyAcceptRelocalization(0.3F, 0.3));
  EXPECT_FALSE(ShouldEarlyAcceptRelocalization(0.4F, 0.3));
  EXPECT_FALSE(ShouldEarlyAcceptRelocalization(
      std::numeric_limits<float>::infinity(), 0.3));
  EXPECT_FALSE(ShouldEarlyAcceptRelocalization(
      std::numeric_limits<float>::quiet_NaN(), 0.3));
}

TEST(RelocalizationAcceptancePolicy, NonPositiveThresholdDisablesEarlyAccept) {
  EXPECT_FALSE(ShouldEarlyAcceptRelocalization(0.0F, 0.0));
  EXPECT_FALSE(ShouldEarlyAcceptRelocalization(0.0F, -1.0));
}
