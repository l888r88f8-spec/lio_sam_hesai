#include "localization_health/log_throttle.h"

#include <gtest/gtest.h>

#include <cstdint>

namespace {

using localization_health::SteadyLogThrottle;

TEST(SteadyLogThrottle, EmitsFirstSampleThenHonorsSteadyInterval) {
  SteadyLogThrottle throttle(1000000U);

  EXPECT_TRUE(throttle.ShouldLog(100000U));
  EXPECT_FALSE(throttle.ShouldLog(1099999U));
  EXPECT_TRUE(throttle.ShouldLog(1100000U));
  EXPECT_FALSE(throttle.ShouldLog(1100001U));
}

TEST(SteadyLogThrottle, InstancesDoNotShareLimiterState) {
  SteadyLogThrottle first(1000000U);
  SteadyLogThrottle second(1000000U);

  EXPECT_TRUE(first.ShouldLog(500000U));
  EXPECT_TRUE(second.ShouldLog(500000U));
  EXPECT_FALSE(first.ShouldLog(700000U));
  EXPECT_FALSE(second.ShouldLog(700000U));
}

TEST(SteadyLogThrottle, SteadyClockGoingBackwardsDoesNotSuppressForever) {
  SteadyLogThrottle throttle(1000000U);

  EXPECT_TRUE(throttle.ShouldLog(2000000U));
  EXPECT_TRUE(throttle.ShouldLog(1500000U));
  EXPECT_FALSE(throttle.ShouldLog(2400000U));
  EXPECT_TRUE(throttle.ShouldLog(2500000U));
}

TEST(SteadyLogThrottle, ResetAllowsImmediateEmission) {
  SteadyLogThrottle throttle(1000000U);

  EXPECT_TRUE(throttle.ShouldLog(100000U));
  EXPECT_FALSE(throttle.ShouldLog(200000U));
  throttle.Reset();
  EXPECT_TRUE(throttle.ShouldLog(200000U));
}

}  // namespace
