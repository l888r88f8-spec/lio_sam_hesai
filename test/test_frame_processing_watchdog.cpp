#include "runtime/frame_processing_watchdog.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>

namespace
{

TEST(FrameProcessingWatchdog, ExceedsOnlyAfterConfiguredTimeout)
{
  runtime::FrameProcessingWatchdog watchdog(500000U);
  watchdog.Begin(1000000U);

  EXPECT_FALSE(watchdog.Exceeded(1500000U));
  EXPECT_TRUE(watchdog.Exceeded(1500001U));
}

TEST(FrameProcessingWatchdog, CountsDroppedTimeoutFrames)
{
  runtime::FrameProcessingWatchdog watchdog(500000U);
  watchdog.Begin(1000000U);
  watchdog.Complete(1500001U, true);

  const auto snapshot = watchdog.Snapshot();
  EXPECT_FALSE(snapshot.active);
  EXPECT_TRUE(snapshot.last_frame_timed_out);
  EXPECT_EQ(snapshot.last_duration_us, 500001U);
  EXPECT_EQ(snapshot.timeout_count, 1U);
  EXPECT_EQ(snapshot.dropped_count, 1U);
}

TEST(FrameProcessingWatchdog, NormalFrameClearsLastTimeoutAndRetainsTotals)
{
  runtime::FrameProcessingWatchdog watchdog(500000U);
  watchdog.Begin(1000000U);
  watchdog.Complete(1500001U, true);
  watchdog.Begin(2000000U);
  watchdog.Complete(2100000U, false);

  const auto snapshot = watchdog.Snapshot();
  EXPECT_FALSE(snapshot.last_frame_timed_out);
  EXPECT_EQ(snapshot.last_duration_us, 100000U);
  EXPECT_EQ(snapshot.timeout_count, 1U);
  EXPECT_EQ(snapshot.dropped_count, 1U);
}

TEST(FrameProcessingWatchdog, RejectsZeroTimeoutAndOverlappingFrames)
{
  EXPECT_THROW(runtime::FrameProcessingWatchdog watchdog(0U), std::invalid_argument);

  runtime::FrameProcessingWatchdog watchdog(500000U);
  watchdog.Begin(1000000U);
  EXPECT_THROW(watchdog.Begin(1000001U), std::invalid_argument);
}

}  // namespace
