#include "localization_health/data_stream_watchdog.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>

namespace {

TEST(DataStreamWatchdog, ColdStartHasUnknownRatesAndFreshLidarIsNotStalled) {
  localization_health::DataStreamWatchdog watchdog(
      localization_health::DataStreamWatchdog::Config{});

  const auto cold_start = watchdog.Evaluate(0U);
  EXPECT_FALSE(cold_start.rates_known);

  watchdog.RecordLidar(100000U);
  const auto health = watchdog.Evaluate(100001U);
  EXPECT_FALSE(health.lidar_stalled);
  EXPECT_FALSE(health.rates_known);
}

TEST(DataStreamWatchdog, EstimatesTenHertzLidarRateWhenBothStreamsHaveSamples) {
  localization_health::DataStreamWatchdog watchdog(
      localization_health::DataStreamWatchdog::Config{});

  for (std::uint64_t stamp_us = 0U; stamp_us <= 400000U; stamp_us += 100000U) {
    watchdog.RecordLidar(stamp_us);
  }
  for (std::uint64_t stamp_us = 0U; stamp_us <= 400000U; stamp_us += 10000U) {
    watchdog.RecordImu(stamp_us);
  }

  const auto health = watchdog.Evaluate(400000U);
  EXPECT_DOUBLE_EQ(health.lidar_rate_hz, 10.0);
  EXPECT_DOUBLE_EQ(health.imu_rate_hz, 100.0);
  EXPECT_TRUE(health.rates_known);
}

TEST(DataStreamWatchdog, MarksLidarStalledOnlyAfterTimeout) {
  localization_health::DataStreamWatchdog watchdog(
      localization_health::DataStreamWatchdog::Config{});
  watchdog.RecordLidar(1000000U);

  EXPECT_FALSE(watchdog.Evaluate(1500000U).lidar_stalled);
  EXPECT_TRUE(watchdog.Evaluate(1500001U).lidar_stalled);
}

TEST(DataStreamWatchdog, UsesArrivalTimeForStallWhenSensorStampIsInFuture) {
  localization_health::DataStreamWatchdog watchdog(
      localization_health::DataStreamWatchdog::Config{});
  watchdog.RecordLidar(100000000U, 1000000U);

  EXPECT_FALSE(watchdog.Evaluate(1500000U).lidar_stalled);
  EXPECT_TRUE(watchdog.Evaluate(1500001U).lidar_stalled);
}

TEST(DataStreamWatchdog, RecoversMonotonicStateAfterNormalSampleFollowingRollback) {
  localization_health::DataStreamWatchdog watchdog(
      localization_health::DataStreamWatchdog::Config{});
  watchdog.RecordLidar(100U);
  watchdog.RecordLidar(99U);

  EXPECT_FALSE(watchdog.Evaluate(100U).timestamp_monotonic);

  watchdog.RecordLidar(100099U);
  EXPECT_TRUE(watchdog.Evaluate(100099U).timestamp_monotonic);
}

TEST(DataStreamWatchdog, ClearsTimestampJumpAfterNormalNominalInterval) {
  localization_health::DataStreamWatchdog watchdog(
      localization_health::DataStreamWatchdog::Config{});
  watchdog.RecordLidar(0U);
  watchdog.RecordLidar(1000000U);

  EXPECT_TRUE(watchdog.Evaluate(1000000U).timestamp_jump);

  watchdog.RecordLidar(1100000U);
  EXPECT_FALSE(watchdog.Evaluate(1100000U).timestamp_jump);
}

TEST(DataStreamWatchdog, LatchesJumpUntilEvaluateAfterNormalSample) {
  localization_health::DataStreamWatchdog watchdog(
      localization_health::DataStreamWatchdog::Config{});
  watchdog.RecordLidar(0U);
  watchdog.RecordLidar(1000000U);
  watchdog.RecordLidar(1100000U);

  EXPECT_TRUE(watchdog.Evaluate(1100000U).timestamp_jump);
  EXPECT_FALSE(watchdog.Evaluate(1100000U).timestamp_jump);
}

TEST(DataStreamWatchdog, StartsNewRateWindowAtForwardJumpTimestamp) {
  auto config = localization_health::DataStreamWatchdog::Config{};
  config.min_samples = 4U;
  localization_health::DataStreamWatchdog watchdog(config);
  watchdog.RecordLidar(0U);
  watchdog.RecordLidar(100000U);
  watchdog.RecordLidar(200000U);

  watchdog.RecordLidar(1000000U);
  watchdog.RecordLidar(1100000U);
  watchdog.RecordLidar(1200000U);
  watchdog.RecordLidar(1300000U);

  const auto health = watchdog.Evaluate(1300000U);
  EXPECT_TRUE(health.timestamp_jump);
  EXPECT_DOUBLE_EQ(health.lidar_rate_hz, 10.0);
  EXPECT_FALSE(watchdog.Evaluate(1300000U).timestamp_jump);
}

TEST(DataStreamWatchdog, LatchesRollbackUntilEvaluateAfterNormalSample) {
  localization_health::DataStreamWatchdog watchdog(
      localization_health::DataStreamWatchdog::Config{});
  watchdog.RecordLidar(100000U);
  watchdog.RecordLidar(10000U);
  watchdog.RecordLidar(110000U);

  EXPECT_FALSE(watchdog.Evaluate(110000U).timestamp_monotonic);
  EXPECT_TRUE(watchdog.Evaluate(110000U).timestamp_monotonic);
}

TEST(DataStreamWatchdog, ConsumesObservationAndAnomalyInSameSnapshot) {
  localization_health::DataStreamWatchdog watchdog(
      localization_health::DataStreamWatchdog::Config{});
  watchdog.RecordLidar(0U);
  watchdog.RecordLidar(1000000U);

  const auto anomaly = watchdog.Evaluate(1000000U);
  EXPECT_TRUE(anomaly.has_new_observation);
  EXPECT_TRUE(anomaly.timestamp_jump);

  const auto consumed = watchdog.Evaluate(1000000U);
  EXPECT_FALSE(consumed.has_new_observation);
  EXPECT_FALSE(consumed.timestamp_jump);

  watchdog.RecordLidar(1100000U);
  const auto normal = watchdog.Evaluate(1100000U);
  EXPECT_TRUE(normal.has_new_observation);
  EXPECT_FALSE(normal.timestamp_jump);
}

TEST(DataStreamWatchdog, RejectsInvalidConfiguration) {
  auto config = localization_health::DataStreamWatchdog::Config{};
  config.min_samples = 0U;
  EXPECT_THROW(localization_health::DataStreamWatchdog watchdog(config),
               std::invalid_argument);

  config.min_samples = 1U;
  EXPECT_THROW(localization_health::DataStreamWatchdog watchdog(config),
               std::invalid_argument);

  config = localization_health::DataStreamWatchdog::Config{};
  config.window_size = config.min_samples - 1U;
  EXPECT_THROW(localization_health::DataStreamWatchdog watchdog(config),
               std::invalid_argument);

  config = localization_health::DataStreamWatchdog::Config{};
  config.nominal_lidar_period_us = 0U;
  EXPECT_THROW(localization_health::DataStreamWatchdog watchdog(config),
               std::invalid_argument);

  config = localization_health::DataStreamWatchdog::Config{};
  config.nominal_imu_period_us = 0U;
  EXPECT_THROW(localization_health::DataStreamWatchdog watchdog(config),
               std::invalid_argument);

  config = localization_health::DataStreamWatchdog::Config{};
  config.jump_period_multiplier = std::numeric_limits<double>::infinity();
  EXPECT_THROW(localization_health::DataStreamWatchdog watchdog(config),
               std::invalid_argument);

  config.jump_period_multiplier = 0.0;
  EXPECT_THROW(localization_health::DataStreamWatchdog watchdog(config),
               std::invalid_argument);
}

TEST(DataStreamWatchdog, StartsNewRateWindowAtRollbackTimestamp) {
  auto config = localization_health::DataStreamWatchdog::Config{};
  config.window_size = 3U;
  config.min_samples = 2U;
  localization_health::DataStreamWatchdog watchdog(config);
  watchdog.RecordLidar(100000U);
  watchdog.RecordLidar(200000U);
  watchdog.RecordLidar(10000U);
  watchdog.RecordLidar(110000U);

  EXPECT_DOUBLE_EQ(watchdog.Evaluate(110000U).lidar_rate_hz, 10.0);
}

TEST(DataStreamWatchdog, RetainsCorrectRateAfterWindowEvictsOldSamples) {
  auto config = localization_health::DataStreamWatchdog::Config{};
  config.window_size = 3U;
  config.min_samples = 2U;
  localization_health::DataStreamWatchdog watchdog(config);
  watchdog.RecordLidar(0U);
  watchdog.RecordLidar(1U);
  watchdog.RecordLidar(100001U);
  watchdog.RecordLidar(200001U);

  EXPECT_DOUBLE_EQ(watchdog.Evaluate(200001U).lidar_rate_hz, 10.0);
}

TEST(DataStreamWatchdog, KeepsRatesUnknownUntilBothStreamsReachMinimumSamples) {
  auto config = localization_health::DataStreamWatchdog::Config{};
  config.min_samples = 2U;
  localization_health::DataStreamWatchdog watchdog(config);
  watchdog.RecordLidar(0U);
  watchdog.RecordLidar(100000U);
  watchdog.RecordImu(0U);

  EXPECT_FALSE(watchdog.Evaluate(100000U).rates_known);
}

}  // namespace
