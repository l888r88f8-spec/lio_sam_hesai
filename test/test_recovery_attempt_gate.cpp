#include "localization_health/recovery_attempt_gate.h"
#include "localization_health/localization_state_machine.h"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace {

using localization_health::LocalizationState;
using localization_health::LocalizationStateMachine;
using localization_health::RecoveryAttemptGate;
using localization_health::RecoveryAttemptReason;
using localization_health::StreamHealth;

StreamHealth HealthyStream(bool has_new_observation = true) {
  StreamHealth stream;
  stream.lidar_rate_hz = 10.0;
  stream.imu_rate_hz = 100.0;
  stream.rates_known = true;
  stream.has_new_observation = has_new_observation;
  return stream;
}

StreamHealth UnknownRateHealthyStream(bool has_new_observation = true) {
  StreamHealth stream;
  stream.rates_known = false;
  stream.has_new_observation = has_new_observation;
  return stream;
}

localization_health::HealthInputs HealthyTrackingInput() {
  localization_health::HealthInputs input;
  input.has_imu_init = true;
  input.map_ready = true;
  input.has_init = true;
  input.is_tracking = true;
  input.has_new_result = true;
  input.has_new_stream_observation = true;
  input.reg_fitness = 0.1F;
  input.stream = HealthyStream();
  return input;
}

TEST(RecoveryAttemptGate, ReturnsLostEntryOnlyOnTransitionIntoLost) {
  RecoveryAttemptGate gate(RecoveryAttemptGate::Config{});

  EXPECT_EQ(gate.Update(LocalizationState::kTracking, HealthyStream()),
            RecoveryAttemptReason::kNone);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kLostEntry);
}

TEST(RecoveryAttemptGate, DoesNotRepeatLostEntryOnContinuingHealthyStream) {
  RecoveryAttemptGate gate(RecoveryAttemptGate::Config{});

  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kLostEntry);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kNone);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kNone);
}

TEST(RecoveryAttemptGate, LidarStallRetriesOnceAfterHealthyNewObservation) {
  RecoveryAttemptGate gate(RecoveryAttemptGate::Config{});

  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kLostEntry);

  auto stalled = HealthyStream();
  stalled.lidar_stalled = true;
  EXPECT_EQ(gate.Update(LocalizationState::kLost, stalled),
            RecoveryAttemptReason::kNone);

  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kStreamRecovered);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kNone);
}

TEST(RecoveryAttemptGate,
     SystemOrderingPreservesStreamRecoveredWhenLostStateExits) {
  LocalizationStateMachine machine(LocalizationStateMachine::Config{});
  RecoveryAttemptGate gate(RecoveryAttemptGate::Config{});

  auto healthy = HealthyTrackingInput();
  auto report = machine.Update(healthy);
  ASSERT_EQ(report.state, LocalizationState::kRelocalizing);
  EXPECT_EQ(gate.Update(report.state, healthy.stream),
            RecoveryAttemptReason::kNone);
  report = machine.Update(healthy);
  ASSERT_EQ(report.state, LocalizationState::kTracking);
  EXPECT_EQ(gate.Update(report.state, healthy.stream),
            RecoveryAttemptReason::kNone);

  auto stalled = healthy;
  stalled.stream.lidar_stalled = true;
  report = machine.Update(stalled);
  ASSERT_EQ(report.state, LocalizationState::kDegraded);
  EXPECT_EQ(gate.Update(report.state, stalled.stream),
            RecoveryAttemptReason::kNone);
  report = machine.Update(stalled);
  ASSERT_EQ(report.state, LocalizationState::kLost);
  EXPECT_EQ(gate.Update(report.state, stalled.stream),
            RecoveryAttemptReason::kLostEntry);

  const LocalizationState state_before_health_update = machine.state();
  ASSERT_EQ(state_before_health_update, LocalizationState::kLost);
  EXPECT_EQ(gate.Update(state_before_health_update, healthy.stream),
            RecoveryAttemptReason::kStreamRecovered);
  report = machine.Update(healthy);
  ASSERT_EQ(report.state, LocalizationState::kRelocalizing);
  EXPECT_EQ(gate.Update(report.state, healthy.stream),
            RecoveryAttemptReason::kNone);
}

TEST(RecoveryAttemptGate,
     TimestampJumpWaitsForHealthySnapshotWithNewObservation) {
  RecoveryAttemptGate gate(RecoveryAttemptGate::Config{});

  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kLostEntry);

  auto jumped = HealthyStream();
  jumped.timestamp_jump = true;
  EXPECT_EQ(gate.Update(LocalizationState::kLost, jumped),
            RecoveryAttemptReason::kNone);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream(false)),
            RecoveryAttemptReason::kNone);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kStreamRecovered);
}

TEST(RecoveryAttemptGate, LeavingLostResetsStateForNextLostEntry) {
  RecoveryAttemptGate gate(RecoveryAttemptGate::Config{});

  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kLostEntry);
  EXPECT_EQ(gate.Update(LocalizationState::kTracking, HealthyStream()),
            RecoveryAttemptReason::kNone);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kLostEntry);
}

TEST(RecoveryAttemptGate, ResetAllowsAnotherLostEntry) {
  RecoveryAttemptGate gate(RecoveryAttemptGate::Config{});

  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kLostEntry);
  gate.Reset();
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kLostEntry);
}

TEST(RecoveryAttemptGate, TreatsSlowKnownLidarRateAsStreamFault) {
  RecoveryAttemptGate gate(RecoveryAttemptGate::Config{});

  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kLostEntry);

  auto slow_lidar = HealthyStream();
  slow_lidar.lidar_rate_hz = 4.9;
  EXPECT_EQ(gate.Update(LocalizationState::kLost, slow_lidar),
            RecoveryAttemptReason::kNone);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kStreamRecovered);
}

TEST(RecoveryAttemptGate, TreatsTimestampRollbackAsStreamFault) {
  RecoveryAttemptGate gate(RecoveryAttemptGate::Config{});

  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kLostEntry);

  auto rollback = HealthyStream();
  rollback.timestamp_monotonic = false;
  EXPECT_EQ(gate.Update(LocalizationState::kLost, rollback),
            RecoveryAttemptReason::kNone);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kStreamRecovered);
}

TEST(RecoveryAttemptGate, UnknownRatesCanRecoverAfterImuStall) {
  RecoveryAttemptGate gate(RecoveryAttemptGate::Config{});

  EXPECT_EQ(gate.Update(LocalizationState::kLost, UnknownRateHealthyStream()),
            RecoveryAttemptReason::kLostEntry);

  auto stalled = UnknownRateHealthyStream();
  stalled.imu_stalled = true;
  EXPECT_EQ(gate.Update(LocalizationState::kLost, stalled),
            RecoveryAttemptReason::kNone);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, UnknownRateHealthyStream()),
            RecoveryAttemptReason::kStreamRecovered);
}

TEST(RecoveryAttemptGate, RetriesAfterFaultPresentOnLostEntry) {
  RecoveryAttemptGate gate(RecoveryAttemptGate::Config{});

  auto stalled = HealthyStream();
  stalled.lidar_stalled = true;
  EXPECT_EQ(gate.Update(LocalizationState::kLost, stalled),
            RecoveryAttemptReason::kLostEntry);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kStreamRecovered);
}

TEST(RecoveryAttemptGate, RejectsInvalidConfiguration) {
  auto config = RecoveryAttemptGate::Config{};
  config.nominal_lidar_rate_hz = 0.0;
  EXPECT_THROW(RecoveryAttemptGate gate(config), std::invalid_argument);

  config = RecoveryAttemptGate::Config{};
  config.nominal_imu_rate_hz = -1.0;
  EXPECT_THROW(RecoveryAttemptGate gate(config), std::invalid_argument);

  config = RecoveryAttemptGate::Config{};
  config.nominal_lidar_rate_hz = std::numeric_limits<double>::infinity();
  EXPECT_THROW(RecoveryAttemptGate gate(config), std::invalid_argument);

  config = RecoveryAttemptGate::Config{};
  config.nominal_imu_rate_hz = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(RecoveryAttemptGate gate(config), std::invalid_argument);

  config = RecoveryAttemptGate::Config{};
  config.min_stream_rate_ratio = 0.0;
  EXPECT_THROW(RecoveryAttemptGate gate(config), std::invalid_argument);

  config = RecoveryAttemptGate::Config{};
  config.min_stream_rate_ratio = 1.1;
  EXPECT_THROW(RecoveryAttemptGate gate(config), std::invalid_argument);
}

TEST(RecoveryAttemptGate, ConvertsReasonsToDiagnosticStrings) {
  EXPECT_STREQ(localization_health::ToString(RecoveryAttemptReason::kNone),
               "NONE");
  EXPECT_STREQ(
      localization_health::ToString(RecoveryAttemptReason::kLostEntry),
      "LOST_ENTRY");
  EXPECT_STREQ(
      localization_health::ToString(RecoveryAttemptReason::kStreamRecovered),
      "STREAM_RECOVERED");
}

}  // namespace
