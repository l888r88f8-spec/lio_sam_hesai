#include "localization_health/localization_state_machine.h"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace {

using localization_health::HealthInputs;
using localization_health::HealthLevel;
using localization_health::LocalizationState;
using localization_health::LocalizationStateMachine;

HealthInputs HealthyTrackingInput() {
  HealthInputs input;
  input.has_imu_init = true;
  input.map_ready = true;
  input.has_init = true;
  input.is_tracking = true;
  input.has_new_result = true;
  input.has_new_stream_observation = true;
  input.reg_fitness = 0.1F;
  input.stream.rates_known = true;
  input.stream.lidar_rate_hz = 10.0;
  input.stream.imu_rate_hz = 100.0;
  return input;
}

LocalizationStateMachine::Config FastConfig() {
  LocalizationStateMachine::Config config;
  config.degrade_enter_frames = 2;
  config.recover_frames = 3;
  config.lost_enter_frames = 3;
  return config;
}

LocalizationStateMachine MakeTrackingMachine(
    const LocalizationStateMachine::Config& config = FastConfig()) {
  LocalizationStateMachine machine(config);
  EXPECT_EQ(machine.Update(HealthyTrackingInput()).state,
            LocalizationState::kRelocalizing);
  EXPECT_EQ(machine.Update(HealthyTrackingInput()).state,
            LocalizationState::kTracking);
  return machine;
}

TEST(LocalizationStateMachine, StartsInInit) {
  LocalizationStateMachine machine(LocalizationStateMachine::Config{});

  EXPECT_EQ(machine.state(), LocalizationState::kInit);
  const auto report = machine.Update(HealthInputs{});
  EXPECT_EQ(report.state, LocalizationState::kInit);
  EXPECT_EQ(report.level, HealthLevel::kWarn);
  EXPECT_EQ(report.reason, "INIT");
}

TEST(LocalizationStateMachine, InitTransitionsToRelocalizingWithoutInitialization) {
  LocalizationStateMachine machine(LocalizationStateMachine::Config{});
  auto input = HealthyTrackingInput();
  input.has_init = false;

  const auto report = machine.Update(input);

  EXPECT_EQ(report.state, LocalizationState::kRelocalizing);
  EXPECT_EQ(report.level, HealthLevel::kWarn);
}

TEST(LocalizationStateMachine,
     InitTransitionsToRelocalizingBeforeTrackingEvenWithInitialization) {
  LocalizationStateMachine machine(LocalizationStateMachine::Config{});

  EXPECT_EQ(machine.Update(HealthyTrackingInput()).state,
            LocalizationState::kRelocalizing);
  EXPECT_EQ(machine.Update(HealthyTrackingInput()).state,
            LocalizationState::kTracking);
}

TEST(LocalizationStateMachine, RelocalizingTransitionsToTrackingWithOkLevel) {
  LocalizationStateMachine machine(LocalizationStateMachine::Config{});
  auto input = HealthyTrackingInput();
  input.has_init = false;
  EXPECT_EQ(machine.Update(input).state, LocalizationState::kRelocalizing);

  input.has_init = true;
  const auto report = machine.Update(input);

  EXPECT_EQ(report.state, LocalizationState::kTracking);
  EXPECT_EQ(report.level, HealthLevel::kOk);
}

TEST(LocalizationStateMachine, TrackingRequiresTwoConsecutiveBadFitnessFramesToDegrade) {
  auto machine = MakeTrackingMachine();
  auto input = HealthyTrackingInput();
  input.reg_fitness = 0.6F;

  EXPECT_EQ(machine.Update(input).state, LocalizationState::kTracking);
  const auto report = machine.Update(input);
  EXPECT_EQ(report.state, LocalizationState::kDegraded);
  EXPECT_EQ(report.level, HealthLevel::kWarn);
  EXPECT_EQ(report.reason, "DEGRADED");
}

TEST(LocalizationStateMachine, HealthyTrackingFrameResetsDegradeHysteresis) {
  auto machine = MakeTrackingMachine();
  auto bad = HealthyTrackingInput();
  bad.reg_fitness = 0.6F;

  EXPECT_EQ(machine.Update(bad).state, LocalizationState::kTracking);
  EXPECT_EQ(machine.Update(HealthyTrackingInput()).state,
            LocalizationState::kTracking);
  EXPECT_EQ(machine.Update(bad).state, LocalizationState::kTracking);
  EXPECT_EQ(machine.Update(bad).state, LocalizationState::kDegraded);
}

TEST(LocalizationStateMachine, DegradedPersistentFitnessTransitionsToLostAfterThreshold) {
  auto machine = MakeTrackingMachine();
  auto degraded = HealthyTrackingInput();
  degraded.reg_fitness = 0.6F;
  EXPECT_EQ(machine.Update(degraded).state, LocalizationState::kTracking);
  EXPECT_EQ(machine.Update(degraded).state, LocalizationState::kDegraded);

  auto lost = HealthyTrackingInput();
  lost.reg_fitness = 1.1F;
  EXPECT_EQ(machine.Update(lost).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(lost).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(lost).state, LocalizationState::kLost);
}

TEST(LocalizationStateMachine, LidarStallIsHardFailureThroughDegradedToLost) {
  auto machine = MakeTrackingMachine();
  auto stalled = HealthyTrackingInput();
  stalled.stream.lidar_stalled = true;
  EXPECT_EQ(machine.Update(stalled).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(stalled).state, LocalizationState::kLost);
}

TEST(LocalizationStateMachine,
     ResultTimeoutIsHardFailureThroughDegradedToLost) {
  auto machine = MakeTrackingMachine();
  auto stale = HealthyTrackingInput();
  stale.last_result_age_us = FastConfig().result_timeout_us + 1U;

  EXPECT_EQ(machine.Update(stale).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(stale).state, LocalizationState::kLost);
}

TEST(LocalizationStateMachine, RepeatedOldBadResultDoesNotAdvanceHysteresis) {
  auto machine = MakeTrackingMachine();
  auto bad = HealthyTrackingInput();
  bad.reg_fitness = 0.6F;
  EXPECT_EQ(machine.Update(bad).state, LocalizationState::kTracking);

  bad.has_new_result = false;
  bad.has_new_stream_observation = false;
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(machine.Update(bad).state, LocalizationState::kTracking);
  }

  bad.has_new_result = true;
  EXPECT_EQ(machine.Update(bad).state, LocalizationState::kDegraded);
}

TEST(LocalizationStateMachine, RepeatedOldBadResultDoesNotAdvanceLostHysteresis) {
  auto machine = MakeTrackingMachine();
  auto degraded = HealthyTrackingInput();
  degraded.reg_fitness = 0.6F;
  machine.Update(degraded);
  ASSERT_EQ(machine.Update(degraded).state, LocalizationState::kDegraded);

  auto lost = HealthyTrackingInput();
  lost.reg_fitness = 1.1F;
  EXPECT_EQ(machine.Update(lost).state, LocalizationState::kDegraded);
  lost.has_new_result = false;
  lost.has_new_stream_observation = false;
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(machine.Update(lost).state, LocalizationState::kDegraded);
  }
  lost.has_new_result = true;
  EXPECT_EQ(machine.Update(lost).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(lost).state, LocalizationState::kLost);
}

TEST(LocalizationStateMachine, RepeatedOldStreamFaultDoesNotAdvanceHysteresis) {
  auto machine = MakeTrackingMachine();
  auto fault = HealthyTrackingInput();
  fault.stream.imu_stalled = true;
  EXPECT_EQ(machine.Update(fault).state, LocalizationState::kTracking);

  fault.has_new_result = false;
  fault.has_new_stream_observation = false;
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(machine.Update(fault).state, LocalizationState::kTracking);
  }

  fault.has_new_stream_observation = true;
  EXPECT_EQ(machine.Update(fault).state, LocalizationState::kDegraded);
}

TEST(LocalizationStateMachine, LostRecoveryWithoutInitializationReturnsToRelocalizing) {
  auto config = FastConfig();
  config.lost_enter_frames = 1;
  auto machine = MakeTrackingMachine(config);
  auto degraded = HealthyTrackingInput();
  degraded.reg_fitness = 0.6F;
  machine.Update(degraded);
  ASSERT_EQ(machine.Update(degraded).state, LocalizationState::kDegraded);
  auto lost = HealthyTrackingInput();
  lost.stream.lidar_stalled = true;
  ASSERT_EQ(machine.Update(lost).state, LocalizationState::kLost);

  auto recovered = HealthyTrackingInput();
  recovered.has_init = false;
  const auto report = machine.Update(recovered);
  EXPECT_EQ(report.state, LocalizationState::kRelocalizing);
  EXPECT_EQ(report.level, HealthLevel::kWarn);
}

TEST(LocalizationStateMachine, LostRecoveryWithInitializationReturnsToTracking) {
  auto config = FastConfig();
  config.lost_enter_frames = 1;
  auto machine = MakeTrackingMachine(config);
  auto degraded = HealthyTrackingInput();
  degraded.reg_fitness = 0.6F;
  machine.Update(degraded);
  ASSERT_EQ(machine.Update(degraded).state, LocalizationState::kDegraded);
  auto lost = HealthyTrackingInput();
  lost.stream.lidar_stalled = true;
  ASSERT_EQ(machine.Update(lost).state, LocalizationState::kLost);

  EXPECT_EQ(machine.Update(HealthyTrackingInput()).state,
            LocalizationState::kRelocalizing);
  const auto report = machine.Update(HealthyTrackingInput());
  EXPECT_EQ(report.state, LocalizationState::kTracking);
  EXPECT_EQ(report.level, HealthLevel::kOk);
}

TEST(LocalizationStateMachine, LostDoesNotRecoverOnBadFitnessOrImuStall) {
  auto machine = MakeTrackingMachine();
  auto hard_failure = HealthyTrackingInput();
  hard_failure.stream.lidar_stalled = true;
  machine.Update(hard_failure);
  ASSERT_EQ(machine.Update(hard_failure).state, LocalizationState::kLost);

  auto bad_fitness = HealthyTrackingInput();
  bad_fitness.reg_fitness = std::numeric_limits<float>::infinity();
  EXPECT_EQ(machine.Update(bad_fitness).state, LocalizationState::kLost);

  auto imu_stalled = HealthyTrackingInput();
  imu_stalled.stream.imu_stalled = true;
  EXPECT_EQ(machine.Update(imu_stalled).state, LocalizationState::kLost);
}

TEST(LocalizationStateMachine,
     LostRequiresNewTrackingResultToLeaveRelocalizing) {
  auto machine = MakeTrackingMachine();
  auto hard_failure = HealthyTrackingInput();
  hard_failure.stream.lidar_stalled = true;
  machine.Update(hard_failure);
  ASSERT_EQ(machine.Update(hard_failure).state, LocalizationState::kLost);

  auto recovered = HealthyTrackingInput();
  EXPECT_EQ(machine.Update(recovered).state, LocalizationState::kRelocalizing);
  recovered.has_new_result = false;
  EXPECT_EQ(machine.Update(recovered).state, LocalizationState::kRelocalizing);
  recovered.has_new_result = true;
  EXPECT_EQ(machine.Update(recovered).state, LocalizationState::kTracking);
}

TEST(LocalizationStateMachine, NonLostFrameResetsLostHysteresis) {
  auto machine = MakeTrackingMachine();
  auto degraded = HealthyTrackingInput();
  degraded.reg_fitness = 0.6F;
  machine.Update(degraded);
  ASSERT_EQ(machine.Update(degraded).state, LocalizationState::kDegraded);
  auto lost = HealthyTrackingInput();
  lost.reg_fitness = 1.1F;
  EXPECT_EQ(machine.Update(lost).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(lost).state, LocalizationState::kDegraded);

  EXPECT_EQ(machine.Update(degraded).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(lost).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(lost).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(lost).state, LocalizationState::kLost);
}

TEST(LocalizationStateMachine, ThresholdAndTimeoutEqualityDoNotEnterLost) {
  auto config = FastConfig();
  config.lost_enter_frames = 1;
  auto machine = MakeTrackingMachine(config);
  auto at_degraded_threshold = HealthyTrackingInput();
  at_degraded_threshold.reg_fitness =
      static_cast<float>(config.degraded_fitness_thres);
  EXPECT_EQ(machine.Update(at_degraded_threshold).state,
            LocalizationState::kTracking);

  auto degraded = HealthyTrackingInput();
  degraded.reg_fitness = 0.6F;
  machine.Update(degraded);
  ASSERT_EQ(machine.Update(degraded).state, LocalizationState::kDegraded);

  auto at_lost_threshold = HealthyTrackingInput();
  at_lost_threshold.reg_fitness = static_cast<float>(config.lost_fitness_thres);
  EXPECT_EQ(machine.Update(at_lost_threshold).state,
            LocalizationState::kDegraded);

  auto at_timeout = HealthyTrackingInput();
  at_timeout.last_result_age_us = config.result_timeout_us;
  EXPECT_EQ(machine.Update(at_timeout).state, LocalizationState::kDegraded);
}

TEST(LocalizationStateMachine, DegradedRecoveryRequiresThreeConsecutiveGoodFrames) {
  auto machine = MakeTrackingMachine();
  auto degraded = HealthyTrackingInput();
  degraded.reg_fitness = 0.6F;
  machine.Update(degraded);
  ASSERT_EQ(machine.Update(degraded).state, LocalizationState::kDegraded);

  EXPECT_EQ(machine.Update(HealthyTrackingInput()).state,
            LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(HealthyTrackingInput()).state,
            LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(HealthyTrackingInput()).state,
            LocalizationState::kTracking);
}

TEST(LocalizationStateMachine, BadDegradedFrameResetsRecoveryHysteresis) {
  auto machine = MakeTrackingMachine();
  auto degraded = HealthyTrackingInput();
  degraded.reg_fitness = 0.6F;
  machine.Update(degraded);
  ASSERT_EQ(machine.Update(degraded).state, LocalizationState::kDegraded);

  machine.Update(HealthyTrackingInput());
  machine.Update(HealthyTrackingInput());
  EXPECT_EQ(machine.Update(degraded).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(HealthyTrackingInput()).state,
            LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(HealthyTrackingInput()).state,
            LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(HealthyTrackingInput()).state,
            LocalizationState::kTracking);
}

TEST(LocalizationStateMachine, HealthScoreUsesSlowestKnownStreamRate) {
  auto machine = MakeTrackingMachine();
  auto input = HealthyTrackingInput();
  input.reg_fitness = 0.0F;
  input.stream.lidar_rate_hz = 2.0;

  const auto report = machine.Update(input);

  EXPECT_DOUBLE_EQ(report.s_stream, 0.2);
  EXPECT_DOUBLE_EQ(report.health_score, 0.2);
  EXPECT_LT(report.health_score, 0.5);
}

TEST(LocalizationStateMachine, InfiniteFitnessHasZeroScore) {
  auto machine = MakeTrackingMachine();
  auto input = HealthyTrackingInput();
  input.reg_fitness = std::numeric_limits<float>::infinity();

  const auto report = machine.Update(input);

  EXPECT_DOUBLE_EQ(report.s_fitness, 0.0);
  EXPECT_DOUBLE_EQ(report.health_score, 0.0);
}

TEST(LocalizationStateMachine,
     InfiniteFitnessTransitionsThroughDegradedToLostWithHysteresis) {
  auto machine = MakeTrackingMachine();
  auto input = HealthyTrackingInput();
  input.reg_fitness = std::numeric_limits<float>::infinity();

  EXPECT_EQ(machine.Update(input).state, LocalizationState::kTracking);
  EXPECT_EQ(machine.Update(input).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(input).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(input).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(input).state, LocalizationState::kLost);
}

TEST(LocalizationStateMachine, NegativeFitnessHasZeroScoreAndTransitionsToLost) {
  auto machine = MakeTrackingMachine();
  auto input = HealthyTrackingInput();
  input.reg_fitness = -0.1F;

  const auto first = machine.Update(input);
  EXPECT_DOUBLE_EQ(first.s_fitness, 0.0);
  EXPECT_EQ(first.state, LocalizationState::kTracking);
  EXPECT_EQ(machine.Update(input).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(input).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(input).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(input).state, LocalizationState::kLost);
}

TEST(LocalizationStateMachine, NanFitnessHasZeroScoreAndTransitionsToLost) {
  auto machine = MakeTrackingMachine();
  auto input = HealthyTrackingInput();
  input.reg_fitness = std::numeric_limits<float>::quiet_NaN();

  const auto first = machine.Update(input);
  EXPECT_DOUBLE_EQ(first.s_fitness, 0.0);
  EXPECT_EQ(first.state, LocalizationState::kTracking);
  EXPECT_EQ(machine.Update(input).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(input).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(input).state, LocalizationState::kDegraded);
  EXPECT_EQ(machine.Update(input).state, LocalizationState::kLost);
}

TEST(LocalizationStateMachine, ImuStallRequiresDegradeHysteresis) {
  auto machine = MakeTrackingMachine();
  auto input = HealthyTrackingInput();
  input.stream.imu_stalled = true;

  EXPECT_EQ(machine.Update(input).state, LocalizationState::kTracking);
  EXPECT_EQ(machine.Update(input).state, LocalizationState::kDegraded);
}

TEST(LocalizationStateMachine, TimestampJumpRequiresDegradeHysteresis) {
  auto machine = MakeTrackingMachine();
  auto input = HealthyTrackingInput();
  input.stream.timestamp_jump = true;

  EXPECT_EQ(machine.Update(input).state, LocalizationState::kTracking);
  EXPECT_EQ(machine.Update(input).state, LocalizationState::kDegraded);
}

TEST(LocalizationStateMachine, TimestampRollbackRequiresDegradeHysteresis) {
  auto machine = MakeTrackingMachine();
  auto input = HealthyTrackingInput();
  input.stream.timestamp_monotonic = false;

  EXPECT_EQ(machine.Update(input).state, LocalizationState::kTracking);
  EXPECT_EQ(machine.Update(input).state, LocalizationState::kDegraded);
}

TEST(LocalizationStateMachine, UnknownRatesDoNotPenalizeStreamScore) {
  auto machine = MakeTrackingMachine();
  auto input = HealthyTrackingInput();
  input.stream.rates_known = false;
  input.stream.lidar_rate_hz = 0.0;
  input.stream.imu_rate_hz = 0.0;

  const auto report = machine.Update(input);

  EXPECT_DOUBLE_EQ(report.s_stream, 1.0);
}

TEST(LocalizationStateMachine, RelocalizingScoreIncludesConsistency) {
  LocalizationStateMachine machine(FastConfig());
  auto input = HealthyTrackingInput();
  input.has_init = false;
  input.consistency_hit = 2;
  input.consistency_required = 10;

  const auto report = machine.Update(input);

  EXPECT_EQ(report.state, LocalizationState::kRelocalizing);
  EXPECT_DOUBLE_EQ(report.s_consistency, 0.2);
  EXPECT_DOUBLE_EQ(report.health_score, 0.2);
}

TEST(LocalizationStateMachine, TrackingConsistencyScoreIsAlwaysOne) {
  auto machine = MakeTrackingMachine();
  auto input = HealthyTrackingInput();
  input.consistency_hit = 2;
  input.consistency_required = 10;

  const auto report = machine.Update(input);

  EXPECT_EQ(report.state, LocalizationState::kTracking);
  EXPECT_DOUBLE_EQ(report.s_consistency, 1.0);
  EXPECT_NEAR(report.health_score, 0.9, 1e-6);
}

TEST(LocalizationStateMachine, LostUsesMaximumCovarianceAndErrorLevel) {
  auto config = FastConfig();
  config.lost_enter_frames = 1;
  config.max_covariance_scale = 42.0;
  auto machine = MakeTrackingMachine(config);
  auto degraded = HealthyTrackingInput();
  degraded.reg_fitness = 0.6F;
  machine.Update(degraded);
  ASSERT_EQ(machine.Update(degraded).state, LocalizationState::kDegraded);
  auto lost = HealthyTrackingInput();
  lost.reg_fitness = 1.1F;

  const auto report = machine.Update(lost);

  EXPECT_EQ(report.state, LocalizationState::kLost);
  EXPECT_EQ(report.level, HealthLevel::kError);
  EXPECT_DOUBLE_EQ(report.covariance_scale, 42.0);
}

TEST(LocalizationStateMachine, ResetReturnsToInitAndClearsHysteresis) {
  auto machine = MakeTrackingMachine();
  auto bad = HealthyTrackingInput();
  bad.reg_fitness = 0.6F;
  machine.Update(bad);

  machine.Reset();
  EXPECT_EQ(machine.state(), LocalizationState::kInit);
  EXPECT_EQ(machine.Update(HealthyTrackingInput()).state,
            LocalizationState::kRelocalizing);
  EXPECT_EQ(machine.Update(HealthyTrackingInput()).state,
            LocalizationState::kTracking);
  EXPECT_EQ(machine.Update(bad).state, LocalizationState::kTracking);
}

TEST(LocalizationStateMachine, RejectsInvalidConfiguration) {
  auto config = LocalizationStateMachine::Config{};
  config.degrade_enter_frames = 0;
  EXPECT_THROW(LocalizationStateMachine machine(config), std::invalid_argument);

  config = LocalizationStateMachine::Config{};
  config.degraded_fitness_thres = config.lost_fitness_thres;
  EXPECT_THROW(LocalizationStateMachine machine(config), std::invalid_argument);

  config = LocalizationStateMachine::Config{};
  config.nominal_lidar_rate_hz = std::numeric_limits<double>::infinity();
  EXPECT_THROW(LocalizationStateMachine machine(config), std::invalid_argument);

  config = LocalizationStateMachine::Config{};
  config.max_covariance_scale = 0.5;
  EXPECT_THROW(LocalizationStateMachine machine(config), std::invalid_argument);
}

}  // namespace
