#include "localization_health/localization_recovery_coordinator.h"
#include "localization_health/reinitialization_control.h"

#include <gtest/gtest.h>

namespace {

using localization_health::HealthInputs;
using localization_health::LocalizationRecoveryCoordinator;
using localization_health::LocalizationState;
using localization_health::LocalizationStateMachine;
using localization_health::RecoveryAttemptGate;
using localization_health::RecoveryAttemptReason;
using localization_health::ReinitializationControl;
using localization_health::StreamHealth;

StreamHealth HealthyStream() {
  StreamHealth stream;
  stream.lidar_rate_hz = 10.0;
  stream.imu_rate_hz = 100.0;
  stream.rates_known = true;
  stream.has_new_observation = true;
  return stream;
}

HealthInputs TrackingInput(float fitness) {
  HealthInputs input;
  input.has_imu_init = true;
  input.map_ready = true;
  input.has_init = true;
  input.is_tracking = true;
  input.has_new_result = true;
  input.has_new_stream_observation = true;
  input.reg_fitness = fitness;
  input.stream = HealthyStream();
  return input;
}

TEST(LocalizationRecoveryCoordinator,
     RequestsOncePerRecoveryEdgeAndDrivesTheRealWorkerControl) {
  LocalizationStateMachine::Config state_config;
  state_config.degrade_enter_frames = 1;
  state_config.lost_enter_frames = 1;
  state_config.recover_frames = 1;
  LocalizationRecoveryCoordinator coordinator(
      state_config, RecoveryAttemptGate::Config{});
  ReinitializationControl control;
  int request_count = 0;
  const auto request_automatic_relocalization = [&] {
    ++request_count;
    control.RequestAutomaticRelocalization();
  };

  auto result = coordinator.Update(TrackingInput(0.1F), true,
                                   request_automatic_relocalization);
  ASSERT_EQ(result.report.state, LocalizationState::kRelocalizing);
  result = coordinator.Update(TrackingInput(0.1F), true,
                              request_automatic_relocalization);
  ASSERT_EQ(result.report.state, LocalizationState::kTracking);

  const auto poor_tracking = TrackingInput(1.1F);
  result = coordinator.Update(poor_tracking, true,
                              request_automatic_relocalization);
  ASSERT_EQ(result.report.state, LocalizationState::kDegraded);
  result = coordinator.Update(poor_tracking, true,
                              request_automatic_relocalization);
  EXPECT_EQ(result.report.state, LocalizationState::kLost);
  EXPECT_EQ(result.reason, RecoveryAttemptReason::kLostEntry);
  EXPECT_TRUE(result.request_sent);
  EXPECT_EQ(request_count, 1);

  {
    auto first_worker = control.BeginWorkerAttempt();
    EXPECT_TRUE(first_worker.automatic_relocalization_requested());
    EXPECT_TRUE(first_worker.owns_automatic_initialization());
    first_worker.RecordInitializationResult(false);
    EXPECT_TRUE(first_worker.waiting_for_automatic_request());
  }

  for (int i = 0; i < 3; ++i) {
    result = coordinator.Update(poor_tracking, true,
                                request_automatic_relocalization);
    EXPECT_EQ(result.report.state, LocalizationState::kLost);
    EXPECT_EQ(result.reason, RecoveryAttemptReason::kNone);
    EXPECT_FALSE(result.request_sent);
  }
  EXPECT_EQ(request_count, 1);
  {
    auto waiting_worker = control.BeginWorkerAttempt();
    EXPECT_FALSE(waiting_worker.automatic_relocalization_requested());
    EXPECT_TRUE(waiting_worker.waiting_for_automatic_request());
  }

  auto stalled = poor_tracking;
  stalled.stream.lidar_stalled = true;
  result = coordinator.Update(stalled, true, request_automatic_relocalization);
  EXPECT_EQ(result.report.state, LocalizationState::kLost);
  EXPECT_EQ(result.reason, RecoveryAttemptReason::kNone);
  EXPECT_FALSE(result.request_sent);

  result = coordinator.Update(poor_tracking, true,
                              request_automatic_relocalization);
  EXPECT_EQ(result.report.state, LocalizationState::kLost);
  EXPECT_EQ(result.reason, RecoveryAttemptReason::kStreamRecovered);
  EXPECT_TRUE(result.request_sent);
  EXPECT_EQ(request_count, 2);
  result = coordinator.Update(poor_tracking, true,
                              request_automatic_relocalization);
  EXPECT_EQ(result.reason, RecoveryAttemptReason::kNone);
  EXPECT_FALSE(result.request_sent);
  EXPECT_EQ(request_count, 2);

  {
    auto recovered_worker = control.BeginWorkerAttempt();
    EXPECT_TRUE(recovered_worker.automatic_relocalization_requested());
    EXPECT_TRUE(recovered_worker.owns_automatic_initialization());
    recovered_worker.RecordInitializationResult(false);
    EXPECT_TRUE(recovered_worker.waiting_for_automatic_request());
  }

  {
    auto manual = control.BeginManualInitialization();
  }
  auto manual_worker = control.BeginWorkerAttempt();
  EXPECT_TRUE(manual_worker.has_manual_initialization());
  EXPECT_FALSE(manual_worker.waiting_for_automatic_request());
}

}  // namespace
