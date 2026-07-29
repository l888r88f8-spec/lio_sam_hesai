#include "localization_health/reinitialization_control.h"

#include <gtest/gtest.h>

#include <atomic>
#include <future>
#include <thread>

namespace {

using localization_health::ReinitializationControl;

TEST(ReinitializationControl, FailedAutomaticAttemptWaitsForAnotherRequest) {
  ReinitializationControl control;
  control.RequestAutomaticRelocalization();

  {
    auto attempt = control.BeginWorkerAttempt();
    EXPECT_TRUE(attempt.automatic_relocalization_requested());
    EXPECT_TRUE(attempt.owns_automatic_initialization());
    EXPECT_FALSE(attempt.has_manual_initialization());
    EXPECT_FALSE(attempt.waiting_for_automatic_request());
    attempt.RecordInitializationResult(false);
    EXPECT_TRUE(attempt.waiting_for_automatic_request());
  }

  auto waiting_attempt = control.BeginWorkerAttempt();
  EXPECT_FALSE(waiting_attempt.automatic_relocalization_requested());
  EXPECT_FALSE(waiting_attempt.owns_automatic_initialization());
  EXPECT_TRUE(waiting_attempt.waiting_for_automatic_request());
}

TEST(ReinitializationControl,
     ManualRequestBeforeWorkerMakesAttemptNonAutomatic) {
  ReinitializationControl control;
  {
    auto manual_request = control.BeginManualInitialization();
  }
  control.RequestAutomaticRelocalization();

  {
    auto attempt = control.BeginWorkerAttempt();
    EXPECT_TRUE(attempt.automatic_relocalization_requested());
    EXPECT_FALSE(attempt.owns_automatic_initialization());
    EXPECT_TRUE(attempt.has_manual_initialization());
    attempt.RecordInitializationResult(false);
    EXPECT_FALSE(attempt.waiting_for_automatic_request());
  }

  auto next_attempt = control.BeginWorkerAttempt();
  EXPECT_FALSE(next_attempt.waiting_for_automatic_request());
}

TEST(ReinitializationControl,
     ManualRequestAfterAutomaticAttemptClearsWaitForNextManualAttempt) {
  ReinitializationControl control;
  control.RequestAutomaticRelocalization();

  std::promise<void> manual_contention_observed;
  std::future<void> manual_contention_observed_future =
      manual_contention_observed.get_future();
  control.SetManualContentionObserverForTest([&] {
    manual_contention_observed.set_value();
  });

  std::atomic_bool automatic_attempt_active{false};
  std::promise<void> automatic_attempt_started;
  std::future<void> automatic_attempt_started_future =
      automatic_attempt_started.get_future();
  std::promise<void> finish_automatic_attempt;
  std::future<void> finish_automatic_attempt_future =
      finish_automatic_attempt.get_future();
  std::promise<bool> automatic_waiting_after_failure;
  std::future<bool> automatic_waiting_after_failure_future =
      automatic_waiting_after_failure.get_future();
  std::promise<void> automatic_attempt_finished;
  std::future<void> automatic_attempt_finished_future =
      automatic_attempt_finished.get_future();

  std::thread automatic_worker([&] {
    {
      auto attempt = control.BeginWorkerAttempt();
      automatic_attempt_active.store(true);
      automatic_attempt_started.set_value();
      finish_automatic_attempt_future.wait();
      attempt.RecordInitializationResult(false);
      automatic_waiting_after_failure.set_value(
          attempt.waiting_for_automatic_request());
    }
    automatic_attempt_active.store(false);
    automatic_attempt_finished.set_value();
  });

  automatic_attempt_started_future.wait();

  std::promise<void> manual_request_acquired;
  std::future<void> manual_request_acquired_future =
      manual_request_acquired.get_future();
  std::promise<void> release_manual_request;
  std::future<void> release_manual_request_future =
      release_manual_request.get_future();

  std::thread manual_worker([&] {
    auto manual_request = control.BeginManualInitialization();
    manual_request_acquired.set_value();
    release_manual_request_future.wait();
  });

  manual_contention_observed_future.wait();
  EXPECT_TRUE(automatic_attempt_active.load());
  finish_automatic_attempt.set_value();
  automatic_attempt_finished_future.wait();
  EXPECT_TRUE(automatic_waiting_after_failure_future.get());
  manual_request_acquired_future.wait();
  release_manual_request.set_value();

  automatic_worker.join();
  manual_worker.join();

  auto next_attempt = control.BeginWorkerAttempt();
  EXPECT_FALSE(next_attempt.automatic_relocalization_requested());
  EXPECT_FALSE(next_attempt.owns_automatic_initialization());
  EXPECT_TRUE(next_attempt.has_manual_initialization());
  EXPECT_FALSE(next_attempt.waiting_for_automatic_request());
}

TEST(ReinitializationControl,
     FailedColdStartAndManualAttemptsDoNotWaitAndAutomaticRequestsCoalesce) {
  ReinitializationControl control;

  {
    auto cold_start = control.BeginWorkerAttempt();
    EXPECT_FALSE(cold_start.automatic_relocalization_requested());
    EXPECT_FALSE(cold_start.has_manual_initialization());
    cold_start.RecordInitializationResult(false);
    EXPECT_FALSE(cold_start.waiting_for_automatic_request());
  }

  {
    auto manual_request = control.BeginManualInitialization();
  }
  {
    auto manual_attempt = control.BeginWorkerAttempt();
    EXPECT_TRUE(manual_attempt.has_manual_initialization());
    manual_attempt.RecordInitializationResult(false);
    EXPECT_FALSE(manual_attempt.waiting_for_automatic_request());
  }

  control.RequestAutomaticRelocalization();
  control.RequestAutomaticRelocalization();
  {
    auto automatic_attempt = control.BeginWorkerAttempt();
    EXPECT_TRUE(automatic_attempt.automatic_relocalization_requested());
    EXPECT_TRUE(automatic_attempt.owns_automatic_initialization());
    automatic_attempt.RecordInitializationResult(true);
  }

  auto next_attempt = control.BeginWorkerAttempt();
  EXPECT_FALSE(next_attempt.automatic_relocalization_requested());
}

}  // namespace
