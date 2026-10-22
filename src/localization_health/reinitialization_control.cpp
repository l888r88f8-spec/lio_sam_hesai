#include "localization_health/reinitialization_control.h"

#include <algorithm>
#include <utility>

namespace localization_health {

ReinitializationControl::ManualInitializationTransaction::
    ManualInitializationTransaction(std::unique_lock<std::mutex>&& lock)
    : lock_(std::move(lock)) {}

ReinitializationControl::WorkerAttempt::WorkerAttempt(
    ReinitializationControl* control, std::unique_lock<std::mutex>&& lock)
    : control_(control), lock_(std::move(lock)) {
  manual_cancellation_generation_at_start_ =
      control_->manual_cancellation_generation_.load();
  automatic_relocalization_requested_ =
      control_->automatic_relocalization_request_pending_.exchange(false);
  has_manual_initialization_ = control_->manual_initialization_pending_;
  if (has_manual_initialization_) {
    control_->automatic_relocalization_in_progress_ = false;
    control_->manual_initialization_pending_ = false;
    // Consume up to the generation represented by the published pending pose.
    // A newer manual request still waiting for mutex_ keeps
    // HasPendingManualInitialization() true.
    const std::uint64_t pending_generation =
        control_->manual_initialization_pending_generation_;
    if (pending_generation >
        control_->manual_initialization_consumed_generation_.load()) {
      control_->manual_initialization_consumed_generation_.store(
          pending_generation);
    }
  } else if (automatic_relocalization_requested_) {
    control_->automatic_relocalization_in_progress_ = true;
  }
  owns_automatic_initialization_ =
      control_->automatic_relocalization_in_progress_ &&
      !has_manual_initialization_;
  if (automatic_relocalization_requested_) {
    control_->waiting_for_automatic_relocalization_request_.store(false);
  }
}

bool ReinitializationControl::WorkerAttempt::
    automatic_relocalization_requested() const {
  return automatic_relocalization_requested_;
}

bool ReinitializationControl::WorkerAttempt::owns_automatic_initialization()
    const {
  return owns_automatic_initialization_;
}

bool ReinitializationControl::WorkerAttempt::has_manual_initialization() const {
  return has_manual_initialization_;
}

bool ReinitializationControl::WorkerAttempt::waiting_for_automatic_request()
    const {
  return control_->waiting_for_automatic_relocalization_request_.load();
}

std::uint64_t ReinitializationControl::WorkerAttempt::
    manual_cancellation_generation_at_start() const {
  return manual_cancellation_generation_at_start_;
}

bool ReinitializationControl::WorkerAttempt::manual_cancellation_observed()
    const {
  return control_->ManualCancellationGeneration() !=
         manual_cancellation_generation_at_start_;
}

bool ReinitializationControl::WorkerAttempt::CanCommitAutomatic() const {
  // A manual request published before this attempt took the mutex has not
  // incremented this attempt's start generation yet. It must still block an
  // automatic commit even though its pose is not published.
  return owns_automatic_initialization_ && !manual_cancellation_observed() &&
         !control_->HasPendingManualInitialization();
}

void ReinitializationControl::WorkerAttempt::
    RecordAutomaticConsistencyPending() {
  if (owns_automatic_initialization_) {
    control_->waiting_for_automatic_relocalization_request_.store(false);
  }
}

void ReinitializationControl::WorkerAttempt::RecordInitializationResult(
    bool initialized) {
  if (owns_automatic_initialization_) {
    control_->automatic_relocalization_in_progress_ = false;
    // Automatic retry is driven by the worker's steady-clock recovery policy:
    // a failed round keeps the attempt alive, and an expired attempt cools
    // down and restarts on a healthy frame. Parking the worker until another
    // external LOST_ENTRY/STREAM_RECOVERED edge was the recovery stall.
    if (initialized) {
      control_->waiting_for_automatic_relocalization_request_.store(false);
    }
  }
}

void ReinitializationControl::RequestAutomaticRelocalization() {
  automatic_relocalization_request_pending_.store(true);
}

void ReinitializationControl::SetManualContentionObserverForTest(
    std::function<void()> observer) {
  std::lock_guard<std::mutex> lock(mutex_manual_contention_observer_);
  manual_contention_observer_for_test_ = std::move(observer);
}

ReinitializationControl::ManualInitializationTransaction
ReinitializationControl::BeginManualInitialization() {
  // Publish cancellation and this request's generation before contending for
  // the worker mutex. A worker already inside a long automatic Init() can
  // therefore stop and refuse to commit even though it still owns mutex_.
  const std::uint64_t request_generation =
      manual_cancellation_generation_.fetch_add(1U) + 1U;
  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
  if (!lock.try_lock()) {
    NotifyManualContentionForTest();
    lock.lock();
  }
  waiting_for_automatic_relocalization_request_.store(false);
  automatic_relocalization_in_progress_ = false;
  manual_initialization_pending_ = true;
  // Keep this monotonic even if mutex_ acquisition order differs from publish
  // order; otherwise an older request could lower the marker and permanently
  // keep a newer consumed generation pending.
  manual_initialization_pending_generation_ =
      std::max(manual_initialization_pending_generation_, request_generation);
  return ManualInitializationTransaction(std::move(lock));
}

ReinitializationControl::WorkerAttempt
ReinitializationControl::BeginWorkerAttempt() {
  return WorkerAttempt(this, std::unique_lock<std::mutex>(mutex_));
}

void ReinitializationControl::NotifyManualContentionForTest() {
  std::function<void()> observer;
  {
    std::lock_guard<std::mutex> lock(mutex_manual_contention_observer_);
    observer = manual_contention_observer_for_test_;
  }
  if (observer) {
    observer();
  }
}

}  // namespace localization_health
