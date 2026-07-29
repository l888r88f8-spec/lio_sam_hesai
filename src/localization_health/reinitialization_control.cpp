#include "localization_health/reinitialization_control.h"

#include <utility>

namespace localization_health {

ReinitializationControl::ManualInitializationTransaction::
    ManualInitializationTransaction(std::unique_lock<std::mutex>&& lock)
    : lock_(std::move(lock)) {}

ReinitializationControl::WorkerAttempt::WorkerAttempt(
    ReinitializationControl* control, std::unique_lock<std::mutex>&& lock)
    : control_(control), lock_(std::move(lock)) {
  automatic_relocalization_requested_ =
      control_->automatic_relocalization_request_pending_.exchange(false);
  has_manual_initialization_ = control_->manual_initialization_pending_;
  if (has_manual_initialization_) {
    control_->automatic_relocalization_in_progress_ = false;
  } else if (automatic_relocalization_requested_) {
    control_->automatic_relocalization_in_progress_ = true;
  }
  owns_automatic_initialization_ =
      control_->automatic_relocalization_in_progress_ &&
      !has_manual_initialization_;
  control_->manual_initialization_pending_ = false;
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
    if (!initialized) {
      control_->waiting_for_automatic_relocalization_request_.store(true);
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
  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
  if (!lock.try_lock()) {
    NotifyManualContentionForTest();
    lock.lock();
  }
  waiting_for_automatic_relocalization_request_.store(false);
  automatic_relocalization_in_progress_ = false;
  manual_initialization_pending_ = true;
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
