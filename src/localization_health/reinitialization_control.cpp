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
  owns_automatic_initialization_ =
      automatic_relocalization_requested_ && !has_manual_initialization_;
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

void ReinitializationControl::WorkerAttempt::RecordInitializationResult(
    bool initialized) {
  if (!initialized && owns_automatic_initialization_) {
    control_->waiting_for_automatic_relocalization_request_.store(true);
  }
}

void ReinitializationControl::RequestAutomaticRelocalization() {
  automatic_relocalization_request_pending_.store(true);
}

ReinitializationControl::ManualInitializationTransaction
ReinitializationControl::BeginManualInitialization() {
  std::unique_lock<std::mutex> lock(mutex_);
  waiting_for_automatic_relocalization_request_.store(false);
  manual_initialization_pending_ = true;
  return ManualInitializationTransaction(std::move(lock));
}

ReinitializationControl::WorkerAttempt
ReinitializationControl::BeginWorkerAttempt() {
  return WorkerAttempt(this, std::unique_lock<std::mutex>(mutex_));
}

}  // namespace localization_health
