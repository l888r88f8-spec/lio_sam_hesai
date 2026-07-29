#include "localization_health/localization_recovery_coordinator.h"

namespace localization_health {

LocalizationRecoveryCoordinator::LocalizationRecoveryCoordinator(
    const LocalizationStateMachine::Config& state_config,
    const RecoveryAttemptGate::Config& recovery_gate_config)
    : state_machine_(state_config), recovery_attempt_gate_(recovery_gate_config) {}

LocalizationRecoveryCoordinator::UpdateResult
LocalizationRecoveryCoordinator::Update(
    const HealthInputs& inputs, bool auto_relocalization_enabled,
    const AutomaticRelocalizationRequest& request_automatic_relocalization) {
  const LocalizationState state_before = state_machine_.state();
  UpdateResult result;
  result.report = state_machine_.Update(inputs);
  result.reason = recovery_attempt_gate_.UpdateForStateTransition(
      state_before, result.report.state, inputs.stream);
  if (result.reason != RecoveryAttemptReason::kNone &&
      auto_relocalization_enabled && request_automatic_relocalization) {
    request_automatic_relocalization();
    result.request_sent = true;
  }
  return result;
}

}  // namespace localization_health
