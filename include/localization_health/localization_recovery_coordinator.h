#ifndef LIO_SAM_HESAI_LOCALIZATION_HEALTH_LOCALIZATION_RECOVERY_COORDINATOR_H
#define LIO_SAM_HESAI_LOCALIZATION_HEALTH_LOCALIZATION_RECOVERY_COORDINATOR_H

#include "localization_health/localization_state_machine.h"
#include "localization_health/recovery_attempt_gate.h"

#include <functional>

namespace localization_health {

class LocalizationRecoveryCoordinator {
 public:
  struct UpdateResult {
    HealthReport report;
    RecoveryAttemptReason reason = RecoveryAttemptReason::kNone;
    bool request_sent = false;
  };

  using AutomaticRelocalizationRequest = std::function<void()>;

  LocalizationRecoveryCoordinator(
      const LocalizationStateMachine::Config& state_config,
      const RecoveryAttemptGate::Config& recovery_gate_config);

  [[nodiscard]] UpdateResult Update(
      const HealthInputs& inputs, bool auto_relocalization_enabled,
      const AutomaticRelocalizationRequest& request_automatic_relocalization);

 private:
  LocalizationStateMachine state_machine_;
  RecoveryAttemptGate recovery_attempt_gate_;
};

}  // namespace localization_health

#endif  // LIO_SAM_HESAI_LOCALIZATION_HEALTH_LOCALIZATION_RECOVERY_COORDINATOR_H
