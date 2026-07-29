#ifndef LIO_SAM_HESAI_LOCALIZATION_HEALTH_RECOVERY_ATTEMPT_GATE_H
#define LIO_SAM_HESAI_LOCALIZATION_HEALTH_RECOVERY_ATTEMPT_GATE_H

#include "localization_health/health_types.h"

namespace localization_health {

enum class RecoveryAttemptReason {
  kNone,
  kLostEntry,
  kStreamRecovered,
};

const char* ToString(RecoveryAttemptReason reason);

class RecoveryAttemptGate {
 public:
  struct Config {
    double nominal_lidar_rate_hz = 10.0;
    double nominal_imu_rate_hz = 100.0;
    double min_stream_rate_ratio = 0.5;
  };

  explicit RecoveryAttemptGate(const Config& config);

  RecoveryAttemptReason Update(LocalizationState state,
                               const StreamHealth& stream);
  void Reset();

 private:
  [[nodiscard]] bool IsStreamHealthy(const StreamHealth& stream) const;

  Config config_;
  LocalizationState previous_state_ = LocalizationState::kInit;
  bool saw_stream_fault_ = false;
};

}  // namespace localization_health

#endif  // LIO_SAM_HESAI_LOCALIZATION_HEALTH_RECOVERY_ATTEMPT_GATE_H
