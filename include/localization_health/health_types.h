#ifndef LIO_SAM_HESAI_LOCALIZATION_HEALTH_HEALTH_TYPES_H
#define LIO_SAM_HESAI_LOCALIZATION_HEALTH_HEALTH_TYPES_H

#include <cstdint>
#include <string>

namespace localization_health {

enum class LocalizationState {
  kInit,
  kRelocalizing,
  kTracking,
  kDegraded,
  kLost,
};

enum class HealthLevel {
  kOk,
  kWarn,
  kError,
};

struct StreamHealth {
  double lidar_rate_hz = 0.0;
  double imu_rate_hz = 0.0;
  bool lidar_stalled = false;
  bool imu_stalled = false;
  bool timestamp_monotonic = true;
  bool timestamp_jump = false;
  bool rates_known = false;
  // Snapshotted and consumed with pending anomalies under the watchdog mutex.
  bool has_new_observation = false;
};

struct HealthInputs {
  bool has_imu_init = false;
  bool map_ready = false;
  bool has_init = false;
  bool is_tracking = false;
  // Only new observations advance frame-based hysteresis counters.
  bool has_new_result = false;
  bool has_new_stream_observation = false;
  float reg_fitness = 0.0f;
  int consistency_hit = 0;
  int consistency_required = 0;
  StreamHealth stream{};
  std::uint64_t now_us = 0u;
  std::uint64_t last_result_age_us = 0u;
};

struct HealthReport {
  LocalizationState state = LocalizationState::kInit;
  HealthLevel level = HealthLevel::kWarn;
  double health_score = 0.0;
  double s_fitness = 0.0;
  double s_stream = 0.0;
  double s_consistency = 0.0;
  double covariance_scale = 1.0;
  std::string reason;
};

inline const char* ToString(LocalizationState state) {
  switch (state) {
    case LocalizationState::kInit:
      return "INIT";
    case LocalizationState::kRelocalizing:
      return "RELOCALIZING";
    case LocalizationState::kTracking:
      return "TRACKING";
    case LocalizationState::kDegraded:
      return "DEGRADED";
    case LocalizationState::kLost:
      return "LOST";
  }
  return "UNKNOWN";
}

}  // namespace localization_health

#endif  // LIO_SAM_HESAI_LOCALIZATION_HEALTH_HEALTH_TYPES_H
