#include "localization_health/recovery_attempt_gate.h"

#include <cmath>
#include <stdexcept>

namespace localization_health {
namespace {

bool IsPositiveFinite(double value) {
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

const char* ToString(RecoveryAttemptReason reason) {
  switch (reason) {
    case RecoveryAttemptReason::kNone:
      return "NONE";
    case RecoveryAttemptReason::kLostEntry:
      return "LOST_ENTRY";
    case RecoveryAttemptReason::kStreamRecovered:
      return "STREAM_RECOVERED";
  }
  return "UNKNOWN";
}

RecoveryAttemptGate::RecoveryAttemptGate(const Config& config) : config_(config) {
  if (!IsPositiveFinite(config_.nominal_lidar_rate_hz) ||
      !IsPositiveFinite(config_.nominal_imu_rate_hz) ||
      !IsPositiveFinite(config_.min_stream_rate_ratio) ||
      config_.min_stream_rate_ratio > 1.0) {
    throw std::invalid_argument("Invalid recovery attempt gate configuration");
  }
}

RecoveryAttemptReason RecoveryAttemptGate::Update(LocalizationState state,
                                                  const StreamHealth& stream) {
  const bool entering_lost = state == LocalizationState::kLost &&
                             previous_state_ != LocalizationState::kLost;
  if (state != LocalizationState::kLost) {
    saw_stream_fault_ = false;
    previous_state_ = state;
    return RecoveryAttemptReason::kNone;
  }

  const bool stream_healthy = IsStreamHealthy(stream);
  if (entering_lost) {
    saw_stream_fault_ = !stream_healthy;
    previous_state_ = state;
    return RecoveryAttemptReason::kLostEntry;
  }

  previous_state_ = state;
  if (!stream_healthy) {
    saw_stream_fault_ = true;
    return RecoveryAttemptReason::kNone;
  }

  if (saw_stream_fault_ && stream.has_new_observation) {
    saw_stream_fault_ = false;
    return RecoveryAttemptReason::kStreamRecovered;
  }

  return RecoveryAttemptReason::kNone;
}

RecoveryAttemptReason RecoveryAttemptGate::UpdateForStateTransition(
    LocalizationState state_before, LocalizationState state_after,
    const StreamHealth& stream) {
  if (state_before != LocalizationState::kLost) {
    return Update(state_after, stream);
  }

  const auto reason = Update(state_before, stream);
  if (state_after != LocalizationState::kLost) {
    (void)Update(state_after, stream);
  }
  return reason;
}

void RecoveryAttemptGate::Reset() {
  previous_state_ = LocalizationState::kInit;
  saw_stream_fault_ = false;
}

bool RecoveryAttemptGate::IsStreamHealthy(const StreamHealth& stream) const {
  if (stream.lidar_stalled || stream.imu_stalled ||
      !stream.timestamp_monotonic || stream.timestamp_jump) {
    return false;
  }
  if (!stream.rates_known) {
    return true;
  }

  const double minimum_lidar_rate_hz =
      config_.nominal_lidar_rate_hz * config_.min_stream_rate_ratio;
  const double minimum_imu_rate_hz =
      config_.nominal_imu_rate_hz * config_.min_stream_rate_ratio;
  return std::isfinite(stream.lidar_rate_hz) &&
         std::isfinite(stream.imu_rate_hz) &&
         stream.lidar_rate_hz >= minimum_lidar_rate_hz &&
         stream.imu_rate_hz >= minimum_imu_rate_hz;
}

}  // namespace localization_health
