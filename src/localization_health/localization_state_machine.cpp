#include "localization_health/localization_state_machine.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace localization_health {
namespace {

double Clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

bool IsPositiveFinite(double value) {
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

LocalizationStateMachine::LocalizationStateMachine(const Config& config)
    : config_(config) {
  if (config_.degrade_enter_frames <= 0 || config_.recover_frames <= 0 ||
      config_.lost_enter_frames <= 0 ||
      !IsPositiveFinite(config_.degraded_fitness_thres) ||
      !IsPositiveFinite(config_.lost_fitness_thres) ||
      config_.degraded_fitness_thres >= config_.lost_fitness_thres ||
      !IsPositiveFinite(config_.nominal_lidar_rate_hz) ||
      !IsPositiveFinite(config_.nominal_imu_rate_hz) ||
      !IsPositiveFinite(config_.min_stream_rate_ratio) ||
      config_.min_stream_rate_ratio > 1.0 ||
      config_.result_timeout_us == 0U ||
      !IsPositiveFinite(config_.base_covariance) ||
      !IsPositiveFinite(config_.max_covariance_scale) ||
      config_.max_covariance_scale < 1.0) {
    throw std::invalid_argument("Invalid localization state machine configuration");
  }
}

HealthReport LocalizationStateMachine::Update(const HealthInputs& input) {
  switch (state_) {
    case LocalizationState::kInit:
      if (input.has_imu_init && input.map_ready) {
        state_ = LocalizationState::kRelocalizing;
        ClearCounters();
      }
      break;
    case LocalizationState::kRelocalizing:
      if (input.has_new_result && input.has_init && input.is_tracking) {
        state_ = LocalizationState::kTracking;
        ClearCounters();
      }
      break;
    case LocalizationState::kTracking:
      if (IsHardFailure(input)) {
        state_ = LocalizationState::kDegraded;
        ClearCounters();
      } else {
        bool enter_degraded = false;
        if (input.has_new_result) {
          if (IsFitnessBad(input.reg_fitness,
                           config_.degraded_fitness_thres)) {
            ++degrade_frames_;
            enter_degraded =
                degrade_frames_ >= config_.degrade_enter_frames;
          } else {
            degrade_frames_ = 0;
          }
        }
        if (input.has_new_stream_observation) {
          if (IsStreamWarn(input)) {
            ++stream_degrade_frames_;
            enter_degraded =
                enter_degraded ||
                stream_degrade_frames_ >= config_.degrade_enter_frames;
          } else {
            stream_degrade_frames_ = 0;
          }
        }
        if (enter_degraded) {
          state_ = LocalizationState::kDegraded;
          ClearCounters();
        }
      }
      break;
    case LocalizationState::kDegraded:
      if (IsHardFailure(input)) {
        state_ = LocalizationState::kLost;
        ClearCounters();
      } else {
        if (input.has_new_result) {
          if (IsLostFrame(input)) {
            ++lost_frames_;
            recover_frames_ = 0;
            if (lost_frames_ >= config_.lost_enter_frames) {
              state_ = LocalizationState::kLost;
              ClearCounters();
            }
          } else {
            lost_frames_ = 0;
            if (IsRecoveryHealthy(input)) {
              ++recover_frames_;
              if (recover_frames_ >= config_.recover_frames) {
                state_ = LocalizationState::kTracking;
                ClearCounters();
              }
            } else {
              recover_frames_ = 0;
            }
          }
        }
        if (state_ == LocalizationState::kDegraded &&
            input.has_new_stream_observation && IsStreamWarn(input)) {
          recover_frames_ = 0;
        }
      }
      break;
    case LocalizationState::kLost:
      if (IsRecoveryHealthy(input)) {
        state_ = LocalizationState::kRelocalizing;
        ClearCounters();
      }
      break;
  }

  HealthReport report;
  report.state = state_;
  report.level = LevelFor(state_);
  report.s_fitness = ScoreFitness(input.reg_fitness);
  report.s_stream = ScoreStream(input.stream);
  report.s_consistency = state_ == LocalizationState::kRelocalizing
                             ? ScoreConsistency(input.consistency_hit,
                                                input.consistency_required)
                             : 1.0;
  report.health_score = std::min(report.s_fitness, report.s_stream);
  if (state_ == LocalizationState::kRelocalizing) {
    report.health_score = std::min(report.health_score, report.s_consistency);
  }
  // A TF pose that stopped tracking the vehicle is a hard safety fault: no
  // reported health may remain and consumers must inflate uncertainty now.
  if (input.tf_pose_stale) {
    report.health_score = 0.0;
  }
  if (state_ == LocalizationState::kLost || input.tf_pose_stale) {
    report.covariance_scale = config_.max_covariance_scale;
  } else if (state_ == LocalizationState::kTracking) {
    report.covariance_scale = 1.0;
  } else {
    report.covariance_scale =
        Clamp(1.0 / std::max(report.health_score, 1e-3), 1.0,
              config_.max_covariance_scale);
  }
  report.reason = ToString(state_);
  return report;
}

LocalizationState LocalizationStateMachine::state() const { return state_; }

void LocalizationStateMachine::Reset() {
  state_ = LocalizationState::kInit;
  ClearCounters();
}

double LocalizationStateMachine::ScoreFitness(float fitness) const {
  if (!std::isfinite(fitness) || fitness < 0.0F) {
    return 0.0;
  }
  return Clamp(1.0 - static_cast<double>(fitness) / config_.lost_fitness_thres,
               0.0, 1.0);
}

double LocalizationStateMachine::ScoreStream(const StreamHealth& stream) const {
  if (stream.lidar_stalled || stream.imu_stalled) {
    return 0.0;
  }
  if (!stream.rates_known) {
    return 1.0;
  }
  const double lidar_score =
      std::isfinite(stream.lidar_rate_hz)
          ? Clamp(stream.lidar_rate_hz / config_.nominal_lidar_rate_hz, 0.0,
                  1.0)
          : 0.0;
  const double imu_score =
      std::isfinite(stream.imu_rate_hz)
          ? Clamp(stream.imu_rate_hz / config_.nominal_imu_rate_hz, 0.0, 1.0)
          : 0.0;
  return std::min(lidar_score, imu_score);
}

double LocalizationStateMachine::ScoreConsistency(int hit, int required) const {
  if (required <= 0) {
    return 1.0;
  }
  return Clamp(static_cast<double>(hit) / static_cast<double>(required), 0.0,
               1.0);
}

bool LocalizationStateMachine::IsFitnessBad(float fitness,
                                            double threshold) const {
  return !std::isfinite(fitness) || fitness < 0.0F ||
         static_cast<double>(fitness) > threshold;
}

bool LocalizationStateMachine::IsStreamWarn(
    const HealthInputs& input) const {
  return input.stream.imu_stalled || input.stream.timestamp_jump ||
         !input.stream.timestamp_monotonic ||
         (input.stream.rates_known &&
          (input.stream.lidar_rate_hz / config_.nominal_lidar_rate_hz <
               config_.min_stream_rate_ratio ||
           input.stream.imu_rate_hz / config_.nominal_imu_rate_hz <
               config_.min_stream_rate_ratio));
}

bool LocalizationStateMachine::IsHardFailure(
    const HealthInputs& input) const {
  return input.stream.lidar_stalled || input.tf_pose_stale ||
         input.last_result_age_us > config_.result_timeout_us;
}

bool LocalizationStateMachine::IsRecoveryHealthy(
    const HealthInputs& input) const {
  return !IsHardFailure(input) && !IsStreamWarn(input) &&
         !IsFitnessBad(input.reg_fitness, config_.degraded_fitness_thres);
}

bool LocalizationStateMachine::IsDegradedFrame(const HealthInputs& input) const {
  return IsFitnessBad(input.reg_fitness, config_.degraded_fitness_thres) ||
         IsStreamWarn(input) || IsHardFailure(input);
}

bool LocalizationStateMachine::IsLostFrame(const HealthInputs& input) const {
  return IsFitnessBad(input.reg_fitness, config_.lost_fitness_thres);
}

HealthLevel LocalizationStateMachine::LevelFor(LocalizationState state) const {
  switch (state) {
    case LocalizationState::kTracking:
      return HealthLevel::kOk;
    case LocalizationState::kLost:
      return HealthLevel::kError;
    case LocalizationState::kInit:
    case LocalizationState::kRelocalizing:
    case LocalizationState::kDegraded:
      return HealthLevel::kWarn;
  }
  return HealthLevel::kWarn;
}

void LocalizationStateMachine::ClearCounters() {
  degrade_frames_ = 0;
  stream_degrade_frames_ = 0;
  recover_frames_ = 0;
  lost_frames_ = 0;
}

}  // namespace localization_health
