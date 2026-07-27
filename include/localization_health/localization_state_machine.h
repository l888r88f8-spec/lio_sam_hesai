#ifndef LIO_SAM_HESAI_LOCALIZATION_HEALTH_LOCALIZATION_STATE_MACHINE_H
#define LIO_SAM_HESAI_LOCALIZATION_HEALTH_LOCALIZATION_STATE_MACHINE_H

#include "localization_health/health_types.h"

#include <cstdint>

namespace localization_health {

class LocalizationStateMachine {
 public:
  struct Config {
    int degrade_enter_frames = 3;
    int recover_frames = 5;
    int lost_enter_frames = 10;
    double degraded_fitness_thres = 0.5;
    double lost_fitness_thres = 1.0;
    double nominal_lidar_rate_hz = 10.0;
    double nominal_imu_rate_hz = 100.0;
    double min_stream_rate_ratio = 0.5;
    std::uint64_t result_timeout_us = 1000000U;
    double base_covariance = 0.05;
    double max_covariance_scale = 100.0;
  };

  explicit LocalizationStateMachine(const Config& config);

  HealthReport Update(const HealthInputs& input);
  [[nodiscard]] LocalizationState state() const;
  void Reset();

 private:
  [[nodiscard]] double ScoreFitness(float fitness) const;
  [[nodiscard]] double ScoreStream(const StreamHealth& stream) const;
  [[nodiscard]] double ScoreConsistency(int hit, int required) const;
  [[nodiscard]] bool IsFitnessBad(float fitness, double threshold) const;
  [[nodiscard]] bool IsStreamWarn(const HealthInputs& input) const;
  [[nodiscard]] bool IsHardFailure(const HealthInputs& input) const;
  [[nodiscard]] bool IsRecoveryHealthy(const HealthInputs& input) const;
  [[nodiscard]] bool IsDegradedFrame(const HealthInputs& input) const;
  [[nodiscard]] bool IsLostFrame(const HealthInputs& input) const;
  [[nodiscard]] HealthLevel LevelFor(LocalizationState state) const;
  void ClearCounters();

  Config config_;
  LocalizationState state_ = LocalizationState::kInit;
  int degrade_frames_ = 0;
  int stream_degrade_frames_ = 0;
  int recover_frames_ = 0;
  int lost_frames_ = 0;
};

}  // namespace localization_health

#endif  // LIO_SAM_HESAI_LOCALIZATION_HEALTH_LOCALIZATION_STATE_MACHINE_H
