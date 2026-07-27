#ifndef LIO_SAM_HESAI_LOCALIZATION_HEALTH_DATA_STREAM_WATCHDOG_H
#define LIO_SAM_HESAI_LOCALIZATION_HEALTH_DATA_STREAM_WATCHDOG_H

#include "localization_health/health_types.h"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>

namespace localization_health {

class DataStreamWatchdog {
 public:
  struct Config {
    std::size_t window_size = 50U;
    std::size_t min_samples = 5U;
    std::uint64_t lidar_stall_timeout_us = 500000U;
    std::uint64_t imu_stall_timeout_us = 100000U;
    std::uint64_t nominal_lidar_period_us = 100000U;
    std::uint64_t nominal_imu_period_us = 10000U;
    double jump_period_multiplier = 5.0;
  };

  explicit DataStreamWatchdog(const Config& config);

  void RecordLidar(std::uint64_t stamp_us);
  void RecordImu(std::uint64_t stamp_us);
  [[nodiscard]] StreamHealth Evaluate(std::uint64_t now_us) const;

 private:
  struct Channel {
    std::deque<std::uint64_t> stamps;
    std::uint64_t last_stamp = 0U;
    bool has_last = false;
    bool monotonic = true;
    bool jump = false;
  };

  void Record(Channel& channel, std::uint64_t stamp_us,
              std::uint64_t nominal_period_us);
  [[nodiscard]] double RateHz(const Channel& channel) const;

  const Config config_;
  mutable std::mutex mutex_;
  Channel lidar_;
  Channel imu_;
};

}  // namespace localization_health

#endif  // LIO_SAM_HESAI_LOCALIZATION_HEALTH_DATA_STREAM_WATCHDOG_H
