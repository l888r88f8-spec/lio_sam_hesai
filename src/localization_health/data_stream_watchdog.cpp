#include "localization_health/data_stream_watchdog.h"

#include <cmath>
#include <stdexcept>

namespace localization_health {

DataStreamWatchdog::DataStreamWatchdog(const Config& config) : config_(config) {
  if (config_.min_samples < 2U || config_.window_size < config_.min_samples ||
      config_.nominal_lidar_period_us == 0U ||
      config_.nominal_imu_period_us == 0U ||
      !std::isfinite(config_.jump_period_multiplier) ||
      config_.jump_period_multiplier <= 0.0) {
    throw std::invalid_argument("Invalid data stream watchdog configuration");
  }
}

void DataStreamWatchdog::RecordLidar(std::uint64_t stamp_us) {
  std::lock_guard<std::mutex> lock(mutex_);
  Record(lidar_, stamp_us, config_.nominal_lidar_period_us);
}

void DataStreamWatchdog::RecordImu(std::uint64_t stamp_us) {
  std::lock_guard<std::mutex> lock(mutex_);
  Record(imu_, stamp_us, config_.nominal_imu_period_us);
}

StreamHealth DataStreamWatchdog::Evaluate(std::uint64_t now_us) const {
  std::lock_guard<std::mutex> lock(mutex_);

  StreamHealth health;
  health.lidar_rate_hz = RateHz(lidar_);
  health.imu_rate_hz = RateHz(imu_);
  health.lidar_stalled =
      lidar_.has_last && now_us > lidar_.last_stamp &&
      now_us - lidar_.last_stamp > config_.lidar_stall_timeout_us;
  health.imu_stalled = imu_.has_last && now_us > imu_.last_stamp &&
                       now_us - imu_.last_stamp > config_.imu_stall_timeout_us;
  health.timestamp_monotonic = lidar_.monotonic && imu_.monotonic;
  health.timestamp_jump = lidar_.jump || imu_.jump;
  health.rates_known = lidar_.stamps.size() >= config_.min_samples &&
                       imu_.stamps.size() >= config_.min_samples;
  return health;
}

void DataStreamWatchdog::Record(Channel& channel, std::uint64_t stamp_us,
                                std::uint64_t nominal_period_us) {
  if (channel.has_last) {
    if (stamp_us < channel.last_stamp) {
      channel.monotonic = false;
      channel.jump = false;
      channel.stamps.clear();
      channel.last_stamp = stamp_us;
      channel.has_last = true;
      channel.stamps.push_back(stamp_us);
      return;
    }

    const std::uint64_t period_us = stamp_us - channel.last_stamp;
    channel.monotonic = true;
    channel.jump = static_cast<double>(period_us) >
                   config_.jump_period_multiplier *
                       static_cast<double>(nominal_period_us);
  }

  channel.last_stamp = stamp_us;
  channel.has_last = true;
  channel.stamps.push_back(stamp_us);
  while (channel.stamps.size() > config_.window_size) {
    channel.stamps.pop_front();
  }
}

double DataStreamWatchdog::RateHz(const Channel& channel) const {
  if (channel.stamps.size() < config_.min_samples) {
    return 0.0;
  }

  const std::uint64_t span_us = channel.stamps.back() - channel.stamps.front();
  if (span_us == 0U) {
    return 0.0;
  }

  return static_cast<double>(channel.stamps.size() - 1U) * 1000000.0 /
         static_cast<double>(span_us);
}

}  // namespace localization_health
