#include "runtime/frame_processing_watchdog.h"

#include <chrono>
#include <stdexcept>

namespace runtime
{

FrameProcessingWatchdog::FrameProcessingWatchdog(std::uint64_t timeout_us)
: timeout_us_(timeout_us)
{
  if (timeout_us_ == 0U) {
    throw std::invalid_argument("frame processing timeout must be positive");
  }
}

std::uint64_t FrameProcessingWatchdog::SteadyNowMicroseconds()
{
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

void FrameProcessingWatchdog::Begin(std::uint64_t start_us)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (snapshot_.active) {
    throw std::invalid_argument("frame processing is already active");
  }

  start_us_ = start_us;
  snapshot_.active = true;
}

bool FrameProcessingWatchdog::Exceeded(std::uint64_t now_us) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.active && now_us > start_us_ &&
         now_us - start_us_ > timeout_us_;
}

void FrameProcessingWatchdog::Complete(
  std::uint64_t now_us,
  bool dropped_for_timeout)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!snapshot_.active) {
    return;
  }

  const std::uint64_t duration_us = now_us > start_us_ ? now_us - start_us_ : 0U;
  const bool timed_out = duration_us > timeout_us_;
  snapshot_.active = false;
  snapshot_.last_duration_us = duration_us;
  snapshot_.last_frame_timed_out = timed_out;
  if (timed_out) {
    ++snapshot_.timeout_count;
    if (dropped_for_timeout) {
      ++snapshot_.dropped_count;
    }
  }
}

FrameProcessingSnapshot FrameProcessingWatchdog::Snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

}  // namespace runtime
