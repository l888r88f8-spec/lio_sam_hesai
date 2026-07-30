#pragma once

#include <cstdint>
#include <mutex>

namespace runtime
{

struct FrameProcessingSnapshot
{
  bool active{false};
  bool last_frame_timed_out{false};
  std::uint64_t last_duration_us{0U};
  std::uint64_t timeout_count{0U};
  std::uint64_t dropped_count{0U};
};

class FrameProcessingWatchdog
{
public:
  explicit FrameProcessingWatchdog(std::uint64_t timeout_us);

  [[nodiscard]] static std::uint64_t SteadyNowMicroseconds();
  void Begin(std::uint64_t start_us);
  [[nodiscard]] bool Exceeded(std::uint64_t now_us) const;
  void Complete(std::uint64_t now_us, bool dropped_for_timeout);
  [[nodiscard]] FrameProcessingSnapshot Snapshot() const;

private:
  std::uint64_t timeout_us_;
  std::uint64_t start_us_{0U};
  mutable std::mutex mutex_;
  FrameProcessingSnapshot snapshot_;
};

}  // namespace runtime
