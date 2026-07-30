#pragma once

#include <functional>
#include <mutex>

namespace runtime
{

struct ThreadRuntimeConfig
{
  bool enable_realtime_scheduling{true};
  int fifo_priority{0};
  int cpu_core{-1};
};

struct ThreadRuntimeStatus
{
  bool scheduling_requested{false};
  bool scheduling_applied{false};
  int scheduling_errno{0};
  bool affinity_requested{false};
  bool affinity_applied{false};
  int affinity_errno{0};
};

struct ThreadRuntimeOperations
{
  std::function<int(int)> set_fifo_priority;
  std::function<int(int)> set_cpu_affinity;
};

ThreadRuntimeOperations NativeOperations();

class ThreadRuntimeController
{
public:
  explicit ThreadRuntimeController(
    ThreadRuntimeConfig config,
    ThreadRuntimeOperations operations = NativeOperations());

  static void Validate(const ThreadRuntimeConfig & config);
  void ApplyToCurrentThread();
  [[nodiscard]] ThreadRuntimeStatus Snapshot() const;

private:
  ThreadRuntimeConfig config_;
  ThreadRuntimeOperations operations_;
  mutable std::mutex mutex_;
  ThreadRuntimeStatus status_;
};

}  // namespace runtime
