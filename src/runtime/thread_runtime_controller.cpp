#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "runtime/thread_runtime_controller.h"

#include <cerrno>
#include <glog/logging.h>
#include <pthread.h>
#include <sched.h>
#include <stdexcept>
#include <utility>

namespace
{

int SetFifoPriority(int priority)
{
  sched_param parameter{};
  parameter.sched_priority = priority;
  return pthread_setschedparam(pthread_self(), SCHED_FIFO, &parameter);
}

int SetCpuAffinity(int core)
{
  cpu_set_t mask;
  CPU_ZERO(&mask);
  CPU_SET(core, &mask);
  return pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask);
}

}  // namespace

namespace runtime
{

ThreadRuntimeOperations NativeOperations()
{
  return {SetFifoPriority, SetCpuAffinity};
}

ThreadRuntimeController::ThreadRuntimeController(
  ThreadRuntimeConfig config,
  ThreadRuntimeOperations operations)
: config_(config), operations_(std::move(operations))
{
}

void ThreadRuntimeController::Validate(const ThreadRuntimeConfig & config)
{
  if (config.enable_realtime_scheduling &&
    (config.fifo_priority < 1 || config.fifo_priority > 99))
  {
    throw std::invalid_argument("FIFO priority must be in [1, 99]");
  }
  if (config.cpu_core < -1 || config.cpu_core >= CPU_SETSIZE) {
    throw std::invalid_argument("CPU core is outside the supported affinity mask");
  }
}

void ThreadRuntimeController::ApplyToCurrentThread()
{
  Validate(config_);

  ThreadRuntimeStatus next_status;
  if (config_.enable_realtime_scheduling) {
    next_status.scheduling_requested = true;
    const int error = operations_.set_fifo_priority ?
      operations_.set_fifo_priority(config_.fifo_priority) : EINVAL;
    next_status.scheduling_applied = error == 0;
    next_status.scheduling_errno = error;
    if (error != 0) {
      LOG(WARNING) << "FIFO scheduling request for priority " << config_.fifo_priority
                   << " failed with pthread error code " << error;
    }
  }
  if (config_.cpu_core != -1) {
    next_status.affinity_requested = true;
    const int error = operations_.set_cpu_affinity ?
      operations_.set_cpu_affinity(config_.cpu_core) : EINVAL;
    next_status.affinity_applied = error == 0;
    next_status.affinity_errno = error;
    if (error != 0) {
      LOG(WARNING) << "CPU affinity request for core " << config_.cpu_core
                   << " failed with pthread error code " << error;
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  status_ = next_status;
}

ThreadRuntimeStatus ThreadRuntimeController::Snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

}  // namespace runtime
