#include <gtest/gtest.h>

#include <cerrno>
#include <stdexcept>

#if __has_include("runtime/thread_runtime_controller.h")
#include "runtime/thread_runtime_controller.h"
#include <sched.h>
#define LIO_SAM_HAS_THREAD_RUNTIME_CONTROLLER 1
#else
#define LIO_SAM_HAS_THREAD_RUNTIME_CONTROLLER 0
#endif

#if LIO_SAM_HAS_THREAD_RUNTIME_CONTROLLER

TEST(ThreadRuntimeController, RejectsInvalidFifoPriorityAndCpuCore) {
  EXPECT_THROW(
    runtime::ThreadRuntimeController::Validate({true, 0, -1}),
    std::invalid_argument);
  EXPECT_THROW(
    runtime::ThreadRuntimeController::Validate({true, 100, -1}),
    std::invalid_argument);
  EXPECT_THROW(
    runtime::ThreadRuntimeController::Validate({false, 0, -2}),
    std::invalid_argument);
  EXPECT_THROW(
    runtime::ThreadRuntimeController::Validate({false, 0, CPU_SETSIZE}),
    std::invalid_argument);
  EXPECT_NO_THROW(
    runtime::ThreadRuntimeController::Validate({false, 0, -1}));
}

TEST(ThreadRuntimeController, RecordsIndependentSchedulingAndAffinityResults) {
  int requested_priority = 0;
  int requested_core = -1;
  runtime::ThreadRuntimeOperations operations{
    [&requested_priority](int priority) {
      requested_priority = priority;
      return EPERM;
    },
    [&requested_core](int core) {
      requested_core = core;
      return 0;
    },
  };
  runtime::ThreadRuntimeController controller({true, 70, 5}, operations);

  controller.ApplyToCurrentThread();

  const auto status = controller.Snapshot();
  EXPECT_EQ(requested_priority, 70);
  EXPECT_EQ(requested_core, 5);
  EXPECT_TRUE(status.scheduling_requested);
  EXPECT_FALSE(status.scheduling_applied);
  EXPECT_EQ(status.scheduling_errno, EPERM);
  EXPECT_TRUE(status.affinity_requested);
  EXPECT_TRUE(status.affinity_applied);
  EXPECT_EQ(status.affinity_errno, 0);

  runtime::ThreadRuntimeController affinity_rejected(
    {false, 0, 6},
    {
      [](int) {return EINVAL;},
      [](int) {return EINVAL;},
    });
  affinity_rejected.ApplyToCurrentThread();

  const auto rejected_status = affinity_rejected.Snapshot();
  EXPECT_FALSE(rejected_status.scheduling_requested);
  EXPECT_TRUE(rejected_status.affinity_requested);
  EXPECT_FALSE(rejected_status.affinity_applied);
  EXPECT_EQ(rejected_status.affinity_errno, EINVAL);
}

TEST(ThreadRuntimeController, DisabledRequestsDoNotInvokeOperations) {
  bool scheduling_called = false;
  bool affinity_called = false;
  runtime::ThreadRuntimeOperations operations{
    [&scheduling_called](int) {
      scheduling_called = true;
      return 0;
    },
    [&affinity_called](int) {
      affinity_called = true;
      return 0;
    },
  };
  runtime::ThreadRuntimeController controller({false, 0, -1}, operations);

  controller.ApplyToCurrentThread();

  const auto status = controller.Snapshot();
  EXPECT_FALSE(scheduling_called);
  EXPECT_FALSE(affinity_called);
  EXPECT_FALSE(status.scheduling_requested);
  EXPECT_FALSE(status.affinity_requested);
}

#else

TEST(ThreadRuntimeController, RejectsInvalidFifoPriorityAndCpuCore) {
  ADD_FAILURE() << "runtime/thread_runtime_controller.h is unavailable";
}

TEST(ThreadRuntimeController, RecordsIndependentSchedulingAndAffinityResults) {
  ADD_FAILURE() << "runtime/thread_runtime_controller.h is unavailable";
}

TEST(ThreadRuntimeController, DisabledRequestsDoNotInvokeOperations) {
  ADD_FAILURE() << "runtime/thread_runtime_controller.h is unavailable";
}

#endif
