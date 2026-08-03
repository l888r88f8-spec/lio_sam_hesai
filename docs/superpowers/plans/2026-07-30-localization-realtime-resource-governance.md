# Localization Realtime Resource Governance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give localization preprocessing and optimization best-effort Linux realtime/CPU placement, and discard frames cooperatively once their processing exceeds 500 ms.

**Architecture:** Keep Linux thread-control and elapsed-frame accounting in two independently testable `runtime` components. `System` validates and wires configuration, applies each thread policy from the owned thread before its worker loop, and exposes immutable snapshots through the existing diagnostics timer. Preprocessing discards an overdue cluster before enqueue; localization resets mutable registration and preintegration state before discarding an overdue result.

**Tech Stack:** C++17, ROS 2 Humble, pthread/Linux affinity APIs, gtest, CMake, glog.

---

### Task 1: Add the testable Linux thread-runtime controller

**Files:**
- Create: `include/runtime/thread_runtime_controller.h`
- Create: `src/runtime/thread_runtime_controller.cpp`
- Create: `test/test_thread_runtime_controller.cpp`
- Modify: `CMakeLists.txt`
- Test: `test/test_auto_relocalization_parameter_surface.cpp`

- [x] **Step 1: Write the failing controller surface and behavior tests**

Extend `test/test_auto_relocalization_parameter_surface.cpp` with a source-surface test requiring the new header, implementation source, CMake library source, and the symbols below. Add `test/test_thread_runtime_controller.cpp`; when the header is unavailable it must fail with `ADD_FAILURE()`, and once available it must exercise the API using injected operations that return `0`, `EPERM`, and `EINVAL`.

```cpp
namespace runtime {
struct ThreadRuntimeConfig {
  bool enable_realtime_scheduling{true};
  int fifo_priority{0};
  int cpu_core{-1};
};
struct ThreadRuntimeStatus {
  bool scheduling_requested{false};
  bool scheduling_applied{false};
  int scheduling_errno{0};
  bool affinity_requested{false};
  bool affinity_applied{false};
  int affinity_errno{0};
};
struct ThreadRuntimeOperations {
  std::function<int(int)> set_fifo_priority;
  std::function<int(int)> set_cpu_affinity;
};
class ThreadRuntimeController {
 public:
  explicit ThreadRuntimeController(ThreadRuntimeConfig config,
      ThreadRuntimeOperations operations = NativeOperations());
  static void Validate(const ThreadRuntimeConfig& config);
  void ApplyToCurrentThread();
  [[nodiscard]] ThreadRuntimeStatus Snapshot() const;
};
}
```

Require these test cases:

```cpp
TEST(ThreadRuntimeController, RejectsInvalidFifoPriorityAndCpuCore);
TEST(ThreadRuntimeController, RecordsIndependentSchedulingAndAffinityResults);
TEST(ThreadRuntimeController, DisabledRequestsDoNotInvokeOperations);
```

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target test_auto_relocalization_parameter_surface -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai -R '^test_auto_relocalization_parameter_surface$' --output-on-failure
```

Expected: FAIL because the `runtime` header/source and controller symbols do not exist.

- [x] **Step 2: Implement the controller with a native-operation boundary**

Define the types above in `include/runtime/thread_runtime_controller.h`. `Validate` throws `std::invalid_argument` when priority is outside `1..99` while scheduling is enabled, or when `cpu_core < -1` or `cpu_core >= CPU_SETSIZE`. It must not reject a numerically valid CPU merely because a runtime cpuset later refuses it.

In `src/runtime/thread_runtime_controller.cpp`, make `NativeOperations()` call the owned current thread only:

```cpp
int SetFifoPriority(int priority) {
  sched_param parameter{};
  parameter.sched_priority = priority;
  return pthread_setschedparam(pthread_self(), SCHED_FIFO, &parameter);
}

int SetCpuAffinity(int core) {
  cpu_set_t mask;
  CPU_ZERO(&mask);
  CPU_SET(core, &mask);
  return pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask);
}
```

`ApplyToCurrentThread()` calls `Validate`, then independently invokes the configured scheduling and affinity operations. It never throws for a nonzero operation result: it snapshots the corresponding errno, leaves that `*_applied` flag false, and still attempts the other operation. Protect the stored `ThreadRuntimeStatus` with a mutex so the diagnostics timer can read it while the worker owns application.

Add `src/runtime/thread_runtime_controller.cpp` to `${PROJECT_NAME}_relocalization_lib`. Add a gtest target which compiles that source directly, includes `include`, and links `Threads::Threads`; add `find_package(Threads REQUIRED)` if absent.

- [x] **Step 3: Verify the controller tests turn green**

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target test_thread_runtime_controller test_auto_relocalization_parameter_surface -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai -R '^(test_thread_runtime_controller|test_auto_relocalization_parameter_surface)$' --output-on-failure
```

Expected: both tests pass. In particular, injected `EPERM` must leave scheduling unapplied without preventing a successful injected affinity operation.

- [x] **Step 4: Commit the controller**

```bash
git add CMakeLists.txt include/runtime/thread_runtime_controller.h src/runtime/thread_runtime_controller.cpp test/test_thread_runtime_controller.cpp test/test_auto_relocalization_parameter_surface.cpp
git commit -m "feat(runtime): add thread scheduling controller"
```

### Task 2: Add deterministic frame-processing deadline accounting

**Files:**
- Create: `include/runtime/frame_processing_watchdog.h`
- Create: `src/runtime/frame_processing_watchdog.cpp`
- Create: `test/test_frame_processing_watchdog.cpp`
- Modify: `CMakeLists.txt`
- Test: `test/test_auto_relocalization_parameter_surface.cpp`

- [x] **Step 1: Write failing frame-watchdog tests**

Add a source-surface assertion for `runtime::FrameProcessingWatchdog`, then create a gtest with explicit microsecond timestamps. Define and test this API:

```cpp
namespace runtime {
struct FrameProcessingSnapshot {
  bool active{false};
  bool last_frame_timed_out{false};
  std::uint64_t last_duration_us{0};
  std::uint64_t timeout_count{0};
  std::uint64_t dropped_count{0};
};
class FrameProcessingWatchdog {
 public:
  explicit FrameProcessingWatchdog(std::uint64_t timeout_us);
  [[nodiscard]] static std::uint64_t SteadyNowMicroseconds();
  void Begin(std::uint64_t start_us);
  [[nodiscard]] bool Exceeded(std::uint64_t now_us) const;
  void Complete(std::uint64_t now_us, bool dropped_for_timeout);
  [[nodiscard]] FrameProcessingSnapshot Snapshot() const;
};
}
```

The test must prove that `500000` microseconds is not overdue, `500001` is overdue, an overdue completion marked for deadline discard increments both counters, and a subsequent normal completion preserves the counters while clearing `last_frame_timed_out`.

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target test_frame_processing_watchdog -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai -R '^test_frame_processing_watchdog$' --output-on-failure
```

Expected: FAIL because the watchdog API has not been implemented.

- [x] **Step 2: Implement mutex-protected accounting**

Implement the API with a mutex around active-frame state and snapshots. `Begin` records one active start timestamp. `Exceeded` is true only when `now_us - start_us > timeout_us`; it must not mutate counters. `Complete` clears active state, records duration, sets `last_frame_timed_out`, increments `timeout_count` only for an exceeded frame, and increments `dropped_count` only when that exceeded frame was marked `dropped_for_timeout`. Reject zero timeout and an attempt to begin a second active frame with `std::invalid_argument`.

Add the source to the relocalization library and add a direct gtest target using the source and `include` directory.

- [x] **Step 3: Verify exact-boundary and recovery behavior**

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target test_frame_processing_watchdog -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai -R '^test_frame_processing_watchdog$' --output-on-failure
```

Expected: 1/1 pass with no sleeps or wall-clock dependency.

- [x] **Step 4: Commit the watchdog**

```bash
git add CMakeLists.txt include/runtime/frame_processing_watchdog.h src/runtime/frame_processing_watchdog.cpp test/test_frame_processing_watchdog.cpp test/test_auto_relocalization_parameter_surface.cpp
git commit -m "feat(runtime): track frame processing deadlines"
```

### Task 3: Wire runtime parameters, worker entry control, and diagnostics

**Files:**
- Modify: `include/slam/config_parameters.h`
- Modify: `include/slam/system.h`
- Modify: `src/slam/system.cpp`
- Modify: `config/relocalization.yaml`
- Modify: `test/test_auto_relocalization_parameter_surface.cpp`

- [x] **Step 1: Extend the failing System contract**

Add a `RuntimeParameter` table to the existing surface test requiring these YAML leaves, ConfigParameters members, and quoted `System::InitConfigParameters()` names:

```cpp
{"enable_realtime_scheduling", "runtime_enable_realtime_scheduling_"},
{"preprocessing_fifo_priority", "runtime_preprocessing_fifo_priority_"},
{"localization_fifo_priority", "runtime_localization_fifo_priority_"},
{"preprocessing_cpu_core", "runtime_preprocessing_cpu_core_"},
{"localization_cpu_core", "runtime_localization_cpu_core_"},
{"frame_processing_timeout_ms", "runtime_frame_processing_timeout_ms_"},
```

Require the constructor to call `ThreadRuntimeController::Validate` before either worker is created, and require each worker lambda to call `ApplyToCurrentThread()` before `PreProcessing::Run` or `Localization::Run`. Require diagnostics keys `preprocessing_fifo_applied`, `preprocessing_affinity_applied`, `localization_fifo_applied`, `localization_affinity_applied`, their four errno keys, and both requested priority/core values.

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target test_auto_relocalization_parameter_surface -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai -R '^test_auto_relocalization_parameter_surface$' --output-on-failure
```

Expected: FAIL because no `runtime` parameter surface, thread wrappers, or diagnostics exist.

- [x] **Step 2: Add defaults and validate before worker construction**

Add the following YAML block after `localization_health`:

```yaml
    runtime:
      enable_realtime_scheduling: true
      preprocessing_fifo_priority: 70
      localization_fifo_priority: 80
      preprocessing_cpu_core: 4
      localization_cpu_core: 5
      frame_processing_timeout_ms: 500
```

Add matching ConfigParameters fields and parse them in `System::InitConfigParameters()`. Immediately after configuration is available, construct two `runtime::ThreadRuntimeConfig` values and call `ThreadRuntimeController::Validate` on both; convert `frame_processing_timeout_ms` to microseconds only after checking it is positive and multiplication cannot overflow `std::uint64_t`.

Store one `std::unique_ptr<runtime::ThreadRuntimeController>` per worker in `System`. Create each `std::thread` with a lambda:

```cpp
localization_thread_ptr_ = std::make_shared<std::thread>([this] {
  localization_thread_runtime_->ApplyToCurrentThread();
  localization_ptr_->Run();
});
```

Use the analogous lambda for preprocessing. In `PublishLocalizationHealth`, snapshot both controllers and append the named requested values, `*_applied` booleans, and errno integers via the existing `add_value` helper. A nonzero errno is diagnostic information, not a node error.

- [x] **Step 3: Verify runtime surface behavior and compile the library**

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target test_auto_relocalization_parameter_surface lio_sam_hesai_relocalization_lib -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai -R '^test_auto_relocalization_parameter_surface$' --output-on-failure
```

Expected: 1/1 pass and library build exit 0. The test must establish that policy application precedes either worker loop without requiring FIFO permission.

- [x] **Step 4: Commit System runtime wiring**

```bash
git add include/slam/config_parameters.h include/slam/system.h src/slam/system.cpp config/relocalization.yaml test/test_auto_relocalization_parameter_surface.cpp
git commit -m "feat(system): configure realtime worker resources"
```

### Task 4: Apply deadline drops in preprocessing and localization

**Files:**
- Modify: `include/slam/preprocessing.h`
- Modify: `src/slam/preprocessing.cpp`
- Modify: `include/slam/localization.h`
- Modify: `src/slam/localization.cpp`
- Modify: `include/slam/system.h`
- Modify: `src/slam/system.cpp`
- Modify: `test/test_auto_relocalization_parameter_surface.cpp`

- [x] **Step 1: Write the failing pipeline contract**

Extend the existing surface test to require a `runtime::FrameProcessingWatchdog` member and thread-safe public snapshot accessor in both worker classes. Require `PreProcessing::Run()` to begin a frame after dequeuing raw cloud data, check `Exceeded`, and avoid `cloud_cluster_deque_.emplace_back` for an overdue frame. Require `Localization::Run()` to begin after dequeuing a cluster, check the deadline after local-map preparation, `matcher_->Match`, and `Optimize`, and to invoke a dedicated `DiscardTimedOutFrame()` before the `CacheResultToSystem` commit boundary.

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target test_auto_relocalization_parameter_surface -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai -R '^test_auto_relocalization_parameter_surface$' --output-on-failure
```

Expected: FAIL because the worker pipelines do not yet own or consult deadline monitors.

- [x] **Step 2: Add preprocessing deadline discard before enqueue**

Construct `FrameProcessingWatchdog` with the validated System runtime timeout in `PreProcessing`. At the point a raw cloud has been dequeued, call `Begin(runtime::FrameProcessingWatchdog::SteadyNowMicroseconds())`. Before the existing `cloud_cluster_deque_.emplace_back`, use this exact decision shape:

```cpp
const auto now_us = runtime::FrameProcessingWatchdog::SteadyNowMicroseconds();
if (frame_processing_watchdog_.Exceeded(now_us)) {
  frame_processing_watchdog_.Complete(now_us, true);
  LOG(WARNING) << "Drop overdue preprocessing frame.";
  continue;
}
frame_processing_watchdog_.Complete(now_us, false);
```

Every early `continue` after `Begin` must call `Complete(runtime::FrameProcessingWatchdog::SteadyNowMicroseconds(), false)` first, so the monitor never remains active. Expose `FrameProcessingSnapshot` through a const accessor.

- [x] **Step 3: Add localization deadline recovery before result commit**

Construct the corresponding watchdog in `Localization`. Add a private `bool DiscardTimedOutFrameIfNeeded()` used after local-map preparation, after `matcher_->Match`, and after `Optimize`. When the deadline is exceeded it must:

```cpp
frame_processing_watchdog_.Complete(
    runtime::FrameProcessingWatchdog::SteadyNowMicroseconds(), true);
InitMatcher();
InitPreintegration();
need_update_local_map_ = true;
delta_pose_ = Mat4d::Identity();
return true;
```

Callers immediately `continue` on `true`; therefore no overdue frame can reach `CacheResultToSystem`, `UpdateCurrentLidarCloud`, `last_pose_`, `last_nav_state_`, or `last_odom_state_`. On the normal path, call `Complete(runtime::FrameProcessingWatchdog::SteadyNowMicroseconds(), false)` immediately after all stable state has been committed. Complete all initialization and insufficient-IMU early paths with `false` so no watch remains active.

In `System::PublishLocalizationHealth`, snapshot both workers and publish `*_last_frame_duration_ms`, `*_last_frame_timed_out`, `*_timeout_count`, and `*_dropped_count` using the established diagnostic helper.

- [x] **Step 4: Verify frame contract and focused unit tests**

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target test_auto_relocalization_parameter_surface test_frame_processing_watchdog lio_sam_hesai_relocalization_lib -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai -R '^(test_auto_relocalization_parameter_surface|test_frame_processing_watchdog|test_thread_runtime_controller)$' --output-on-failure
```

Expected: all three tests pass. Verify by source inspection that no deadline path passes the localization result commit boundary.

- [x] **Step 5: Commit deadline integration**

```bash
git add include/slam/preprocessing.h src/slam/preprocessing.cpp include/slam/localization.h src/slam/localization.cpp include/slam/system.h src/slam/system.cpp test/test_auto_relocalization_parameter_surface.cpp
git commit -m "feat(localization): drop overdue processing frames"
```

### Task 5: Run regression, replay, and record D2 verification

**Files:**
- Modify: `docs/superpowers/specs/2026-07-30-localization-realtime-resource-governance-design.md`
- Modify: this plan

- [x] **Step 1: Run the enabled package tests**

Run:

```bash
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai --output-on-failure -E '^(copyright|cpplint|flake8|lint_cmake|uncrustify|xmllint)$'
```

Expected: every enabled CTest passes.

- [x] **Step 2: Install and execute the short replay**

Run:

```bash
cmake --install /home/robot/slam_ws/build/lio_sam_hesai
LIO_SAM_KEEP_RESULTS=1 scripts/run_localization_stability.sh quick
```

Expected: install exits 0; the replay exits 0; its retained `observed.json` has `error: null`, `error_samples: 0`, and `valid_odom_ratio: 1.0`. The default FIFO request may appear as an `EPERM` downgrade in a non-privileged test environment, but must not terminate the node.

- [x] **Step 3: Mark the design and plan verified**

Only after Steps 1--2 pass, change the design status to `已实施并验证` and change every Task 1--5 checkbox in this file to `[x]`. The documentation directory is ignored, so force-add only these two files.

```bash
git add -f docs/superpowers/specs/2026-07-30-localization-realtime-resource-governance-design.md docs/superpowers/plans/2026-07-30-localization-realtime-resource-governance.md
git commit -m "docs(localization): record realtime governance verification"
```
