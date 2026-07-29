# Localization Fault Self-Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** When localization enters `LOST`, request one automatic Scan Context reinitialization immediately, then retry once per recovered lidar/IMU data-stream edge until localization leaves `LOST`.

**Architecture:** Add a pure `RecoveryAttemptGate` that converts the health state and watchdog snapshot into a one-shot reason (`lost_entry` or `stream_recovered`). `System` owns this gate and posts work to `Localization`; the localization worker consumes an atomic request on its next point-cloud cluster, uses the existing `Init()` path once, and waits for a new request after failure.

**Tech Stack:** C++17, ROS 2 Humble/rclcpp, existing `localization_health` value types, gtest, CMake/ament.

---

## File structure

- Create `include/localization_health/recovery_attempt_gate.h`: pure stateful policy API and action reason enum.
- Create `src/localization_health/recovery_attempt_gate.cpp`: stream-health predicate and LOST/recovery-edge state transition implementation.
- Create `test/test_recovery_attempt_gate.cpp`: isolated gtests for the retry contract.
- Create `include/localization_health/localization_recovery_coordinator.h` and `src/localization_health/localization_recovery_coordinator.cpp`: production health-state/gate orchestration seam.
- Create `test/test_localization_recovery_coordinator.cpp`: composition test for LOST entry, stream recovery, worker consumption, and manual takeover.
- Modify `CMakeLists.txt`: compile the new production source and its gtest target.
- Modify `include/slam/localization.h` and `src/slam/localization.cpp`: public atomic request entry point and worker-owned one-shot reinitialization handling.
- Modify `include/slam/system.h` and `src/slam/system.cpp`: own and invoke the gate; expose the latest trigger reason through `/diagnostics`.
- Modify `test/test_auto_relocalization_parameter_surface.cpp`: retain a low-cost source-level contract for the cross-thread wiring.
- Create or update only these documentation files: `docs/superpowers/specs/2026-07-29-localization-fault-self-recovery-design.md` and this plan.

### Task 1: Implement and test the pure retry gate

**Files:**
- Create: `include/localization_health/recovery_attempt_gate.h`
- Create: `src/localization_health/recovery_attempt_gate.cpp`
- Create: `test/test_recovery_attempt_gate.cpp`
- Modify: `CMakeLists.txt:145-149,271-277`

- [x] **Step 1: Write the failing retry-gate tests**

Create `test/test_recovery_attempt_gate.cpp` with this complete test fixture and contract tests. Keep it independent from ROS, `System`, and `Localization`.

```cpp
#include "localization_health/recovery_attempt_gate.h"

#include <gtest/gtest.h>

namespace {

using localization_health::LocalizationState;
using localization_health::RecoveryAttemptGate;
using localization_health::RecoveryAttemptReason;
using localization_health::StreamHealth;

RecoveryAttemptGate MakeGate() {
  RecoveryAttemptGate::Config config;
  config.nominal_lidar_rate_hz = 10.0;
  config.nominal_imu_rate_hz = 100.0;
  config.min_stream_rate_ratio = 0.5;
  return RecoveryAttemptGate(config);
}

StreamHealth HealthyStream(bool has_new_observation = true) {
  StreamHealth stream;
  stream.lidar_rate_hz = 10.0;
  stream.imu_rate_hz = 100.0;
  stream.rates_known = true;
  stream.has_new_observation = has_new_observation;
  return stream;
}

TEST(RecoveryAttemptGate, RequestsExactlyOnceWhenEnteringLost) {
  auto gate = MakeGate();
  EXPECT_EQ(gate.Update(LocalizationState::kTracking, HealthyStream()),
            RecoveryAttemptReason::kNone);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kLostEntry);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kNone);
}

TEST(RecoveryAttemptGate, DoesNotRetryWhileFailedAttemptSeesOnlyHealthyData) {
  auto gate = MakeGate();
  gate.Update(LocalizationState::kLost, HealthyStream());
  for (int index = 0; index < 10; ++index) {
    EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
              RecoveryAttemptReason::kNone);
  }
}

TEST(RecoveryAttemptGate, RetriesOnceAtTheFirstHealthyObservationAfterFault) {
  auto gate = MakeGate();
  gate.Update(LocalizationState::kLost, HealthyStream());

  auto stalled = HealthyStream(false);
  stalled.lidar_stalled = true;
  EXPECT_EQ(gate.Update(LocalizationState::kLost, stalled),
            RecoveryAttemptReason::kNone);

  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kStreamRecovered);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kNone);
}

TEST(RecoveryAttemptGate, RequiresNewObservationAndClearsAfterLeavingLost) {
  auto gate = MakeGate();
  gate.Update(LocalizationState::kLost, HealthyStream());

  auto jumped = HealthyStream(false);
  jumped.timestamp_jump = true;
  gate.Update(LocalizationState::kLost, jumped);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream(false)),
            RecoveryAttemptReason::kNone);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kStreamRecovered);

  EXPECT_EQ(gate.Update(LocalizationState::kRelocalizing, HealthyStream()),
            RecoveryAttemptReason::kNone);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kLostEntry);
}

TEST(RecoveryAttemptGate, TreatsRateAndTimestampFaultsAsUnhealthy) {
  auto gate = MakeGate();
  gate.Update(LocalizationState::kLost, HealthyStream());

  auto slow = HealthyStream(false);
  slow.lidar_rate_hz = 4.9;
  gate.Update(LocalizationState::kLost, slow);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kStreamRecovered);

  gate.Update(LocalizationState::kLost, HealthyStream());
  auto rollback = HealthyStream(false);
  rollback.timestamp_monotonic = false;
  gate.Update(LocalizationState::kLost, rollback);
  EXPECT_EQ(gate.Update(LocalizationState::kLost, HealthyStream()),
            RecoveryAttemptReason::kStreamRecovered);
}

TEST(RecoveryAttemptGate, HandlesImuStallAndUnknownRateRecovery) {
  auto gate = MakeGate();
  gate.Update(LocalizationState::kLost, HealthyStream());

  auto imu_stalled = HealthyStream(false);
  imu_stalled.imu_stalled = true;
  gate.Update(LocalizationState::kLost, imu_stalled);

  auto unknown_rate = HealthyStream(true);
  unknown_rate.rates_known = false;
  unknown_rate.lidar_rate_hz = 0.0;
  unknown_rate.imu_rate_hz = 0.0;
  EXPECT_EQ(gate.Update(LocalizationState::kLost, unknown_rate),
            RecoveryAttemptReason::kStreamRecovered);
}

}  // namespace
```

- [x] **Step 2: Add a temporary gtest target and run it to verify it fails**

Add the target immediately below `test_localization_state_machine` in the `BUILD_TESTING` block:

```cmake
  ament_add_gtest(test_recovery_attempt_gate
    test/test_recovery_attempt_gate.cpp)
  target_include_directories(test_recovery_attempt_gate PRIVATE include)
```

Run:

```bash
cd /home/robot/slam_ws
colcon build --packages-select lio_sam_hesai --cmake-force-configure
```

Expected: build fails because `localization_health/recovery_attempt_gate.h` does not yet exist.

- [x] **Step 3: Implement the minimal gate and link it into production and test targets**

Create `include/localization_health/recovery_attempt_gate.h`:

```cpp
#ifndef LIO_SAM_HESAI_LOCALIZATION_HEALTH_RECOVERY_ATTEMPT_GATE_H
#define LIO_SAM_HESAI_LOCALIZATION_HEALTH_RECOVERY_ATTEMPT_GATE_H

#include "localization_health/health_types.h"

namespace localization_health {

enum class RecoveryAttemptReason {
  kNone,
  kLostEntry,
  kStreamRecovered,
};

const char* ToString(RecoveryAttemptReason reason);

class RecoveryAttemptGate {
 public:
  struct Config {
    double nominal_lidar_rate_hz = 10.0;
    double nominal_imu_rate_hz = 100.0;
    double min_stream_rate_ratio = 0.5;
  };

  explicit RecoveryAttemptGate(const Config& config);

  [[nodiscard]] RecoveryAttemptReason Update(LocalizationState state,
                                              const StreamHealth& stream);
  void Reset();

 private:
  [[nodiscard]] bool IsStreamHealthy(const StreamHealth& stream) const;

  Config config_;
  LocalizationState previous_state_ = LocalizationState::kInit;
  bool saw_stream_fault_ = false;
};

}  // namespace localization_health

#endif  // LIO_SAM_HESAI_LOCALIZATION_HEALTH_RECOVERY_ATTEMPT_GATE_H
```

Create `src/localization_health/recovery_attempt_gate.cpp`:

```cpp
#include "localization_health/recovery_attempt_gate.h"

#include <cmath>
#include <stdexcept>

namespace localization_health {

namespace {

bool IsPositiveFinite(double value) {
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

const char* ToString(RecoveryAttemptReason reason) {
  switch (reason) {
    case RecoveryAttemptReason::kNone:
      return "none";
    case RecoveryAttemptReason::kLostEntry:
      return "lost_entry";
    case RecoveryAttemptReason::kStreamRecovered:
      return "stream_recovered";
  }
  return "unknown";
}

RecoveryAttemptGate::RecoveryAttemptGate(const Config& config) : config_(config) {
  if (!IsPositiveFinite(config_.nominal_lidar_rate_hz) ||
      !IsPositiveFinite(config_.nominal_imu_rate_hz) ||
      !IsPositiveFinite(config_.min_stream_rate_ratio) ||
      config_.min_stream_rate_ratio > 1.0) {
    throw std::invalid_argument("Invalid recovery attempt gate configuration");
  }
}

RecoveryAttemptReason RecoveryAttemptGate::Update(
    LocalizationState state, const StreamHealth& stream) {
  const bool entered_lost = previous_state_ != LocalizationState::kLost &&
                            state == LocalizationState::kLost;
  previous_state_ = state;
  if (state != LocalizationState::kLost) {
    saw_stream_fault_ = false;
    return RecoveryAttemptReason::kNone;
  }

  const bool stream_healthy = IsStreamHealthy(stream);
  if (entered_lost) {
    saw_stream_fault_ = !stream_healthy;
    return RecoveryAttemptReason::kLostEntry;
  }
  if (!stream_healthy) {
    saw_stream_fault_ = true;
    return RecoveryAttemptReason::kNone;
  }
  if (saw_stream_fault_ && stream.has_new_observation) {
    saw_stream_fault_ = false;
    return RecoveryAttemptReason::kStreamRecovered;
  }
  return RecoveryAttemptReason::kNone;
}

void RecoveryAttemptGate::Reset() {
  previous_state_ = LocalizationState::kInit;
  saw_stream_fault_ = false;
}

bool RecoveryAttemptGate::IsStreamHealthy(const StreamHealth& stream) const {
  if (stream.lidar_stalled || stream.imu_stalled ||
      !stream.timestamp_monotonic || stream.timestamp_jump) {
    return false;
  }
  if (!stream.rates_known) {
    return true;
  }
  return std::isfinite(stream.lidar_rate_hz) &&
         std::isfinite(stream.imu_rate_hz) &&
         stream.lidar_rate_hz >=
             config_.nominal_lidar_rate_hz * config_.min_stream_rate_ratio &&
         stream.imu_rate_hz >=
             config_.nominal_imu_rate_hz * config_.min_stream_rate_ratio;
}

}  // namespace localization_health
```

Replace the temporary gtest target with:

```cmake
  ament_add_gtest(test_recovery_attempt_gate
    test/test_recovery_attempt_gate.cpp
    src/localization_health/recovery_attempt_gate.cpp)
  target_include_directories(test_recovery_attempt_gate PRIVATE include)
```

Add the production source next to the other health sources:

```cmake
  src/localization_health/data_stream_watchdog.cpp
  src/localization_health/localization_state_machine.cpp
  src/localization_health/recovery_attempt_gate.cpp
```

- [x] **Step 4: Run the isolated gate tests**

Run:

```bash
cd /home/robot/slam_ws
colcon build --packages-select lio_sam_hesai --cmake-force-configure
ctest --test-dir build/lio_sam_hesai --output-on-failure -R '^test_recovery_attempt_gate$'
```

Expected: target builds and all six `RecoveryAttemptGate` tests pass.

- [x] **Step 5: Commit the isolated policy**

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
git add CMakeLists.txt include/localization_health/recovery_attempt_gate.h \
  src/localization_health/recovery_attempt_gate.cpp test/test_recovery_attempt_gate.cpp
git commit -m "feat(health): add LOST recovery attempt gate"
```

### Task 2: Make automatic reinitialization a one-shot worker request

**Files:**
- Modify: `include/slam/localization.h:25-46,128-155`
- Modify: `src/slam/localization.cpp:91-97,462-476`
- Modify: `test/test_auto_relocalization_parameter_surface.cpp:220-280`

- [x] **Step 1: Write the failing source-level contract test**

Add this test after `SystemIntegration.WiresLocalizationHealthObservability` in
`test/test_auto_relocalization_parameter_surface.cpp`:

```cpp
TEST(AutomaticRecoverySurface, UsesOneShotWorkerOwnedReinitialization) {
  ExpectToken("include/slam/localization.h",
              "void RequestAutomaticRelocalization()");
  ExpectToken("include/slam/localization.h",
              "automatic_relocalization_request_pending_");
  ExpectToken("include/slam/localization.h",
              "waiting_for_automatic_relocalization_request_");
  ExpectToken("src/slam/localization.cpp",
              "automatic_relocalization_request_pending_.exchange(false)");
  ExpectToken("src/slam/localization.cpp",
              "waiting_for_automatic_relocalization_request_.store(true)");
  ExpectToken("src/slam/localization.cpp",
              "waiting_for_automatic_relocalization_request_.store(false)");
  ExpectToken("src/slam/localization.cpp", "InitPreintegration()");
}
```

- [x] **Step 2: Run the focused test to verify it fails**

Run:

```bash
cd /home/robot/slam_ws
colcon build --packages-select lio_sam_hesai
ctest --test-dir build/lio_sam_hesai --output-on-failure \
  -R '^test_auto_relocalization_parameter_surface$'
```

Expected: `AutomaticRecoverySurface.UsesOneShotWorkerOwnedReinitialization`
fails because the public request path and worker-owned waiting state do not exist.

- [x] **Step 3: Add the thread-safe request interface and consume it only in the localization worker**

In `include/slam/localization.h`, add the public method immediately after `SetInitPose`:

```cpp
    void RequestAutomaticRelocalization();
```

Add the two atomic members beside `has_init_`:

```cpp
    std::atomic_bool automatic_relocalization_request_pending_{false};
    std::atomic_bool waiting_for_automatic_relocalization_request_{false};
```

In `src/slam/localization.cpp`, update the manual path so it cancels the automatic
wait before recording the manual pose:

```cpp
void Localization::SetInitPose(const Mat4d& pose) {
  waiting_for_automatic_relocalization_request_.store(false);
  std::lock_guard<std::mutex> lg(mutex_init_pose_optional_);
  init_pose_optional_ = MapLidarPoseToMapImuPose(pose);
  has_init_ = false;
}

void Localization::RequestAutomaticRelocalization() {
  automatic_relocalization_request_pending_.store(true);
}
```

At the top of each `Run()` loop iteration, immediately after assigning
`curr_time_us_`, consume and apply the request:

```cpp
    const bool automatic_relocalization_requested =
        automatic_relocalization_request_pending_.exchange(false);
    if (automatic_relocalization_requested) {
      has_init_.store(false);
      waiting_for_automatic_relocalization_request_.store(false);
      consistency_ref_pose_.reset();
      consistency_hit_count_ = 0;
      InitPreintegration();
    }
```

Replace the existing `if (!has_init_)` initialization block with:

```cpp
    if (!has_init_) {
      if (waiting_for_automatic_relocalization_request_.load()) {
        continue;
      }

      has_init_ = Init();
      if (automatic_relocalization_requested && !has_init_) {
        waiting_for_automatic_relocalization_request_.store(true);
      }
      if (has_init_) {
        InitPreintegration();
      }
      continue;
    }
```

This preserves cold-start retries when no automatic request has occurred, coalesces
multiple `System` requests into one pending bit, and makes a failed automatic attempt
wait until a later gate action or `/initialpose` clears the wait.

- [x] **Step 4: Run the contract and existing automatic relocalization tests**

Run:

```bash
cd /home/robot/slam_ws
colcon build --packages-select lio_sam_hesai
ctest --test-dir build/lio_sam_hesai --output-on-failure \
  -R '^(test_auto_relocalization_parameter_surface|test_relocalization_acceptance_policy|test_scan_context)$'
```

Expected: all selected tests pass.

- [x] **Step 5: Commit the one-shot worker path**

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
git add include/slam/localization.h src/slam/localization.cpp \
  test/test_auto_relocalization_parameter_surface.cpp
git commit -m "feat(localization): add one-shot automatic reinitialization"
```

### Task 3: Wire the gate through System and diagnostics

**Files:**
- Modify: `include/slam/system.h:10-24,106-112,151-156`
- Modify: `src/slam/system.cpp:70-120,804-890`
- Modify: `test/test_auto_relocalization_parameter_surface.cpp:190-280`

- [x] **Step 1: Extend the source-level integration test before wiring production code**

In `SystemIntegration.WiresLocalizationHealthObservability`, add the following header
tokens to the `header_tokens` array and adjust its size from 8 to 10:

```cpp
    "localization_health/recovery_attempt_gate.h",
    "recovery_attempt_gate_",
```

Add these source tokens to `source_tokens` and adjust the array size from 28 to 32:

```cpp
    "recovery_attempt_gate_->Update(report.state, inputs.stream)",
    "RequestAutomaticRelocalization",
    "auto_relocalization_trigger",
    "ToString(recovery_attempt_reason)",
```

Add `"src/localization_health/recovery_attempt_gate.cpp"` to `cmake_tokens` and
adjust its size from 4 to 5.

- [x] **Step 2: Run the integration-surface test to verify it fails**

Run:

```bash
cd /home/robot/slam_ws
colcon build --packages-select lio_sam_hesai
ctest --test-dir build/lio_sam_hesai --output-on-failure \
  -R '^test_auto_relocalization_parameter_surface$'
```

Expected: the existing target fails because `System` does not yet include, own, or
invoke `RecoveryAttemptGate`.

- [x] **Step 3: Add System ownership, action dispatch, and diagnostic observability**

Include the gate in `include/slam/system.h`:

```cpp
#include <string>

#include "localization_health/recovery_attempt_gate.h"
```

Add members beside `localization_state_machine_` and the existing health fields:

```cpp
  std::unique_ptr<localization_health::RecoveryAttemptGate>
      recovery_attempt_gate_;
  std::string last_auto_relocalization_trigger_{"none"};
```

After creating `localization_state_machine_` in `System`'s constructor, create the
gate from the same rate configuration:

```cpp
  localization_health::RecoveryAttemptGate::Config recovery_config;
  recovery_config.nominal_lidar_rate_hz = config.health_nominal_lidar_rate_hz_;
  recovery_config.nominal_imu_rate_hz = config.health_nominal_imu_rate_hz_;
  recovery_config.min_stream_rate_ratio = config.health_min_rate_ratio_;
  recovery_attempt_gate_ =
      std::make_unique<localization_health::RecoveryAttemptGate>(recovery_config);
```

Require the gate in `PublishLocalizationHealth()`'s early return. Immediately after
the state machine update and covariance assignment, dispatch a non-empty action only
when automatic relocalization is enabled:

```cpp
  const auto recovery_attempt_reason =
      recovery_attempt_gate_->Update(report.state, inputs.stream);
  if (recovery_attempt_reason !=
          localization_health::RecoveryAttemptReason::kNone &&
      config.localization_auto_relocalization_ && localization_ptr_) {
    last_auto_relocalization_trigger_ =
        localization_health::ToString(recovery_attempt_reason);
    LOG(INFO) << "Request automatic relocalization: "
              << last_auto_relocalization_trigger_;
    localization_ptr_->RequestAutomaticRelocalization();
  }
```

Publish the value with the existing diagnostic helper:

```cpp
  add_value("auto_relocalization_trigger",
            last_auto_relocalization_trigger_);
```

Do not add a second retry timer, call `Init()` from `System`, or modify a state-machine
state directly. The gate updates even while `localization.auto_relocalization` is
false, but dispatch remains disabled in that configuration.

- [x] **Step 4: Run focused health and integration tests**

Run:

```bash
cd /home/robot/slam_ws
colcon build --packages-select lio_sam_hesai
ctest --test-dir build/lio_sam_hesai --output-on-failure \
  -R '^(test_recovery_attempt_gate|test_auto_relocalization_parameter_surface|test_data_stream_watchdog|test_localization_state_machine)$'
```

Expected: all selected tests pass; the gate tests prove the retry contract and the
surface test proves the production dispatch and diagnostic key are wired.

- [x] **Step 5: Commit System integration**

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
git add include/slam/system.h src/slam/system.cpp \
  test/test_auto_relocalization_parameter_surface.cpp
git commit -m "feat(health): trigger relocalization on LOST recovery"
```

### Task 4: Verify the completed recovery loop and record its contract

**Files:**
- Modify: `docs/superpowers/specs/2026-07-29-localization-fault-self-recovery-design.md`
- Create: `docs/superpowers/plans/2026-07-29-localization-fault-self-recovery.md`

- [x] **Step 1: Mark the design document as implemented**

Change its status line to:

```markdown
状态：已实施并验证
```

- [x] **Step 2: Build the package from a clean CMake configuration**

Run:

```bash
cd /home/robot/slam_ws
colcon build --symlink-install --packages-select lio_sam_hesai --cmake-clean-cache
```

Expected: `Summary: 1 package finished` and no compiler or linker errors.

- [x] **Step 3: Run the complete enabled package test suite**

Run:

```bash
cd /home/robot/slam_ws
ctest --test-dir build/lio_sam_hesai --output-on-failure \
  -E '^(copyright|cpplint|flake8|lint_cmake|uncrustify|xmllint)$'
```

Expected: every enabled runtime/unit/integration test passes, including
`test_recovery_attempt_gate`, `test_localization_state_machine`, and the stability
Python tests. The excluded repository-wide lint groups are pre-existing and outside
this subproject's behavior.

- [x] **Step 4: Run the short replay regression harness**

Run:

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
scripts/run_localization_stability.sh quick
```

Expected: exit code 0 and the emitted observation report satisfies the approved
`hesai_sim` baseline. This confirms the normal automatic relocalization startup path
was not regressed.

- [x] **Step 5: Commit the confirmed design and implementation plan**

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
git add -f docs/superpowers/specs/2026-07-29-localization-fault-self-recovery-design.md \
  docs/superpowers/plans/2026-07-29-localization-fault-self-recovery.md
git commit -m "docs(recovery): record LOST self-recovery design"
```
