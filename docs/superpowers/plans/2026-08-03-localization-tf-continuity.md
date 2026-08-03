# Localization TF Continuity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** After the first accepted localization, keep `map -> base_link` available from the existing lidar and IMU streams while clearly distinguishing measured, short IMU-extrapolated, and held transforms.

**Architecture:** Add a pure `TfContinuityController` that selects and rate-limits TF publication. `Localization` owns a mutex-protected committed navigation snapshot and predicts a map-to-lidar pose with a temporary pre-integrator; it never exposes or mutates its live optimizer state. `System` converts the predicted pose to base_link, publishes only TF on fallback paths, and exposes the selected mode in existing diagnostics.

**Tech Stack:** C++17, Eigen, existing `PreIntegration`, ROS 2 Humble (`rclcpp`, `tf2_ros`), gtest, `rclpy`, rosbag2.

---

## File structure

- Create: `include/localization_health/tf_continuity_controller.h` — pure mode, throttling and held/extrapolated decision state.
- Create: `src/localization_health/tf_continuity_controller.cpp` — validation and decision implementation.
- Create: `test/test_tf_continuity_controller.cpp` — deterministic gtest coverage of the pure selector.
- Modify: `include/slam/localization.h` — read-only committed TF-prediction snapshot and predictor API.
- Modify: `src/slam/localization.cpp` — update the snapshot only after successful commits; predict from a local pre-integrator.
- Modify: `include/slam/config_parameters.h` — `localization_tf` configuration members.
- Modify: `include/slam/system.h` — controller member and fallback/publication helpers.
- Modify: `src/slam/system.cpp` — parameter parsing, fallback TF publication and diagnostics.
- Modify: `config/relocalization.yaml` — `localization_tf` defaults.
- Modify: `CMakeLists.txt` — library source and gtest target.
- Modify: `test/test_auto_relocalization_parameter_surface.cpp` — parameter and source-order contracts.
- Create: `test/integration/localization_tf_continuity.py` — rosbag relay, TF/diagnostic/path/odometry collector and fault-profile assertions.
- Create: `test/test_localization_tf_continuity.py` — bag-independent Python contracts for the fault schedule and collector.
- Modify: `test/integration/stability_common.py` — render sensor-topic overrides into a per-run parameter file.
- Modify: `scripts/run_localization_stability.sh` — `tf-continuity` entrypoint.
- Modify: `.github/workflows/localization-stability.yml` — scheduled TF-continuity bag job.
- Modify: `docs/superpowers/specs/2026-08-03-localization-tf-continuity-design.md` and this plan only after all verification passes.

### Task 1: Add the pure TF continuity selector

**Files:**
- Create: `include/localization_health/tf_continuity_controller.h`
- Create: `src/localization_health/tf_continuity_controller.cpp`
- Create: `test/test_tf_continuity_controller.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write failing selector tests**

Create `test/test_tf_continuity_controller.cpp` with a small `Sample()` helper and these contracts:

```cpp
TEST(TfContinuityController, DoesNotPublishWithoutAnAnchor) {
  TfContinuityController controller({100000U, 500000U});
  EXPECT_FALSE(controller.Select(100000U, std::nullopt).has_value());
}

TEST(TfContinuityController, PublishesExtrapolationInsideTheLimit) {
  TfContinuityController controller({100000U, 500000U});
  controller.RecordMeasured(Sample(1000000U, 1.0));
  const auto decision = controller.Select(1200000U, Sample(1200000U, 1.2));
  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kExtrapolated);
  EXPECT_EQ(decision->sample.timestamp_us, 1200000U);
}

TEST(TfContinuityController, HoldsAnchorWhenImuPredictionIsMissingOrTooOld) {
  TfContinuityController controller({100000U, 500000U});
  controller.RecordMeasured(Sample(1000000U, 1.0));
  ASSERT_TRUE(controller.Select(1100000U, std::nullopt).has_value());
  const auto held = controller.Select(1600000U, Sample(1600000U, 1.6));
  ASSERT_TRUE(held.has_value());
  EXPECT_EQ(held->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(held->sample.timestamp_us, 1600000U);
  EXPECT_DOUBLE_EQ(held->sample.map_base_pose(0, 3), 1.0);
}

TEST(TfContinuityController, RateLimitsFallbackAndNewMeasuredSampleReplacesAnchor) {
  TfContinuityController controller({100000U, 500000U});
  controller.RecordMeasured(Sample(1000000U, 1.0));
  EXPECT_FALSE(controller.Select(1050000U, std::nullopt).has_value());
  controller.RecordMeasured(Sample(1100000U, 2.0));
  const auto held = controller.Select(1200000U, std::nullopt);
  ASSERT_TRUE(held.has_value());
  EXPECT_DOUBLE_EQ(held->sample.map_base_pose(0, 3), 2.0);
}
```

- [ ] **Step 2: Add the test target and verify RED**

Add:

```cmake
ament_add_gtest(test_tf_continuity_controller
  test/test_tf_continuity_controller.cpp
  src/localization_health/tf_continuity_controller.cpp)
target_include_directories(test_tf_continuity_controller PRIVATE include)
```

Run: `cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target test_tf_continuity_controller -- -j1`

Expected: compilation fails because `localization_health/tf_continuity_controller.h` does not exist.

- [ ] **Step 3: Implement the controller**

Define these public values in the header:

```cpp
enum class TfPublicationMode { kMeasured, kExtrapolated, kHeld, kNoAnchor };
enum class TfHoldReason { kNone, kImuUnavailable, kExtrapolationLimit, kNoAnchor };
struct TfPoseSample { Mat4d map_base_pose; TimeStampUs timestamp_us; };
struct TfPublicationDecision {
  TfPublicationMode mode;
  TfHoldReason hold_reason;
  TfPoseSample sample;
  TimeStampUs anchor_timestamp_us;
  TimeStampUs extrapolation_duration_us;
};
class TfContinuityController {
 public:
  struct Config { TimeStampUs publish_period_us; TimeStampUs max_extrapolation_us; };
  explicit TfContinuityController(Config config);
  static void Validate(const Config& config);
  TfPublicationDecision RecordMeasured(const TfPoseSample& measured);
  std::optional<TfPublicationDecision> Select(
      TimeStampUs now_us, const std::optional<TfPoseSample>& extrapolated);
  [[nodiscard]] TfPublicationMode mode() const;
  [[nodiscard]] TfHoldReason hold_reason() const;
  [[nodiscard]] std::optional<TimeStampUs> anchor_timestamp_us() const;
 private:
  Config config_;
  std::optional<TfPoseSample> anchor_;
  TimeStampUs last_publish_us_{0U};
  TfPublicationMode mode_{TfPublicationMode::kNoAnchor};
  TfHoldReason hold_reason_{TfHoldReason::kNoAnchor};
};
```

`Select()` must return no decision before an anchor or before the next period. It accepts
an extrapolated sample only when its timestamp is not older than the anchor and its span is
at most `max_extrapolation_us`; otherwise it returns the anchor pose stamped `now_us` in
`kHeld` mode. `RecordMeasured()` always replaces the anchor and returns an immediate
`kMeasured` decision.

- [ ] **Step 4: Verify GREEN**

Run: `ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai --output-on-failure -R '^test_tf_continuity_controller$'`

Expected: all four selector tests pass.

- [ ] **Step 5: Commit the pure unit**

```bash
git add include/localization_health/tf_continuity_controller.h \
  src/localization_health/tf_continuity_controller.cpp \
  test/test_tf_continuity_controller.cpp CMakeLists.txt
git commit -m "feat(health): select continuous TF fallback mode"
```

### Task 2: Add a read-only localization prediction snapshot

**Files:**
- Modify: `include/slam/localization.h`
- Modify: `src/slam/localization.cpp`
- Modify: `test/test_auto_relocalization_parameter_surface.cpp`

- [ ] **Step 1: Write the failing source contract**

Add `LocalizationTfPredictionSurface` to `test_auto_relocalization_parameter_surface.cpp`.
It must assert that `localization.h` declares `PredictMapLidarPose`, `TfPredictionAnchor`,
and `mutex_tf_prediction_anchor_`; then extract `DiscardTimedOutFrameIfNeeded` and assert it
does **not** contain `UpdateTfPredictionAnchor`. Extract `Localization::Init()` and
`Localization::Run()` and require `UpdateTfPredictionAnchor(initialized_nav_state)` and
`UpdateTfPredictionAnchor(curr_nav_state)` after their final watchdog `Complete(..., false)`
calls and before their matching `CacheResultToSystem(...)` calls.

Run: `ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai --output-on-failure -R '^test_auto_relocalization_parameter_surface$'`

Expected: FAIL because the prediction snapshot symbols are absent.

- [ ] **Step 2: Define the snapshot and predictor interface**

Add to `Localization`:

```cpp
struct MapLidarPrediction {
  Mat4d map_lidar_pose{Mat4d::Identity()};
  TimeStampUs timestamp_{0U};
};
[[nodiscard]] std::optional<MapLidarPrediction> PredictMapLidarPose(
    const std::vector<IMUData>& imu_segment) const;
```

Add the private `TfPredictionAnchor { NavStateData nav_state; Mat4d map_lidar_pose;
TimeStampUs timestamp; }`, a `mutable std::mutex mutex_tf_prediction_anchor_`, an optional
anchor, `UpdateTfPredictionAnchor(const NavStateData&)`, and
`MakePreIntegrationConfig() const`. Refactor `InitPreintegration()` to construct its two live
integrators with `MakePreIntegrationConfig()`; do not change its live reset behavior.

- [ ] **Step 3: Implement an isolated prediction path**

Implement `UpdateTfPredictionAnchor()` by constructing `map_lidar_pose` through the existing
`MapImuPoseToMapLidarPose(nav_state.Pose())`, then assigning the complete anchor under its
mutex. Call it only after each successful final watchdog gate, before queuing the matching
`LocalizationResult`:

```cpp
frame_processing_watchdog_.Complete(tracking_commit_now_us, false);
UpdateTfPredictionAnchor(curr_nav_state);
CacheResultToSystem(result);
```

`PredictMapLidarPose()` copies the anchor under the mutex, rejects an empty segment, an anchor
timestamp mismatch, a non-increasing segment or a segment with fewer than two samples. It then
uses a stack-local `PreIntegration`:

```cpp
PreIntegration predictor(MakePreIntegrationConfig());
predictor.SetGyroAccBias(anchor.nav_state.bg_, anchor.nav_state.ba_);
predictor.IntegrateDataSegment(imu_segment);
const NavStateData predicted = predictor.Predict(anchor.nav_state);
return MapLidarPrediction{MapImuPoseToMapLidarPose(predicted.Pose()),
                          predicted.timestamp_};
```

This method must never call `Reset()`, `SetGyroAccBias()` or `IntegrateDataSegment()` on
`pre_integration_ptr_` or `odom_pre_integration_ptr_`.

- [ ] **Step 4: Build and verify GREEN**

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target lio_sam_hesai_relocalization_lib test_auto_relocalization_parameter_surface -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai --output-on-failure \
  -R '^test_auto_relocalization_parameter_surface$'
```

Expected: library builds and the source contract passes.

- [ ] **Step 5: Commit snapshot isolation**

```bash
git add include/slam/localization.h src/slam/localization.cpp \
  test/test_auto_relocalization_parameter_surface.cpp
git commit -m "feat(localization): expose read-only TF prediction"
```

### Task 3: Wire parameters, fallback TF publication and diagnostics

**Files:**
- Modify: `include/slam/config_parameters.h`
- Modify: `include/slam/system.h`
- Modify: `src/slam/system.cpp`
- Modify: `config/relocalization.yaml`
- Modify: `CMakeLists.txt`
- Modify: `test/test_auto_relocalization_parameter_surface.cpp`

- [ ] **Step 1: Extend the failing System surface test**

Add a YAML helper `ExpectLocalizationTfYamlParameter()` and a table requiring these member,
parameter and YAML triples:

```cpp
{"publish_rate_hz", "localization_tf_publish_rate_hz_"},
{"max_extrapolation_seconds", "localization_tf_max_extrapolation_seconds_"},
{"enable_imu_extrapolation", "localization_tf_enable_imu_extrapolation_"},
```

Require the source tokens `TfContinuityController`, `PublishTfFallback`, `tf_mode`,
`tf_anchor_age_ms`, `tf_extrapolation_age_ms`, and `tf_hold_reason`. Extract `System::Run()`
and assert that its `!had_new_result` branch calls `PublishLocalizationHealth()` before
`PublishTfFallback()`. Assert `PublishTfFallback()` contains `PublishTF` but neither
`localization_odom_pub_->publish` nor `PublishLocalizationPath`.

Run the same surface CTest. Expected: FAIL because the parameters and fallback method are absent.

- [ ] **Step 2: Add validated configuration and controller ownership**

Add the three configuration fields with defaults `20.0`, `0.5`, and `true`. Parse
`localization_tf.*` in `System::InitConfigParameters()` and add this YAML block:

```yaml
    localization_tf:
      publish_rate_hz: 20.0
      max_extrapolation_seconds: 0.5
      enable_imu_extrapolation: true
```

Create `TfContinuityController::Config` in the `System` constructor after health parameters
are loaded. Convert seconds with the existing checked `SecondsToMicroseconds()` helper, require
`max_extrapolation_seconds <= localization_health.stream_timeout`, call `Validate()` before
worker startup, and store the controller in `System`.

- [ ] **Step 3: Publish measured and fallback TF without output pollution**

Replace the direct measured `PublishTF(map_base_pose, result.timestamp_)` call with:

```cpp
const auto measured = tf_continuity_controller_.RecordMeasured(
    {map_base_pose, result.timestamp_});
PublishTF(measured.sample.map_base_pose, measured.sample.timestamp_us);
```

Implement `PublishTfFallback()` as follows:

1. Obtain `now_us` from the node ROS clock and the last controller anchor timestamp.
2. If IMU extrapolation is enabled, obtain `LatestData()` and then
   `GetDataSegment(anchor_timestamp, latest.timestamp_)` from `imu_data_searcher_ptr_`.
3. Pass that segment to `localization_ptr_->PredictMapLidarPose()`. When it returns a pose,
   multiply by the existing base-to-lidar transform and apply `last_localization_z_offset_`
   when the ground-height constraint is active.
4. Pass the optional map-to-base sample into `tf_continuity_controller_.Select(now_us, sample)`.
   If no decision is due, return. Otherwise call `PublishTF()` once with its decision sample.

Do not publish odometry, path, current cloud or call `CacheResultToSystem()` here. In
`System::Run()`, call this helper only after `PublishLocalizationHealth()` when no new result
was processed.

In `PublishLocalizationHealth()`, append controller mode, anchor age, extrapolation duration
and hold reason as the four specified diagnostic key/value pairs. Use `ToString` helpers for
the two enums; add them next to the controller values rather than serializing integer enums.

Add `src/localization_health/tf_continuity_controller.cpp` to
`${PROJECT_NAME}_relocalization_lib` in `CMakeLists.txt`.

- [ ] **Step 4: Verify parameters and integration contracts**

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target lio_sam_hesai_relocalization_lib test_auto_relocalization_parameter_surface -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai --output-on-failure \
  -R '^(test_tf_continuity_controller|test_auto_relocalization_parameter_surface)$'
```

Expected: both targets build and all selected tests pass.

- [ ] **Step 5: Commit System wiring**

```bash
git add include/slam/config_parameters.h include/slam/system.h src/slam/system.cpp \
  config/relocalization.yaml CMakeLists.txt test/test_auto_relocalization_parameter_surface.cpp
git commit -m "feat(system): publish continuous localization TF"
```

### Task 4: Add deterministic rosbag fault injection and assertions

**Files:**
- Create: `test/integration/localization_tf_continuity.py`
- Modify: `test/integration/stability_common.py`
- Modify: `scripts/run_localization_stability.sh`
- Modify: `.github/workflows/localization-stability.yml`

- [ ] **Step 1: Write pure Python tests for schedule and observation contracts**

Create `test/test_localization_tf_continuity.py`. Test a pure `FaultSchedule` using replay-clock
offsets and a `TfContinuityCollector`:

```python
def test_schedule_keeps_imu_during_lidar_only_outage():
    schedule = FaultSchedule(lidar_outage=(30.0, 40.0), all_outage=(50.0, 60.0))
    assert not schedule.forward_lidar(35.0)
    assert schedule.forward_imu(35.0)
    assert not schedule.forward_lidar(55.0)
    assert not schedule.forward_imu(55.0)

def test_collector_requires_each_fallback_mode_and_recovery():
    collector = TfContinuityCollector(20.0)
    collector.tf(29.0, "measured")
    collector.tf(30.1, "extrapolated")
    collector.tf(31.0, "held")
    collector.tf(55.0, "held", "imu_unavailable")
    collector.tf(65.0, "measured")
    assert collector.assert_contracts() == []
```

Run: `python3 -m unittest test/test_localization_tf_continuity.py`

Expected: FAIL because the module and classes do not exist.

- [ ] **Step 2: Implement the relay and collector**

In `test/integration/localization_tf_continuity.py`, create one `rclpy` node that:

- subscribes to raw `/lidar_points`, `/imu`, `/clock`, `/tf`, `/diagnostics`,
  `/lio_sam/localization/odometry`, and `/lio_sam/localization/path`;
- republishes the two sensor topics to `/tf_continuity_test/lidar_points` and
  `/tf_continuity_test/imu` with sensor-data QoS;
- uses message header time minus first `/clock` to apply exactly these intervals:
  lidar-only drop `[30.0, 40.0)`, both-sensor drop `[50.0, 60.0)`;
- records `map -> base_link` transforms and the `tf_mode`, `tf_hold_reason`, state and
  `auto_relocalization_trigger` diagnostic keys at replay-clock time;
- fails if it does not observe measured before 30 s, extrapolated between 30--30.5 s, held
  before 40 s, held with `imu_unavailable` between 50--60 s, and measured after 60 s; or if
  TF timestamps regress, TF spacing exceeds 0.10 s while a mode is active, any odometry or
  path message is published during either outage, or `stream_recovered` is emitted more than
  once.

Launch the normal localization node with a generated parameter file whose two sensor topic
leaves are replaced by the relay topics. Extend `write_runtime_parameters()` with an optional
`topic_overrides` mapping, validate each supplied leaf appears once under `sensor_topic`, and
write JSON-quoted replacement values while retaining the existing map-path replacement checks.

Start `ros2 bag play <bag> --clock 100 --rate 1.0 --start-offset 0
--disable-keyboard-controls`; the bag continues emitting `/clock` through relay outages, so the
held-TF branch is observable even though the relay drops both sensor streams.

- [ ] **Step 3: Verify the pure harness tests**

Run: `python3 -m unittest test/test_localization_tf_continuity.py`

Expected: the schedule/collector tests pass without ROS or a bag.

- [ ] **Step 4: Add the developer and CI entrypoints**

Add `tf-continuity` to the usage and accepted modes in
`scripts/run_localization_stability.sh`; invoke:

```bash
python3 test/integration/localization_tf_continuity.py \
  --bag "$bag_dir" --map "$map_dir" \
  --params-template config/relocalization.yaml --output-dir "$results_dir"
```

Add a scheduled workflow step after the existing quick replay that runs
`LIO_SAM_KEEP_RESULTS=1 scripts/run_localization_stability.sh tf-continuity` and uploads its
result directory on failure. Keep pull-request CI limited to the pure Python test; the bag job
remains scheduled because it takes more than a normal unit-test window.

- [ ] **Step 5: Commit the fault-injection harness**

```bash
git add test/integration/localization_tf_continuity.py \
  test/test_localization_tf_continuity.py test/integration/stability_common.py \
  scripts/run_localization_stability.sh .github/workflows/localization-stability.yml
git commit -m "test(localization): verify TF continuity through sensor outages"
```

### Task 5: Full verification and documentation

**Files:**
- Modify: `docs/superpowers/specs/2026-08-03-localization-tf-continuity-design.md`
- Modify: `docs/superpowers/plans/2026-08-03-localization-tf-continuity.md`

- [ ] **Step 1: Build and run all enabled package tests**

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai --output-on-failure \
  -E '^(copyright|cpplint|flake8|lint_cmake|uncrustify|xmllint)$'
```

Expected: build exits 0 and all enabled CTests pass, including the new controller and Python
unit contracts.

- [ ] **Step 2: Install and run both bag acceptance commands**

Run:

```bash
cmake --install /home/robot/slam_ws/build/lio_sam_hesai
LIO_SAM_KEEP_RESULTS=1 scripts/run_localization_stability.sh quick
LIO_SAM_KEEP_RESULTS=1 scripts/run_localization_stability.sh tf-continuity
```

Expected: both commands exit 0; the quick replay stays within its existing baseline and the
TF-continuity run writes an observation report proving all three modes and the one-shot
recovery behavior. FIFO `EPERM` warnings remain an allowed runtime fallback when the host
lacks `CAP_SYS_NICE`.

- [ ] **Step 3: Record verified status and commit**

After both bag commands succeed, change the design status from `已确认，待实施` to
`已实施并验证`, check every task checkbox in this plan, force-add the two ignored Markdown
files, and commit:

```bash
git add -f docs/superpowers/specs/2026-08-03-localization-tf-continuity-design.md \
  docs/superpowers/plans/2026-08-03-localization-tf-continuity.md
git commit -m "docs(localization): record TF continuity verification"
```

- [ ] **Step 4: Final checks**

Run:

```bash
git diff --check HEAD~5..HEAD
git status -s
```

Expected: no whitespace errors and a clean worktree.
