# Localization Startup Readiness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Finish localization map preparation before the node accepts lidar/IMU input or starts data workers.

**Architecture:** Extract the existing startup map/ground-map/Scan-Context preparation from `Localization::Run()` into an explicit readiness operation. `System` calls it before subscriber and worker creation, then exposes its duration in diagnostics.

**Tech Stack:** C++17, ROS 2 Humble, PCL, gtest, CMake.

---

### Task 1: Make map preparation an explicit, testable lifecycle

**Files:**
- Modify: `include/slam/localization.h`, `src/slam/localization.cpp`
- Test: `test/test_auto_relocalization_parameter_surface.cpp`

- [x] **Step 1: Write a failing lifecycle-contract test**

Require the header to expose `bool PrepareMap()` and `MapPreparationDurationMs()`, and require `Run()` to call neither `LoadGlobalMap(` nor `LoadScanContextDb(`. Run `ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai -R '^test_auto_relocalization_parameter_surface$'`; expect failure because the existing code prepares maps inside `Run()`.

- [x] **Step 2: Extract the existing preparation block**

Move the current `Run()` map, ground-map, XY kd-tree and optional Scan Context setup into `PrepareMap()`. Measure elapsed `steady_clock` time, set `map_ready_` only after all required resources are valid, and return `false` on a missing/empty required global map. `Run()` must return immediately unless preparation completed successfully.

- [x] **Step 3: Verify the lifecycle contract**

Build `test_auto_relocalization_parameter_surface` with `cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target test_auto_relocalization_parameter_surface -- -j1`, then run its CTest entry. Expected: 1/1 pass.

- [x] **Step 4: Commit**

Commit the lifecycle change with `feat(localization): prepare maps before data processing`.

### Task 2: Gate System data intake and report readiness timing

**Files:**
- Modify: `include/slam/system.h`, `src/slam/system.cpp`
- Test: `test/test_auto_relocalization_parameter_surface.cpp`

- [x] **Step 1: Extend the failing System contract**

Require `System` to construct `Localization`, call `PrepareMap()` before `InitSubscriber()` and before constructing either `Localization::Run` or `PreProcessing::Run` thread; require diagnostics to publish `map_preparation_duration_ms`. Run the focused CTest and expect failure.

- [x] **Step 2: Add the startup gate and diagnostic value**

Call `PrepareMap()` immediately after constructing `Localization`. Throw `std::runtime_error` if it returns false, then initialize publishers/subscribers and start workers. Publish `map_preparation_duration_ms` using `Localization::MapPreparationDurationMs()` beside `map_ready`.

- [x] **Step 3: Verify focused tests and build**

Run the focused CTest, then `cmake --build /home/robot/slam_ws/build/lio_sam_hesai -- -j1`. Expected: test passes and build exits 0.

- [x] **Step 4: Commit**

Commit with `feat(system): gate data pipeline on map readiness`.

### Task 3: Full regression and replay evidence

**Files:**
- Modify: this plan and `docs/superpowers/specs/2026-07-30-localization-startup-readiness-design.md`

- [x] **Step 1: Run enabled package tests**

Run `ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai --output-on-failure -E '^(copyright|cpplint|flake8|lint_cmake|uncrustify|xmllint)$'`. Expected: all enabled tests pass.

- [x] **Step 2: Install and replay**

Run `cmake --install /home/robot/slam_ws/build/lio_sam_hesai`, then `LIO_SAM_KEEP_RESULTS=1 scripts/run_localization_stability.sh quick`. Expected: exit 0 with no error samples and valid odometry ratio 1.0.

- [x] **Step 3: Mark documentation verified and commit**

Change the design status to `已实施并验证`, mark all plan checkboxes `[x]`, force-add the ignored docs, and commit with `docs(localization): record startup readiness verification`.
