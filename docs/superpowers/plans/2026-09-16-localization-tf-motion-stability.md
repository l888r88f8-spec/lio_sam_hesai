# Localization TF Motion Stability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate the moving-vehicle `map -> base_link` sawtooth caused by publishing a delayed LiDAR measurement after a newer IMU prediction, without using external odometry.

**Architecture:** Keep the direct `map -> base_link` tree. Treat a LiDAR result as a sensor-time anchor; use the same bounded IMU predictor to bring both newly measured and fallback TF to the newest valid IMU sensor time before stamping with current ROS publication time. Preserve raw measured odometry/path. Hold the last actually emitted TF pose when prediction is unavailable.

**Tech Stack:** ROS 2 Humble, C++17, GoogleTest/CTest, Python unittest, rosbag2 replay.

**Spec:** `docs/superpowers/specs/2026-09-16-localization-tf-motion-stability-design.md`

**Workspace constraint:** Work in the current checkout as requested. Preserve the existing user edit in `launch/localization_with_nonground.launch.py`; do not stage or commit it. Its `publish_static_tf` default already makes `test_lidar_imu_calibration` fail independently of this task.

---

### Task 1: Lock the spatial regression with a failing replay contract

**Files:** `test/integration/localization_tf_continuity.py`, `test/test_localization_tf_continuity.py`

- [ ] Add a pure collector check for the forward-driving 10–30 s window. Match each new measured TF to its adjacent odometry receive event, project the TF translation delta onto the prior body X axis, and report systematic `fallback -> measured` negative steps. Keep the existing timestamp, outage, and recovery checks. Test with synthetic forward samples and a deliberate `+0.11 m, -0.075 m` sawtooth; also prove real reverse travel is not rejected by a generic no-backward rule.
- [ ] Run `python3 -m unittest test/test_localization_tf_continuity.py`; the new sawtooth test must fail against the old collector logic for the expected spatial-contract reason, not an import or syntax error.
- [ ] Add the minimal collector implementation, rerun the Python test to green, then use the preserved baseline artifact `/tmp/lio-sam-hesai-stability.J8t4vZ/tf-continuity.json` or a fresh unmodified replay to demonstrate a spatial violation against the old C++ binary. Record the exact count and distribution rather than relying on `violations: []`.

### Task 2: Make TF prediction anchor/result pairing explicit

**Files:** `include/slam/localization.h`, `src/slam/localization.cpp`, `test/test_auto_relocalization_parameter_surface.cpp` and/or a focused GoogleTest target.

- [ ] Write a failing test for `PredictMapLidarPose`: a request for result timestamp/TF epoch A must not return a prediction from a newer anchor B. Test the valid same-anchor case and time-jump reset as separate cases.
- [ ] Run the focused test and confirm expected red.
- [ ] Add the expected sensor timestamp and epoch to the prediction request; carry epoch with the committed `TfPredictionAnchor`. Reject mismatched anchors and nonmonotonic/insufficient IMU segments; keep the configured extrapolation bound at the `System` publication boundary. Update initialization and tracking anchor commits to pass their `tf_continuity_epoch`.
- [ ] Run the focused test to green and the existing `test_tf_continuity_controller` target to detect contract drift.

### Task 3: Publish new measurements and fallbacks at one sensor-time horizon

**Files:** `include/localization_health/tf_continuity_controller.h`, `src/localization_health/tf_continuity_controller.cpp`, `test/test_tf_continuity_controller.cpp`, `include/slam/system.h`, `src/slam/system.cpp`.

- [ ] First write failing controller tests: after an extrapolated pose at X=2, a new delayed measurement at X=1 must not be emitted as a current-time TF; a valid projection of that measurement to the latest IMU time may be emitted; held mode must reuse the last **published** pose, not the old measurement anchor. Cover first-anchor/no-IMU and ROS time-jump reset.
- [ ] Run `ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai --output-on-failure -R '^test_tf_continuity_controller$'` after rebuilding that target, and confirm expected red.
- [ ] Extend the controller's measured update to accept a validated projected sample (or no sample) and return a publish decision only for a current-horizon sample. Preserve measured sensor-time anchor for later predictions, publication timestamp monotonicity, 20 Hz fallback pacing, and mode diagnostics. On failure, publish/mark held only if a previously emitted pose exists; never relabel the historical measured pose as current motion.
- [ ] In `System::ProcessLocalizationResultCache`, keep raw `sampled_map_T_base` for odometry/path; obtain the latest fresh IMU segment using `result.timestamp_`, request prediction with matching timestamp/epoch, apply the same lidar-to-base and Z-offset transform as fallback, and pass that projected pose to the controller. Reuse a small private helper for the common projection path instead of duplicating sensor/TF conversions. Keep the existing health and clock-jump gates.
- [ ] Build and rerun the focused C++ tests to green; do not change mapping TF, launch defaults, external odom wiring, or optimizer state.

### Task 4: Verify the trial and preserve failure evidence

**Files:** only tests/production files above; update `docs/PROJECT_STATUS.md` solely with actual measured results if the trial passes.

- [ ] Run the relevant Python unit tests and CTest targets, then the established functional CTest gate. Report the existing `test_lidar_imu_calibration` failure separately if unchanged; never claim a full green gate while it fails.
- [ ] Build/install an up-to-date localization binary before any replay. Run `LIO_SAM_KEEP_RESULTS=1 scripts/run_localization_stability.sh quick`, `tf-continuity`, and `tiled-quick` using the same bag/map and record result paths, exit codes, TF spatial deltas, odometry/path source-time invariance, TF spacing, and health/LOST modes.
- [ ] Compare with the baseline: in the forward 10–30 s window, the old 101/101 systematic backward measured edges must disappear; median edge delta must be at least -0.01 m and no individual backward edge may exceed 0.05 m. If the correction instead exposes genuine registration jumps, stop and report the residual cause rather than adding a low-pass filter.
- [ ] Inspect `git diff --check`, final diff, and `git status`. Commit only task-owned files in focused commits. Do not touch the user launch edit. No real-vehicle navigation run is authorized by this plan.
