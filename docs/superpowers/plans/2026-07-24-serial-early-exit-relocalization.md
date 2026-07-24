# Serial Early-Exit Relocalization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Evaluate ordered Scan Context candidates one at a time and stop on a high-confidence fitness result while retaining IVOX point-level parallelism.

**Architecture:** Replace the candidate-level bounded async call in `Localization::TryAutoRelocalize` with an ordered loop. Prepare each candidate lazily, run its independent matcher, retain the best acceptable result, and stop only when `ShouldEarlyAcceptRelocalization` accepts the current fitness.

**Tech Stack:** C++17, ROS2, PCL, Eigen, GoogleTest, CMake/CTest.

---

### Task 1: Lock down serial ordered evaluation

**Files:**
- Modify: `test/test_relocalization_efficiency_surface.cpp`
- Modify: `src/slam/localization.cpp`

- [ ] **Step 1: Write the failing surface test**

Add a test that requires an indexed candidate loop, requires per-candidate early
acceptance, and rejects `RunBoundedAsync` in `src/slam/localization.cpp`.

- [ ] **Step 2: Verify the test fails**

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target test_relocalization_efficiency_surface -j2
/home/robot/slam_ws/build/lio_sam_hesai/test_relocalization_efficiency_surface \
  --gtest_filter=RelocalizationEfficiencySurface.EvaluatesCandidatesSeriallyWithEarlyExit
```

Expected: the test fails because `RunBoundedAsync` remains in the localization
path and no ordered candidate loop exists.

- [ ] **Step 3: Implement lazy serial evaluation**

In `Localization::TryAutoRelocalize`, replace the eager task preparation and
bounded async evaluation with a single indexed loop over `candidates`. Move
ground-height correction and `LoadCandidateLocalMap` into the loop, create one
matcher, run `Match`, retain the lowest finite fitness below
`fitness_threshold`, and break only when
`ShouldEarlyAcceptRelocalization(fitness, localization_sc_accept_fitness_)`
returns true.

Before starting each candidate after the first, inspect
`init_pose_optional_` under `mutex_init_pose_optional_` and stop automatic
evaluation when a manual pose is pending. Log candidate rank, score, timings,
fitness, totals, and early-exit state.

- [ ] **Step 4: Verify the focused test passes**

Run the command from Step 2. Expected: one passing test.

- [ ] **Step 5: Build and run relevant tests**

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target lio_sam_hesai_relocalization -j2
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  --output-on-failure \
  -R '^(test_relocalization_efficiency_surface|test_relocalization_acceptance_policy|test_scan_context)$'
```

Expected: the executable builds and all selected tests pass.

- [ ] **Step 6: Commit**

Commit only the implementation, test, design, and plan files. Preserve the
user's existing changes in `config/relocalization.yaml` and
`launch/localization_with_nonground.launch.py`.
