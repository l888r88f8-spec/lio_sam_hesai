# Output Topic Parameter Group Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move both externally configurable output topics into the canonical `output_topic` group and remove the obsolete `output.*` and `sensor_topic.non_ground_cloud_topic` names.

**Architecture:** Keep input and sensor-facing topics under `sensor_topic`, while publisher destinations live under `output_topic`. This is a hard cutover: C++ readers, both YAML files, schema tests, and the original migration plan change together without compatibility aliases.

**Tech Stack:** ROS 2 Humble parameters, C++17, YAML, GoogleTest, CMake/CTest, colcon

---

### Task 1: Define the output-topic contract with failing tests

**Files:**
- Modify: `test/test_unified_parameter_schema.cpp:49-58`
- Test: `test/test_unified_parameter_schema.cpp`

- [ ] **Step 1: Change the schema contract to the approved names**

Replace the existing non-ground contract and add explicit hard-cutover entries:

```cpp
  {"nonGroundCloudTopic", "output_topic.non_ground_cloud_topic",
    kGroundSegmentationSurfaces},
  {"sensor_topic.non_ground_cloud_topic",
    "output_topic.non_ground_cloud_topic",
    kGroundSegmentationSurfaces},
  {"output.mapping_odom_topic", "output_topic.mapping_odom_topic",
    kRelocalizationSurfaces},
```

The duplicate canonical non-ground entry is intentional: the test must reject
both the original camelCase name and the first grouped-but-misclassified name.

- [ ] **Step 2: Build and run the test to verify RED**

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target test_unified_parameter_schema -j2
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  --output-on-failure -R '^test_unified_parameter_schema$'
```

Expected: the test fails because production readers/YAML still contain
`sensor_topic.non_ground_cloud_topic` and `output.mapping_odom_topic`, while the
new `output_topic.*` names are absent.

- [ ] **Step 3: Commit the RED contract**

```bash
git add test/test_unified_parameter_schema.cpp
git commit -m "test(config): define output topic parameter group"
```

### Task 2: Migrate readers, YAML, and migration documentation

**Files:**
- Modify: `src/imageProjection.cpp:331-340`
- Modify: `src/slam/system.cpp:94-96`
- Modify: `config/mapping.yaml:5-14`
- Modify: `config/relocalization.yaml:5-20`
- Modify: `docs/superpowers/plans/2026-07-24-unified-ros-parameters.md:257-269`

- [ ] **Step 1: Move the image-projection publisher parameter**

Use the canonical name for both declaration and read:

```cpp
        declare_parameter(
            "output_topic.non_ground_cloud_topic",
            "lio_sam/deskew/cloud_nonground");
        get_parameter(
            "output_topic.non_ground_cloud_topic",
            nonGroundCloudTopic_);
```

Do not declare or read `sensor_topic.non_ground_cloud_topic`.

- [ ] **Step 2: Move the relocalization odometry publisher parameter**

Change the System reader to:

```cpp
  util::param(this, "output_topic.mapping_odom_topic",
              config.mapping_odom_topic_, config.mapping_odom_topic_);
```

Do not retain `output.mapping_odom_topic` as an alias.

- [ ] **Step 3: Restructure both YAML files**

In `config/mapping.yaml`, remove `non_ground_cloud_topic` from
`sensor_topic` and add:

```yaml
    output_topic:
      non_ground_cloud_topic: "lio_sam/cloud_nonground"
```

In `config/relocalization.yaml`, remove the same key from `sensor_topic`,
rename the existing `output` mapping to `output_topic`, and keep both values:

```yaml
    output_topic:
      mapping_odom_topic: "lio_sam/localization/odometry"
      non_ground_cloud_topic: "lio_sam/cloud_nonground"
```

- [ ] **Step 4: Correct the original migration plan**

Replace its `sensor_topic.non_ground_cloud_topic` instruction with
`output_topic.non_ground_cloud_topic`, and document
`output.mapping_odom_topic` → `output_topic.mapping_odom_topic`.

- [ ] **Step 5: Verify GREEN**

Run:

```bash
python3 -c 'import yaml; [yaml.safe_load(open(p)) for p in ["config/mapping.yaml", "config/relocalization.yaml"]]'
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target test_unified_parameter_schema \
           test_ground_patch_parameter_surface \
           test_auto_relocalization_parameter_surface -j2
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  --output-on-failure \
  -R '^(test_unified_parameter_schema|test_ground_patch_parameter_surface|test_auto_relocalization_parameter_surface)$'
```

Expected: valid YAML and all three tests pass.

- [ ] **Step 6: Commit the migration**

```bash
git add src/imageProjection.cpp src/slam/system.cpp \
  config/mapping.yaml config/relocalization.yaml
git add -f docs/superpowers/plans/2026-07-24-unified-ros-parameters.md
git commit -m "refactor(config): group publisher output topics"
```

### Task 3: Build and runtime-smoke the hard cutover

**Files:**
- No source changes expected.

- [ ] **Step 1: Build the package with tests**

Run:

```bash
cd /home/robot/slam_ws
colcon build --symlink-install --packages-select lio_sam_hesai \
  --cmake-args -DBUILD_TESTING=ON
```

Expected: `Summary: 1 package finished`.

- [ ] **Step 2: Run the relevant regression suite**

Run:

```bash
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  --output-on-failure \
  -R '^(test_lidar_imu_calibration|test_unified_parameter_schema|test_ground_patch_parameter_surface|test_auto_relocalization_parameter_surface|test_relocalization_efficiency_surface|test_relocalization_acceptance_policy|test_ground_map_source|test_scan_context)$'
```

Expected: all eight selected tests pass.

- [ ] **Step 3: Audit old and new external names**

Confirm production C++ and YAML contain:

```text
output_topic.mapping_odom_topic
output_topic.non_ground_cloud_topic
```

and contain neither:

```text
output.mapping_odom_topic
sensor_topic.non_ground_cloud_topic
```

Documentation and negative-test fixtures are exempt from the legacy-name
search.

- [ ] **Step 4: Runtime-smoke relocalization**

Launch in an isolated ROS domain for a bounded interval:

```bash
ROS_DOMAIN_ID=181 ros2 launch lio_sam_hesai \
  localization_with_nonground.launch.py \
  use_rviz:=false publish_static_tf:=false
```

While it is alive, run:

```bash
ROS_DOMAIN_ID=181 ros2 param list /lio_sam_hesai_relocalization
ROS_DOMAIN_ID=181 ros2 param list /lio_sam_hesai_ground_segmentation
```

Expected: `output_topic.mapping_odom_topic` appears on relocalization,
`output_topic.non_ground_cloud_topic` appears on ground segmentation, and the
two obsolete names do not appear. Logs contain no parameter declaration, type,
or undeclared-parameter failures.

- [ ] **Step 5: Verify repository state**

Run:

```bash
git diff --check
git status --short
```

Expected: no whitespace errors and a clean worktree.
