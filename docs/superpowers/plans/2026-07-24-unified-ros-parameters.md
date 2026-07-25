# Unified ROS Parameters Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace all externally visible camelCase ROS2 parameters with the grouped snake_case schema defined in `docs/superpowers/specs/2026-07-24-unified-ros-parameters-design.md`.

**Architecture:** Mapping and relocalization share canonical topic, frame, lidar, IMU, calibration, gravity, and loop-closure names. Mapping-only and localization-only parameters remain in focused groups. A small calibration parser validates vector sizes and exposes the direct lidar-to-IMU transform plus the transposed rotation required by mapping.

**Tech Stack:** C++17, ROS2 Humble parameters and launch, YAML, Eigen, GoogleTest, CMake/CTest, colcon.

---

## File Structure

- Create `include/common/lidar_imu_calibration.h` — validate and convert the canonical translation/rotation/RPY vectors.
- Create `test/test_lidar_imu_calibration.cpp` — verify non-identity direction handling and invalid dimensions.
- Create `test/test_unified_parameter_schema.cpp` — enforce canonical external parameter strings and reject legacy names.
- Modify `include/lio_sam/utility.hpp` — mapping-side canonical parameter declarations and calibration conversion.
- Modify `src/imageProjection.cpp` — canonical segmentation-only and non-ground topic reads.
- Modify `src/slam/system.cpp` — relocalization-side canonical shared parameter reads and calibration conversion.
- Modify `config/mapping.yaml` — grouped mapping configuration.
- Modify `config/relocalization.yaml` — grouped relocalization configuration while preserving `sc_top_n: 6`.
- Modify `launch/localization_with_nonground.launch.py` — canonical dotted override.
- Modify `test/test_ground_patch_parameter_surface.cpp` and `test/test_auto_relocalization_parameter_surface.cpp` — update existing schema expectations.
- Modify `CMakeLists.txt` — register the new tests.

### Task 1: Add calibration conversion with direction tests

**Files:**
- Create: `include/common/lidar_imu_calibration.h`
- Create: `test/test_lidar_imu_calibration.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing calibration tests**

Create tests that pass a non-symmetric 90-degree Z rotation and translation
`[0.0, 0.0, 0.10]`, then require:

```cpp
const auto calibration =
    ParseLidarImuCalibration(translation, rotation, rpy);
EXPECT_TRUE(calibration.lidar_to_imu.block<3, 3>(0, 0).isApprox(expected));
EXPECT_TRUE(calibration.mapping_imu_to_lidar_rotation.isApprox(
    expected.transpose()));
EXPECT_TRUE(calibration.lidar_to_imu.block<3, 1>(0, 3).isApprox(
    Eigen::Vector3d(0.0, 0.0, 0.10)));
```

Add separate tests requiring `std::invalid_argument` for translation lengths
other than 3 and matrix lengths other than 9.

- [ ] **Step 2: Register and run the test to verify RED**

Add:

```cmake
ament_add_gtest(test_lidar_imu_calibration
  test/test_lidar_imu_calibration.cpp)
target_include_directories(test_lidar_imu_calibration PRIVATE include)
```

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target test_lidar_imu_calibration -j2
```

Expected: compilation fails because `common/lidar_imu_calibration.h` does not
exist.

- [ ] **Step 3: Implement the parser**

Define:

```cpp
struct LidarImuCalibration {
  Eigen::Matrix4d lidar_to_imu = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d mapping_imu_to_lidar_rotation =
      Eigen::Matrix3d::Identity();
  Eigen::Vector3d mapping_imu_to_lidar_lever_arm =
      Eigen::Vector3d::Zero();
  Eigen::Matrix3d mapping_rpy_rotation = Eigen::Matrix3d::Identity();
};

LidarImuCalibration ParseLidarImuCalibration(
    const std::vector<double>& translation,
    const std::vector<double>& rotation,
    const std::vector<double>& rpy);
```

Validate `3/9/9` vector lengths, map both matrices row-major, place the direct
rotation and translation into `lidar_to_imu`, transpose only the mapping
acceleration/gyro rotation, derive
`mapping_imu_to_lidar_lever_arm = R^{lidar}_{imu} * t^{imu}_{lidar}`, and
leave the RPY matrix untransposed.

- [ ] **Step 4: Verify GREEN**

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target test_lidar_imu_calibration -j2
/home/robot/slam_ws/build/lio_sam_hesai/test_lidar_imu_calibration
```

Expected: all calibration tests pass.

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt include/common/lidar_imu_calibration.h
git add -f test/test_lidar_imu_calibration.cpp
git commit -m "test(config): define lidar imu calibration contract"
```

### Task 2: Add the canonical schema regression test

**Files:**
- Create: `test/test_unified_parameter_schema.cpp`
- Modify: `CMakeLists.txt`
- Modify: `test/test_ground_patch_parameter_surface.cpp`
- Modify: `test/test_auto_relocalization_parameter_surface.cpp`

- [ ] **Step 1: Write a failing project-surface test**

The test reads:

```cpp
const std::array<std::string, 7> parameter_files = {
    "include/lio_sam/utility.hpp",
    "src/imageProjection.cpp",
    "src/slam/system.cpp",
    "config/mapping.yaml",
    "config/relocalization.yaml",
    "launch/mapping.launch.py",
    "launch/localization_with_nonground.launch.py",
};
```

Require representative canonical strings:

```cpp
"sensor_topic.lidar_topic"
"lidar.sensor_type"
"imu.gravity"
"calibration.translation"
"map_save.generate_scan_context_db"
"ground_segmentation.grid_patch.max_slope_deg"
"loop_closure.enable"
```

Reject every legacy external name listed in the design, including:

```cpp
"pointCloudTopic"
"mappingMode"
"groundGridPatchEnable"
"imuGravity"
"extrinsicTrans"
"loopClosureEnableFlag"
"system.enable_loopclosure"
```

Search quoted C++ parameter strings and YAML keys, not internal C++ member
identifiers, because internal member renaming is out of scope.

- [ ] **Step 2: Update existing surface expectations**

Change the ground-patch test to require the ten
`ground_segmentation.grid_patch.*` names. Change the mapping export test to
require `map_save.generate_scan_context_db`,
`map_save.align_ground_to_zero`, and `map_save.ground_z_offset` while retaining
the existing map-export behavior tokens.

- [ ] **Step 3: Register and verify RED**

Add:

```cmake
ament_add_gtest(test_unified_parameter_schema
  test/test_unified_parameter_schema.cpp)
```

Build and run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target test_unified_parameter_schema -j2
/home/robot/slam_ws/build/lio_sam_hesai/test_unified_parameter_schema
```

Expected: failures report the current legacy names and missing canonical names.

### Task 3: Migrate mapping parameter readers

**Files:**
- Modify: `include/lio_sam/utility.hpp`
- Modify: `src/imageProjection.cpp`
- Modify: `launch/localization_with_nonground.launch.py`

- [ ] **Step 1: Replace topic, frame, GPS, system, lidar, and IMU declarations**

Remove the duplicate legacy-first declarations in `ParamServer` and declare
only canonical names. Representative reads become:

```cpp
declare_parameter("sensor_topic.lidar_topic", "points");
get_parameter("sensor_topic.lidar_topic", pointCloudTopic);
declare_parameter("frames.odometry", "odom");
get_parameter("frames.odometry", odometryFrame);
declare_parameter("lidar.sensor_type", "ouster");
get_parameter("lidar.sensor_type", sensorStr);
declare_parameter("imu.gravity", 9.80511);
get_parameter("imu.gravity", imuGravity);
```

Use all exact names from the approved design and preserve the current mapping
defaults.

- [ ] **Step 2: Replace calibration reads**

Read:

```cpp
calibration.translation
calibration.rotation
calibration.rpy
```

with defaults `[0.0, 0.0, 0.10]`, identity, and identity. Parse through
`ParseLidarImuCalibration`; assign:

```cpp
extTrans = parsed.mapping_imu_to_lidar_lever_arm;
extRot = parsed.mapping_imu_to_lidar_rotation;
extRPY = parsed.mapping_rpy_rotation;
extQRPY = Eigen::Quaterniond(extRPY);
```

This binds `extTrans` to the IMU-to-lidar lever arm expressed in lidar axes,
rather than the canonical direct `lidar_to_imu` translation.

Catch `std::invalid_argument`, log a fatal parameter error, and rethrow so
startup fails clearly.

- [ ] **Step 3: Replace mapping-only declarations**

Retarget map saving, ground segmentation/grid patch, feature extraction, voxel
filters, dynamic/static filters, motion constraints, local-map construction,
loop closure, and visualization to their canonical names. Preserve all current
defaults and internal member variables.

- [ ] **Step 4: Update ImageProjection and launch override**

Replace:

```cpp
segmentation_only_mode
nonGroundCloudTopic
```

with:

```cpp
ground_segmentation.only_mode
output_topic.non_ground_cloud_topic
output_topic.mapping_odom_topic
```

Also rename `output.mapping_odom_topic` to
`output_topic.mapping_odom_topic`.

Change the launch override dictionary key to
`"ground_segmentation.only_mode": True`.

- [ ] **Step 5: Run the mapping side of the schema test**

Run:

```bash
/home/robot/slam_ws/build/lio_sam_hesai/test_unified_parameter_schema \
  --gtest_filter=UnifiedParameterSchema.MappingReadersUseCanonicalNames
```

Expected: mapping-reader assertions pass; YAML assertions remain pending until
Task 5.

### Task 4: Migrate relocalization parameter readers

**Files:**
- Modify: `src/slam/system.cpp`

- [ ] **Step 1: Retarget shared names**

Change the relevant `util::param` calls to:

```cpp
lidar.sensor_type
lidar.point_jump_span
lidar.scan_lines
lidar.lower_angle_deg
lidar.horizon_scan
lidar.vertical_resolution_deg
lidar.min_range
lidar.max_range
lidar.point_time_scale
lidar.rotation_noise_std
lidar.position_noise_std
imu.acc_bias_rw_noise_std
imu.gyro_bias_rw_noise_std
imu.gravity
loop_closure.enable
visualization.enable_global_map
visualization.global_map_resolution
loop_closure.*
```

Keep the already canonical `localization.*`, `frontend.*`, `output.*`,
`system.keyframe_delta_*`, and `system.tile_map_grid_size` names unchanged.

- [ ] **Step 2: Replace 4x4 calibration parameter loading**

Read the same three canonical calibration vectors as mapping, using translation
`[0.0, 0.0, 0.10]` and identity matrix defaults. Parse them and assign:

```cpp
config.calibration_lidar_to_imu_ = parsed.lidar_to_imu;
```

Do not invert or transpose this transform in `System`.

- [ ] **Step 3: Run the relocalization reader schema test**

Run:

```bash
/home/robot/slam_ws/build/lio_sam_hesai/test_unified_parameter_schema \
  --gtest_filter=UnifiedParameterSchema.RelocalizationReaderUsesCanonicalNames
```

Expected: relocalization reader assertions pass.

### Task 5: Rewrite both YAML configurations

**Files:**
- Modify: `config/mapping.yaml`
- Modify: `config/relocalization.yaml`

- [ ] **Step 1: Rewrite mapping.yaml**

Move every value into the approved nested group and retain its current numeric,
boolean, string, and array value. Use:

```yaml
calibration:
  translation: [0.0, 0.0, 0.10]
  rotation: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
  rpy: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
```

and preserve all comments next to their migrated values.

- [ ] **Step 2: Rewrite relocalization.yaml**

Retain `localization.sc_top_n: 6`, all existing relocalization thresholds, and
the user's reordered ground-height block. Add all canonical common and
ground-segmentation names needed by the relocalization and segmentation nodes.
Use the same calibration arrays as mapping.

- [ ] **Step 3: Verify YAML and schema tests**

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

Expected: valid YAML and all three schema tests pass.

- [ ] **Step 4: Commit the migration**

Stage only the files named in Tasks 2–5, including the user's existing
`sc_top_n: 6` and `publish_static_tf: false` changes now integrated into the
canonical configs/launch. Commit:

```bash
git commit -m "refactor(config): unify ROS parameter schema"
```

### Task 6: Build and runtime-smoke both pipelines

**Files:**
- No source changes expected.

- [ ] **Step 1: Build the package with tests**

```bash
cd /home/robot/slam_ws
colcon build --symlink-install --packages-select lio_sam_hesai \
  --cmake-args -DBUILD_TESTING=ON
```

Expected: `Summary: 1 package finished`.

- [ ] **Step 2: Run relevant tests**

```bash
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  --output-on-failure \
  -R '^(test_lidar_imu_calibration|test_unified_parameter_schema|test_ground_patch_parameter_surface|test_auto_relocalization_parameter_surface|test_relocalization_efficiency_surface|test_relocalization_acceptance_policy|test_ground_map_source|test_scan_context)$'
```

Expected: all selected tests pass.

- [ ] **Step 3: Run project-wide legacy string audit**

Search parameter declaration/read strings and YAML keys for every legacy name
listed in the design. Matches are allowed only in migration documentation and
negative-test fixtures; production/config/launch matches fail the task.

- [ ] **Step 4: Launch-smoke mapping**

Launch mapping without RViz for a bounded interval and inspect output for
`ParameterAlreadyDeclaredException`, `InvalidParameterTypeException`,
undeclared parameters, or calibration validation failures.

- [ ] **Step 5: Launch-smoke relocalization**

Launch `localization_with_nonground.launch.py` with RViz and static TF disabled.
Confirm the relocalization node and segmentation node start with canonical
parameters, the Scan Context DB loads, and no legacy-key defaults are used.

- [ ] **Step 6: Inspect exposed parameters**

Use `ros2 param list` for the mapping, relocalization, and ground-segmentation
nodes. Confirm project parameters are grouped and snake_case; ROS built-ins
such as `use_sim_time` are exempt.

- [ ] **Step 7: Final commit if verification required corrections**

If verification required source corrections, rerun Steps 1–6 and commit only
those verified corrections with:

```bash
git commit -m "fix(config): complete canonical parameter migration"
```
