# P2.2 Mapping/Localization Coordinate and Naming Unification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Unify mapping and localization coordinate semantics, transform types, configuration ownership, topic classification, and map artifact paths without changing either mode's TF topology or runtime algorithm behavior.

**Architecture:** Add a shared contract library containing compile-time directed transforms, immutable configuration, legacy-parameter resolution, ROS pose conversion, and map artifact layout. Migrate localization and mapping at module boundaries while retaining optimizer-native matrices, existing topic strings, disk filenames, and TF publishers.

**Tech Stack:** C++17, ROS 2 Humble `rclcpp`/tf2, Eigen, PCL, yaml-cpp, GTest, Python launch files, rosbag2.

---

## Preconditions

- Execute in a worktree created with `superpowers:using-git-worktrees`.
- Preserve the user's current uncommitted P2.1 follow-up edits in a commit or approved patch before creating the worktree.
- Read `docs/superpowers/specs/2026-09-10-mapping-localization-coordinate-naming-unification-design.md`.
- Build serially with `-j1`.

### Task 1: Freeze the Existing Runtime Contract

**Files:**
- Create: `test/test_coordinate_contract_surface.cpp`
- Modify: `CMakeLists.txt`

- [x] **Step 1: Write source-level topology tests**

Reuse `ReadProjectFile()` from `test/test_unified_parameter_schema.cpp` and add:

```cpp
TEST(CoordinateContractSurface, MappingKeepsMapOdomBaseTopology) {
  const auto launch = ReadProjectFile("launch/mapping.launch.py");
  const auto fusion = ReadProjectFile("src/imuPreintegration.cpp");
  EXPECT_NE(launch.find("\"map\", \"odom\""), std::string::npos);
  EXPECT_NE(fusion.find("ts.child_frame_id = baselinkFrame"),
            std::string::npos);
  EXPECT_NE(fusion.find("tfBroadcaster->sendTransform(ts)"),
            std::string::npos);
}

TEST(CoordinateContractSurface, LocalizationHasNoOdomTfDependency) {
  const auto system = ReadProjectFile("src/slam/system.cpp");
  EXPECT_EQ(system.find("frames.odometry"), std::string::npos);
  EXPECT_EQ(system.find("LookupExternalOdom"), std::string::npos);
  EXPECT_NE(system.find("map_frame"), std::string::npos);
  EXPECT_NE(system.find("base_frame"), std::string::npos);
}
```

Assert these current literals: mapping `lio_sam/mapping/odometry`, `lio_sam/mapping/path`, `lio_sam/mapping/map_global`, `lio_sam/mapping/map_local`, `lio_sam/mapping/cloud_registered`; localization `lio_sam/localization/odometry`, `lio_sam/localization/path`, `lio_sam/localization/global_map`, `lio_sam/localization/local_map`, `lio_sam/localization/current_lidar`; and the three launch files' static translations `[0.02, 0, 0.32]` and `[0.02, 0, 0.22]`.

- [x] **Step 2: Register and run the baseline**

Add:

```cmake
ament_add_gtest(test_coordinate_contract_surface
  test/test_coordinate_contract_surface.cpp)
```

Run: `cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target test_coordinate_contract_surface test_lidar_imu_calibration -- -j1`

Run: `ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai -R '^(test_coordinate_contract_surface|test_lidar_imu_calibration)$' --output-on-failure`

Expected: both pass before production edits.

- [x] **Step 3: Commit**

```bash
git add CMakeLists.txt test/test_coordinate_contract_surface.cpp
git commit -m "test(contract): freeze mapping and localization coordinates"
```

### Task 2: Add Direction-Safe Transform Types

**Files:**
- Create: `include/common/rigid_transform.h`
- Create: `test/test_rigid_transform.cpp`
- Modify: `CMakeLists.txt`

- [x] **Step 1: Write failing composition and validation tests**

```cpp
#include "common/rigid_transform.h"
#include <gtest/gtest.h>
#include <type_traits>

TEST(RigidTransform, CompositionPreservesFrameTypes) {
  const BaseTLidar base_T_lidar = BaseTLidar::Identity();
  const LidarTImu lidar_T_imu = LidarTImu::Identity();
  const BaseTImu base_T_imu = base_T_lidar * lidar_T_imu;
  static_assert(std::is_same_v<decltype(base_T_imu), const BaseTImu>);
  EXPECT_TRUE(base_T_imu.Matrix().isApprox(Eigen::Matrix4d::Identity()));
}

TEST(RigidTransform, InverseSwapsFrameTypes) {
  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
  value.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  const LidarTBase lidar_T_base = BaseTLidar(value).Inverse();
  EXPECT_TRUE((lidar_T_base * lidar_T_base.Inverse()).Matrix().isApprox(
      Eigen::Matrix4d::Identity()));
}

TEST(RigidTransform, RejectsInvalidHomogeneousMatrix) {
  Eigen::Matrix4d invalid = Eigen::Matrix4d::Identity();
  invalid(3, 0) = 1.0;
  EXPECT_THROW(MapTLidar::FromMatrix(invalid), std::invalid_argument);
}
```

- [x] **Step 2: Run RED**

Register `test_rigid_transform`, build it, and verify compilation fails because the header/types do not exist.

- [x] **Step 3: Implement the header-only type**

Define empty tags `MapFrame`, `OdomFrame`, `BaseFrame`, `ImuFrame`, `LidarFrame`, and `LocalIntegrationFrame`. Implement:

```cpp
template <typename Parent, typename Child>
class RigidTransform3d {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RigidTransform3d() : value_(Eigen::Isometry3d::Identity()) {}
  explicit RigidTransform3d(const Eigen::Isometry3d& value) : value_(value) {
    Validate(value_.matrix());
  }
  static RigidTransform3d Identity() { return {}; }
  static RigidTransform3d FromMatrix(const Eigen::Matrix4d& matrix);
  template <typename Descendant>
  RigidTransform3d<Parent, Descendant> operator*(
      const RigidTransform3d<Child, Descendant>& rhs) const {
    return RigidTransform3d<Parent, Descendant>::FromMatrix(
        value_.matrix() * rhs.Matrix());
  }
  RigidTransform3d<Child, Parent> Inverse() const {
    return RigidTransform3d<Child, Parent>(value_.inverse());
  }
  const Eigen::Matrix4d& Matrix() const { return value_.matrix(); }
  Eigen::Matrix3d Rotation() const { return value_.rotation(); }
  Eigen::Vector3d Translation() const { return value_.translation(); }
 private:
  static void Validate(const Eigen::Matrix4d& matrix);
  Eigen::Isometry3d value_;
};
```

`Validate` rejects non-finite values, a non-homogeneous last row, a non-orthogonal rotation, or determinant not near `+1`. Add aliases `MapTLidar`, `MapTBase`, `OdomTLidar`, `OdomTBase`, `BaseTLidar`, `LidarTBase`, `ImuTLidar`, `LidarTImu`, `BaseTImu`, `ImuTBase`, and `LocalTImu`.

- [x] **Step 4: Run GREEN and commit**

Run: `cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target test_rigid_transform -- -j1`

Run: `ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai -R '^test_rigid_transform$' --output-on-failure`

Expected: pass.

```bash
git add CMakeLists.txt include/common/rigid_transform.h test/test_rigid_transform.cpp
git commit -m "feat(contract): add direction-safe rigid transforms"
```

### Task 3: Centralize ROS Pose and TF Conversion

**Files:**
- Create: `include/common/ros_pose_adapter.h`
- Create: `src/common/ros_pose_adapter.cpp`
- Create: `test/test_ros_pose_adapter.cpp`
- Modify: `include/common/ros_utility.h`
- Modify: `CMakeLists.txt`

- [x] **Step 1: Write failing adapter tests**

Test `MapTBase` conversion, frame IDs, initial-pose validation, and invalid quaternion rejection:

```cpp
FrameIds frames{"map", "odom", "base_link", "imu_link", "hesai_lidar"};
const auto tf = RosPoseAdapter::ToTransformStamped(
    MapTBase::Identity(), frames.map, frames.base_link, rclcpp::Time(123));
EXPECT_EQ(tf.header.frame_id, "map");
EXPECT_EQ(tf.child_frame_id, "base_link");
EXPECT_DOUBLE_EQ(tf.transform.rotation.w, 1.0);

std::string error;
EXPECT_TRUE(RosPoseAdapter::ParseMapTBase(message, "map", &error));
message.header.frame_id = "odom";
EXPECT_FALSE(RosPoseAdapter::ParseMapTBase(message, "map", &error));
```

Also test NaN position, zero quaternion, quaternion normalization, and `Odometry.header.frame_id`/`child_frame_id`.

- [x] **Step 2: Run RED**

Register the test with `rclcpp`, `geometry_msgs`, `nav_msgs`, and `tf2_geometry_msgs`; verify missing adapter symbols.

- [x] **Step 3: Implement the adapter API**

```cpp
struct FrameIds {
  std::string map{"map"};
  std::string odometry{"odom"};
  std::string base_link{"base_link"};
  std::string imu{"imu_link"};
  std::string lidar{"hesai_lidar"};
};

class RosPoseAdapter {
 public:
  template <typename Parent, typename Child>
  static geometry_msgs::msg::Pose ToPose(
      const RigidTransform3d<Parent, Child>& transform);
  template <typename Parent, typename Child>
  static geometry_msgs::msg::TransformStamped ToTransformStamped(
      const RigidTransform3d<Parent, Child>& transform,
      const std::string& parent, const std::string& child,
      const rclcpp::Time& stamp);
  template <typename Parent, typename Child>
  static nav_msgs::msg::Odometry ToOdometry(
      const RigidTransform3d<Parent, Child>& transform,
      const std::string& parent, const std::string& child,
      const rclcpp::Time& stamp);
  static std::optional<MapTBase> ParseMapTBase(
      const geometry_msgs::msg::PoseWithCovarianceStamped& message,
      const std::string& map_frame, std::string* error);
};
```

Keep templates in the header. Normalize valid quaternions. Accept an empty initial-pose frame as `map` for ROS compatibility; reject any other frame. Keep `eigen2Transform` temporarily in Task 3 so the tree builds; remove it in Task 5 after the final caller migrates.

Create `${PROJECT_NAME}_contracts` as a static library containing `src/common/ros_pose_adapter.cpp`, link it to all four mapping executables and `${PROJECT_NAME}_relocalization_lib`, and append the Task 4 and Task 7 common sources to this target when introduced.

- [x] **Step 4: Run GREEN and commit**

Run: `cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target test_ros_pose_adapter -- -j1`

Run: `ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai -R '^test_ros_pose_adapter$' --output-on-failure`

```bash
git add CMakeLists.txt include/common/ros_pose_adapter.h src/common/ros_pose_adapter.cpp include/common/ros_utility.h test/test_ros_pose_adapter.cpp
git commit -m "refactor(ros): centralize pose and TF conversion"
```

### Task 4: Add Immutable Configuration and Alias Resolution

**Files:**
- Create: `include/common/parameter_alias.h`
- Create: `include/common/slam_config.h`
- Create: `src/common/slam_config.cpp`
- Create: `test/test_parameter_alias.cpp`
- Create: `test/test_slam_config.cpp`
- Modify: `CMakeLists.txt`

- [x] **Step 1: Write failing pure alias tests**

```cpp
TEST(ParameterAlias, UsesCanonical) {
  EXPECT_EQ(ResolveParameterAlias<int>(3, std::nullopt, 1).value, 3);
}
TEST(ParameterAlias, UsesLegacyAndMarksIt) {
  const auto result = ResolveParameterAlias<int>(std::nullopt, 4, 1);
  EXPECT_EQ(result.value, 4);
  EXPECT_TRUE(result.used_legacy);
}
TEST(ParameterAlias, RejectsConflict) {
  EXPECT_THROW(ResolveParameterAlias<int>(3, 4, 1), ParameterAliasConflict);
}
```

Use `std::optional<T>` inputs. Vector equality is element-wise exact.

- [x] **Step 2: Define the configuration hierarchy**

```cpp
enum class SlamMode { kMapping, kLocalization };
struct SensorTopics { std::string lidar, imu, gps; };
struct PipelineTopics {
  std::string cloud_info, feature_cloud_info, mapping_odometry;
  std::string imu_odometry_incremental, loop_closure_hint;
};
struct CommandTopics { std::string initial_pose; };
struct OutputTopics {
  std::string odometry, path, global_map, local_map;
  std::string registered_cloud, registered_cloud_raw, ground_map;
  std::string trajectory, diagnostics;
};
struct CalibrationConfig {
  ImuTLidar imu_T_lidar{ImuTLidar::Identity()};
  Eigen::Matrix3d mapping_imu_to_lidar_rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d mapping_imu_to_lidar_lever_arm{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rpy_rotation{Eigen::Matrix3d::Identity()};
};
struct SlamConfig {
  SlamMode mode;
  FrameIds frames;
  SensorTopics sensors;
  PipelineTopics pipeline_topics;
  CommandTopics command_topics;
  OutputTopics outputs;
  CalibrationConfig calibration;
  MappingConfig mapping;
  LocalizationConfig localization;
};
SlamConfig LoadSlamConfig(rclcpp::Node& node, SlamMode mode);
```

Move current `ParamServer` fields into nested `MappingConfig` groups matching their ROS prefixes. Move current `ConfigParameters` fields into nested `LocalizationConfig` groups. Use snake_case leaf names without repeating the group name.

- [x] **Step 3: Implement parameter declaration, aliases, and validation**

Implement:

```cpp
template <typename T>
T DeclareRead(rclcpp::Node& node, const std::string& canonical,
              const std::optional<std::string>& legacy,
              const T& default_value, LegacyWarningSet& warnings);
```

Inspect `get_parameter_overrides()` to distinguish absent values from defaults. Canonical wins; legacy-only works and warns once per node; equal dual values warn; unequal dual values throw with both keys and values. Reject empty required frames/topics, non-positive sizes/rates, non-finite values, and invalid calibration.

Use this exact migration table:

| Canonical key | Legacy/source | Mapping default | Localization default |
|---|---|---|---|
| `output_topic.odometry` | `sensor_topic.odom_topic` / `output_topic.mapping_odom_topic` | `odometry/imu` | `lio_sam/localization/odometry` |
| `output_topic.path` | hardcoded | `lio_sam/mapping/path` | `lio_sam/localization/path` |
| `pipeline_topic.mapping_odometry` | hardcoded | `lio_sam/mapping/odometry` | empty |
| `pipeline_topic.imu_odometry_incremental` | legacy odom prefix plus suffix | `odometry/imu_incremental` | empty |
| `command_topic.initial_pose` | hardcoded | empty | `/initialpose` |
| `output_topic.diagnostics` | hardcoded | `/diagnostics` | `/diagnostics` |

Add these remaining exact defaults. Empty mode-inapplicable topics are valid and must not create publishers.

| Field | Default |
|---|---|
| `pipeline_topic.deskewed_cloud` | `lio_sam/deskew/cloud_deskewed` |
| `pipeline_topic.ground_cloud` | `lio_sam/deskew/ground_cloud_global` |
| `pipeline_topic.cloud_info` | `lio_sam/deskew/cloud_info` |
| `pipeline_topic.feature_cloud_info` | `lio_sam/feature/cloud_info` |
| `pipeline_topic.corner_cloud` | `lio_sam/feature/cloud_corner` |
| `pipeline_topic.surface_cloud` | `lio_sam/feature/cloud_surface` |
| `pipeline_topic.mapping_odometry_incremental` | `lio_sam/mapping/odometry_incremental` |
| `pipeline_topic.loop_closure_hint` | `lio_loop/loop_closure_detection` |
| `output_topic.imu_path` | `lio_sam/imu/path` |
| `output_topic.trajectory` | `lio_sam/mapping/trajectory` |
| `output_topic.global_map` | mapping: `lio_sam/mapping/map_global`; localization: `lio_sam/localization/global_map` |
| `output_topic.ground_map` | `lio_sam/mapping/ground_cloud_global` |
| `output_topic.local_map` | mapping: `lio_sam/mapping/map_local`; localization: `lio_sam/localization/local_map` |
| `output_topic.registered_cloud` | `lio_sam/mapping/cloud_registered` |
| `output_topic.registered_cloud_raw` | `lio_sam/mapping/cloud_registered_raw` |
| `output_topic.current_lidar` | `lio_sam/localization/current_lidar` |
| `output_topic.loop_history` | `lio_sam/mapping/icp_loop_closure_history_cloud` |
| `output_topic.loop_constraints` | `/lio_sam/mapping/loop_closure_constraints` |

- [x] **Step 4: Test the ROS loader**

Instantiate `rclcpp::Node` with `NodeOptions().parameter_overrides(...)`. Test both modes, canonical override, legacy-only, equal dual, conflicting dual, empty frame, and bad calibration matrix.

Run: `cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target test_parameter_alias test_slam_config -- -j1`

Run: `ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai -R '^(test_parameter_alias|test_slam_config)$' --output-on-failure`

- [x] **Step 5: Commit**

```bash
git add CMakeLists.txt include/common/parameter_alias.h include/common/slam_config.h src/common/slam_config.cpp test/test_parameter_alias.cpp test/test_slam_config.cpp
git commit -m "feat(config): add shared immutable SLAM configuration"
```

### Task 5: Migrate Relocalization Boundaries

**Files:**
- Modify: `include/common/lidar_imu_calibration.h`
- Modify: `include/common/sensor_data_type.h`
- Modify: `include/slam/system.h`
- Modify: `include/slam/localization.h`
- Modify: `include/slam/preprocessing.h`
- Modify: `src/slam/system.cpp`
- Modify: `src/slam/localization.cpp`
- Modify: `src/slam/preprocessing.cpp`
- Delete: `include/slam/config_parameters.h`
- Delete: `src/slam/config_parameters.cpp`
- Modify: `test/test_tf_continuity_controller.cpp`
- Modify: `test/test_auto_relocalization_parameter_surface.cpp`
- Modify: `test/test_coordinate_contract_surface.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Turn semantic-type tests RED**

Change fixtures to construct `LocalizationResult::map_T_lidar` as `MapTLidar`, `LocalizationResult::local_T_imu` as `LocalTImu`, and `TfPoseSample::map_T_base` as `MapTBase`. Add source assertions that `ConfigParameters::Instance()`, `last_odom_state_`, and `frames.odometry` are absent from localization production files. Run the three affected tests and expect compile/source failures.

- [ ] **Step 2: Make calibration direction explicit**

Replace raw calibration pose storage with:

```cpp
ImuTLidar imu_T_lidar{ImuTLidar::Identity()};
LidarTImu lidar_T_imu{LidarTImu::Identity()};
```

Build `imu_T_lidar` from the validated canonical matrix and derive the inverse. Preserve the existing mapping rotation, lever arm, and RPY numerical calculations.

- [ ] **Step 3: Inject immutable configuration**

`System` owns `std::shared_ptr<const SlamConfig> config_`, created once by `LoadSlamConfig(*this, SlamMode::kLocalization)`. Use these constructors:

```cpp
Localization(System*, std::shared_ptr<const SlamConfig>);
PreProcessing(System*, std::shared_ptr<const SlamConfig>);
```

Replace every singleton read with a nested immutable config read. Do not add global accessors or setters.

- [ ] **Step 4: Apply exact pose-state renames**

| Old | New |
|---|---|
| `LocalizationResult::map_pose` | `MapTLidar map_T_lidar` |
| `LocalizationResult::odom_pose` | `LocalTImu local_T_imu` |
| `MapLidarPrediction::map_lidar_pose` | `MapTLidar map_T_lidar` |
| `TfPoseSample::map_base_pose` | `MapTBase map_T_base` |
| `last_odom_state_` | `last_local_state_` |
| `odom_pre_integration_ptr_` | `local_pre_integration_ptr_` |
| `IntegrateImuMeasuresOdom` | `IntegrateImuMeasuresLocal` |
| `T_base_to_lidar_` | `LidarTBase lidar_T_base_` |
| `GetBaseToLidarTransform` | `GetLidarTBase` |

Use `.Matrix()` only at Eigen/g2o/PCL algorithm boundaries.

- [ ] **Step 5: Preserve the initial-pose and result equations**

Implement the typed equations:

```cpp
const MapTLidar map_T_lidar = map_T_base * lidar_T_base.Inverse();
const MapTBase map_T_base = result.map_T_lidar * lidar_T_base;
```

Use `RosPoseAdapter` for TF, odometry, and initial-pose parsing. Preserve covariance, ground-height adjustment, measured/extrapolated/held TF selection, timestamps, queues, and rates. Never add an odom lookup or publisher.

- [ ] **Step 6: Delete the singleton and verify**

Remove its source from CMake, delete both files, and run:

`cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target lio_sam_hesai_relocalization test_tf_continuity_controller test_auto_relocalization_parameter_surface test_coordinate_contract_surface -- -j1`

`ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai -R '^(test_tf_continuity_controller|test_auto_relocalization_parameter_surface|test_coordinate_contract_surface)$' --output-on-failure`

Expected: pass; `rg 'ConfigParameters::Instance' include src` returns no matches.

- [ ] **Step 7: Commit**

```bash
git add CMakeLists.txt include/common/lidar_imu_calibration.h include/common/sensor_data_type.h include/slam/system.h include/slam/localization.h include/slam/preprocessing.h src/slam/system.cpp src/slam/localization.cpp src/slam/preprocessing.cpp test/test_tf_continuity_controller.cpp test/test_auto_relocalization_parameter_surface.cpp test/test_coordinate_contract_surface.cpp
git rm include/slam/config_parameters.h src/slam/config_parameters.cpp
git commit -m "refactor(localization): enforce explicit coordinate contracts"
```

### Task 6: Migrate Mapping Configuration, Topics, and Pose Boundaries

**Files:**
- Modify: `include/lio_sam/utility.hpp`
- Modify: `src/imageProjection.cpp`
- Modify: `src/featureExtraction.cpp`
- Modify: `src/imuPreintegration.cpp`
- Modify: `src/mapOptimization.cpp`
- Modify: `test/test_unified_parameter_schema.cpp`
- Modify: `test/test_coordinate_contract_surface.cpp`

- [ ] **Step 1: Turn hardcoded-topic tests RED**

Change Task 1 assertions to require `pipeline_topic.*`, `command_topic.*`, and `output_topic.*` in YAML and to reject hardcoded `lio_sam/...` topic literals in production constructors. Run `test_coordinate_contract_surface`; expected: fail.

- [ ] **Step 2: Reduce ParamServer to a config-owning node base**

```cpp
class ParamServer : public rclcpp::Node {
 public:
  ParamServer(std::string name, const rclcpp::NodeOptions& options)
      : Node(std::move(name), options),
        config_(LoadSlamConfig(*this, SlamMode::kMapping)) {}
 protected:
  const SlamConfig config_;
};
```

Delete its flat fields and constructor parameter declarations. Replace accesses by existing ROS group: for example `pointCloudTopic` -> `config_.sensors.lidar`, `mapFrame` -> `config_.frames.map`, `extRot` -> `config_.calibration.mapping_imu_to_lidar_rotation`, and `historyKeyframeFitnessScore` -> `config_.mapping.loop_closure.fitness_threshold`.

- [ ] **Step 3: Parameterize all mapping topics**

Replace every publisher/subscriber literal with `config_.pipeline_topics`, `config_.command_topics`, or `config_.outputs`. Do not derive the incremental topic by string concatenation. Preserve topic values, QoS, callback groups, and node names.

Classify optimized lidar odometry consumed by `TransformFusion` as `pipeline_topics.mapping_odometry`; IMU delta as `pipeline_topics.imu_odometry_incremental`; fused existing `odomTopic` as `outputs.odometry`; user-visible maps/path/clouds as `outputs`.

- [ ] **Step 4: Type only the mapping output boundary**

Keep `transformTobeMapped`, GTSAM `Pose3`, `PointTypePose`, and internal `Eigen::Affine3f` unchanged. Convert the optimizer result to `OdomTLidar` at publication. In `TransformFusion`, name states `odom_T_lidar_at_correction`, `local_delta`, and `odom_T_lidar_now`, then compute:

```cpp
const OdomTBase odom_T_base = odom_T_lidar_now * lidar_T_base;
```

Publish the same `odom -> base_link` TF. Do not alter `map -> odom` or any numeric filter.

- [ ] **Step 5: Update schema tests**

Add compatibility mappings:

```cpp
{"sensor_topic.odom_topic", "output_topic.odometry", kMappingSurfaces},
{"output_topic.mapping_odom_topic", "output_topic.odometry",
 kRelocalizationSurfaces},
```

Legacy strings are allowed only in `slam_config.cpp` and compatibility tests. Require every canonical topic leaf in the applicable YAML and loader.

- [ ] **Step 6: Build and test**

Run: `cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target lio_sam_hesai_imageProjection lio_sam_hesai_featureExtraction lio_sam_hesai_imuPreintegration lio_sam_hesai_mapOptimization test_unified_parameter_schema test_coordinate_contract_surface -- -j1`

Run: `ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai -R '^(test_unified_parameter_schema|test_coordinate_contract_surface)$' --output-on-failure`

- [ ] **Step 7: Commit**

```bash
git add include/lio_sam/utility.hpp src/imageProjection.cpp src/featureExtraction.cpp src/imuPreintegration.cpp src/mapOptimization.cpp test/test_unified_parameter_schema.cpp test/test_coordinate_contract_surface.cpp
git commit -m "refactor(mapping): use shared configuration and pose semantics"
```

### Task 7: Centralize Map Artifact Paths

**Files:**
- Create: `include/common/map_artifact_layout.h`
- Create: `src/common/map_artifact_layout.cpp`
- Create: `test/test_map_artifact_layout.cpp`
- Modify: `include/common/constant_variable.h`
- Modify: `src/mapOptimization.cpp`
- Modify: `src/slam/localization.cpp`
- Modify: `src/localization_map/tile_map_generator.cpp`
- Modify: `test/test_tile_map_generator.cpp`
- Modify: `test/test_localization_stability_common.py`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write failing layout tests**

```cpp
const MapArtifactLayout layout("/maps/site");
EXPECT_EQ(layout.GlobalMap(), "/maps/site/GlobalMap.pcd");
EXPECT_EQ(layout.GroundMap(), "/maps/site/GroundMap.pcd");
EXPECT_EQ(layout.ScanContextDb(), "/maps/site/sc_db.bin");
EXPECT_EQ(layout.TiledManifest(), "/maps/site/tiled/manifest.yaml");
EXPECT_EQ(MapArtifactLayout::FromGlobalMap(
    "/maps/site/GlobalMap.pcd").Root(), "/maps/site");
EXPECT_EQ(MapArtifactLayout::FromTiledRoot(
    "/maps/site/tiled").Root(), "/maps/site");
```

- [ ] **Step 2: Implement the value object**

Store normalized `std::filesystem::path root_` plus `tile_subdirectory_`, defaulting to `tiled`. Expose `Root`, `GlobalMap`, `GroundMap`, `ScanContextDb`, `Trajectory`, `Transformations`, `CornerMap`, `SurfaceMap`, `TiledRoot`, and `TiledManifest`. `FromTiledRoot(path, expected_subdirectory)` validates the configured basename rather than forcing `tiled`. Reject empty paths and a non-`GlobalMap.pcd` global-map filename before I/O.

- [ ] **Step 3: Replace path literals**

Use the layout in map save service, shutdown save, localization global/ground/SC resolution, and tile generation. Keep all filenames and manifest content unchanged. Remove `kGlobalMapFileName = "map.pcd"`; the empty localization path resolves through `MapArtifactLayout(kDataPath).GlobalMap()`.

- [ ] **Step 4: Run P2.1 regression tests**

Run: `cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target test_map_artifact_layout test_tile_map_generator test_global_map_provider test_tiled_map_provider -- -j1`

Run: `ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai -R '^(test_map_artifact_layout|test_tile_map_generator|test_global_map_provider|test_tiled_map_provider|test_localization_stability_common)$' --output-on-failure`

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt include/common/map_artifact_layout.h src/common/map_artifact_layout.cpp include/common/constant_variable.h src/mapOptimization.cpp src/slam/localization.cpp src/localization_map/tile_map_generator.cpp test/test_map_artifact_layout.cpp test/test_tile_map_generator.cpp test/test_localization_stability_common.py
git commit -m "refactor(map): centralize artifact path conventions"
```

### Task 8: Update YAML, Launch, RViz, and Migration Documentation

**Files:**
- Modify: `config/mapping.yaml`
- Modify: `config/relocalization.yaml`
- Modify: `config/rviz2.rviz`
- Modify: `launch/mapping.launch.py`
- Modify: `launch/localization.launch.py`
- Modify: `launch/localization_with_nonground.launch.py`
- Create: `docs/P2_2_MIGRATION.md`
- Modify: `test/test_unified_parameter_schema.cpp`
- Modify: `test/test_coordinate_contract_surface.cpp`

- [ ] **Step 1: Write canonical topic groups into YAML**

Move values without changing strings. Mapping receives explicit `pipeline_topic.*` and `output_topic.*`; localization receives `command_topic.initial_pose` and every output used by `System`. Remove `sensor_topic.odom_topic` and `output_topic.mapping_odom_topic` from repository YAML so canonical paths are exercised by default.

- [ ] **Step 2: Normalize launch argument naming without changing behavior**

Use `use_sim_time` as the canonical name. Accept legacy `use_sim` for one cycle and map an explicit legacy value into the canonical value. Retain each launch's current effective default and all current `publish_static_tf` defaults, including the user's working-tree value in `localization_with_nonground.launch.py`. Do not change TF nodes, transforms, conditions, executables, or node names.

- [ ] **Step 3: Align RViz with configured publishers**

Update only stale topic entries. Every RViz topic must equal a canonical YAML value that still resolves to the same current ROS topic string. Keep fixed frames unchanged.

- [ ] **Step 4: Document the migration**

Create `docs/P2_2_MIGRATION.md` containing the full legacy-to-canonical key table, unchanged TF diagrams, `parent_T_child` convention, alias WARN/conflict examples, and this exact removal policy:

> Legacy parameter aliases remain enabled throughout the P2.2 release. They may be removed only by a separately reviewed compatibility-cleanup change after P2.3 acceptance and a deployed-configuration audit.

- [ ] **Step 5: Run syntax and contract tests**

Run: `python3 -m py_compile launch/mapping.launch.py launch/localization.launch.py launch/localization_with_nonground.launch.py`

Run: `cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target test_unified_parameter_schema test_lidar_imu_calibration test_coordinate_contract_surface -- -j1`

Run: `ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai -R '^(test_unified_parameter_schema|test_lidar_imu_calibration|test_coordinate_contract_surface)$' --output-on-failure`

Expected: all pass and current TF topology assertions remain green.

- [ ] **Step 6: Commit**

```bash
git add config/mapping.yaml config/relocalization.yaml config/rviz2.rviz launch/mapping.launch.py launch/localization.launch.py launch/localization_with_nonground.launch.py docs/P2_2_MIGRATION.md test/test_unified_parameter_schema.cpp test/test_coordinate_contract_surface.cpp
git commit -m "docs(config): publish P2.2 naming migration"
```

### Task 9: Full Verification and Handoff

**Files:**
- Modify: `docs/PROJECT_STATUS.md`

- [ ] **Step 1: Run static checks**

Run: `git diff --check HEAD~8`

Run: `rg -n 'ConfigParameters::Instance|T_[A-Za-z0-9_]+_to_[A-Za-z0-9_]+|last_odom_state_' include src`

Expected: diff check passes and the search returns no production matches. Names inside third-party APIs or migration-history tests do not count.

- [ ] **Step 2: Build and run the full non-license suite**

Run: `source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select lio_sam_hesai --cmake-args -DBUILD_TESTING=ON --executor sequential`

Run: `ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai -E '(copyright|cpplint)' --output-on-failure`

Expected: build and all functional/static tests pass. Copyright governance remains outside P2.2.

- [ ] **Step 3: Run localization full-map and tiled-map replay**

Run: `scripts/run_localization_stability.sh quick`

Run: `scripts/run_localization_stability.sh tiled-quick`

Run: `scripts/run_localization_stability.sh tiled-faults`

Require finite monotonic odometry, no new ERROR samples, no baseline regression, continuous `map -> base_link`, and no localization odom TF. Do not update a baseline to hide a regression.

- [ ] **Step 4: Run a mapping smoke replay**

Use the existing Hesai bag and verify:

```text
map -> odom remains a static identity
odom -> base_link remains dynamic
base_link -> imu and base_link -> lidar retain canonical extrinsics
all pre-P2.2 topic strings remain present
```

Record the bag identity, duration, commands, TF observations, and process exits.

- [ ] **Step 5: Update HANDOFF only after evidence passes**

Mark P2.2 complete, link the spec and this plan, and record exact build, CTest, replay, TF, and odometry results. If any required check fails, leave P2.2 incomplete and record the blocker instead.

- [ ] **Step 6: Commit handoff evidence**

```bash
git add docs/PROJECT_STATUS.md
git commit -m "docs(p2): record coordinate and naming verification"
```

- [ ] **Step 7: Inspect final scope**

Run: `git status --short`

Run: `git log -9 --oneline`

Run: `git diff HEAD~8 --stat`

Expected: only planned P2.2 files are included and the user's pre-existing edits are preserved.

---

## Progress: Tasks 1-4 complete (2026-09-10)

Branch `p2.2-coordinate-naming-unification`, branched from `main` after
preserving the user's P2.1 working-tree edits as commit `45df657`.

Commits:

- `a0fd37f` Task 1 - contract surface tests (`test_coordinate_contract_surface`)
- `f1ac095` Task 2 - `include/common/rigid_transform.h` + `test_rigid_transform`
- `3fd0919` Task 3 - `RosPoseAdapter` + `liblio_sam_hesai_contracts`
  (`test_ros_pose_adapter`)
- `4f30b44` Task 4 - `ParameterAlias` + `SlamConfig`
  (`test_parameter_alias`, `test_slam_config`)

Verification at this checkpoint:

- Full serial build (`make -j1`, every target): exit 0, no errors.
- Full non-license CTest run: 34/34 gtest pass.
- `uncrustify`, `lint_cmake`, `flake8`, `xmllint` FAIL, but they already fail on
  a clean `main` checkout (verified with a throwaway worktree running the lint
  tools directly). They flag ~200 untouched files. Task 9's expectation that
  `ctest -E '(copyright|cpplint)'` passes does not hold on `main` today either;
  resolve that separately before relying on Task 9 Step 2.

### Notes for Tasks 5-7

1. **Most vexing parse.** In `test_rigid_transform.cpp`, `EXPECT_THROW(
   LidarTBase(value), ...)` silently passed nothing: `LidarTBase(value);` parses
   as a declaration of a variable named `value`, so the validating constructor
   never ran. Use brace initialization (`LidarTBase{value}`) for any
   `EXPECT_THROW` on a single-argument constructor. Tasks 5-7 have similar
   assertions.

2. **Spec conflict, resolved in favour of this plan.** Design section 6 classifies
   `loop_closure_hint` under `command_topic.*`, but the Task 4 defaults table and
   the plan's own `PipelineTopics` struct both put it at
   `pipeline_topic.loop_closure_hint`. The plan was followed. Flip it if the
   design should win.

3. **`SlamConfig` is currently unconsumed.** Nothing links against it yet; Task 5
   (localization) and Task 6 (mapping) are what migrate production code onto it.
   `LoadSlamConfig` already declares `ground_segmentation.only_mode` and
   `output_topic.non_ground_cloud_topic` for mapping, so Task 6 must delete the
   duplicate `declare_parameter` calls in `src/imageProjection.cpp` (currently
   lines ~332-339) or the node will throw `ParameterAlreadyDeclaredException` at
   startup.

4. **`eigen2Transform` in `include/common/ros_utility.h`** is annotated as
   superseded and must be removed once its last caller migrates (Task 5).

Tasks 5-9 are not started. The branch is NOT merged to `main`; the user requires
explicit approval before merging.

---

## Progress: Task 5 complete (2026-09-10)

Commit `58b6b36` `refactor(localization): enforce explicit coordinate contracts`.
Task 5 Steps 1-6 all ran as written. `ConfigParameters` and both of its files are
gone; `rg 'ConfigParameters::Instance' include src` returns nothing.

Verification at this checkpoint:

- Full serial build (`make -j1`, every target): exit 0, no errors.
- Full non-license CTest run: 39/40 gtest pass.
- `test_unified_parameter_schema` FAILS and is expected to: see note 3 below.
- `uncrustify`, `lint_cmake`, `flake8`, `xmllint` still FAIL as on `main`.
- Task 9's `rg 'T_[A-Za-z0-9_]+_to_[A-Za-z0-9_]+'` now returns no matches.
- The three tests named in Step 6 pass; `test_coordinate_contract_surface` now
  also asserts the singleton, the removed header, `last_odom_state_` and
  `frames.odometry` are absent from all six localization production files.

### Changes outside the Task 5 file list

The plan's list was incomplete; these were required to keep the tree building and
green, and are all direct consequences of the migration:

1. `include/localization_health/tf_continuity_controller.h` — the rename table
   renames `TfPoseSample::map_base_pose`, which is declared here.
2. `include/slam/loop_closure.h` — dead header (no `.cpp`, absent from CMake)
   that included the deleted `slam/config_parameters.h`. Include dropped.
3. `src/common/slam_config.cpp` — `LoadCalibration` now consumes the typed
   `parsed.imu_T_lidar` / `parsed.lidar_T_imu` instead of `parsed.lidar_to_imu`.
4. `include/lidar/lidar_distortion_corrector.{h,cpp}` — member was
   `T_lidar_to_imu_`, holding `lidar_T_imu`; renamed to `lidar_T_imu_`. Only
   `preprocessing.cpp` constructs it.
5. `test/test_lidar_imu_calibration.cpp`, `test/test_relocalization_efficiency_surface.cpp`,
   `test/test_tiled_map_localization_surface.cpp` — all consume the changed
   calibration fields or frozen topic literals and had to be retargeted.
6. `CMakeLists.txt` — `lio_sam_hesai_contracts` needed
   `POSITION_INDEPENDENT_CODE ON`. It was compiled without PIC and previously
   never linked into `liblio_sam_hesai_relocalization_lib.so` because no object
   in it was referenced; the first real `LoadSlamConfig` call pulls
   `slam_config.cpp.o` in and the link fails without this.

### Notes for Tasks 6-9

1. **`test_unified_parameter_schema` is red by design of this handoff.** It
   asserts the relocalization surface is declared in `src/slam/system.cpp`, but
   Task 4 already moved declarations to `src/common/slam_config.cpp`. Fixing it
   needs two coordinated edits that the plan assigns elsewhere: the contract
   entry `{"output.mapping_odom_topic", "output_topic.mapping_odom_topic",
   kRelocalizationSurfaces}` must become legacy
   `output_topic.mapping_odom_topic` -> canonical `output_topic.odometry`
   (Task 6 Step 5), and `config/relocalization.yaml` must switch to
   `output_topic.odometry` (Task 8 Step 1). Changing either alone leaves the
   suite red, so both were left to their owning tasks. `ReadCppParameterNames`
   also only recognises `declare_parameter`/`get_parameter`/`util::param`;
   Task 6 will need to add `ReadParam` for the loader to be readable.

2. **Gravity is runtime state, not configuration.** `InitIMU` estimates
   `gravity_vector_` after ~200 stationary samples and `MakePreIntegrationConfig`
   reads it. It cannot live in the immutable `SlamConfig`, so it moved to
   `System::gravity_vector_` (public, next to `has_imu_init_`) and Localization
   reads it through `system_ptr_`. It is still written and read without a lock,
   exactly as before; adding one would change behaviour and was out of scope.

3. **Three functions lost `static`** because they now read `config_`:
   `System::InitLidarModel`, `System::InitIMU`,
   `PreProcessing::ConvertMessageToCloud`, `PreProcessing::ConvertHesaiMessageToCloud`.

4. **One behaviour-relevant risk to watch in Task 9's replay.**
   `MapTLidar::FromMatrix` / `LocalTImu::FromMatrix` validate orthonormality
   (relative tolerance 1e-12) on the optimizer and preintegration pose path,
   where the code previously smuggled raw `Mat4d` through unchecked. The
   rotations come from `SO3Exp` products so they should be orthonormal to
   ~1e-14, but a replay that throws `std::invalid_argument` out of
   `Localization::Run` would be this. `EstimateMapGroundZ` was deliberately left
   taking `Mat4d`: it only reads the translation, and it also receives candidate
   matrices straight from the Scan Context database, which are not guaranteed
   orthonormal.

5. `eigen2Transform` was removed from `include/common/ros_utility.h` as
   scheduled; `RosPoseAdapter::ToTransformStamped` replaced its last caller.

Task 6's earlier note about the duplicate `declare_parameter` calls in
`src/imageProjection.cpp` still applies. The branch is still NOT merged to
`main`.

---

## Progress: Task 6 complete (2026-09-10)

Commit `f66ec8b` `refactor(mapping): use shared configuration and pose
semantics`. All 7 steps ran. `ParamServer` no longer holds any flat camelCase
config member, and no mapping node hardcodes a `lio_sam/...` topic string.

Verification at this checkpoint:

- Full serial build (`make -j1`, every target): exit 0, no errors.
- Full non-license CTest run: 41/41 gtest pass (Task 5's
  `test_unified_parameter_schema` failure is resolved; see below).
- `uncrustify`, `lint_cmake`, `flake8`, `xmllint` still FAIL as on `main`.
- `test_coordinate_contract_surface` now requires the four mapping nodes to be
  free of the 19 hardcoded topic literals and to resolve them from
  `pipeline_topics`/`outputs` instead.

### Changes outside the Task 6 file list

1. `config/mapping.yaml`, `config/relocalization.yaml` — the two legacy odometry
   keys were renamed to the canonical `output_topic.odometry`, keeping the exact
   same strings. **This pre-empts part of Task 8 Step 1.** It was necessary
   because Task 6 Step 5 reclassifies `output_topic.mapping_odom_topic` as a
   legacy alias, and `ProjectSurfaceRejectsLegacyExternalNames` then rejects the
   repository YAML until the keys move. Task 8 still owns `pipeline_topic.*`,
   `command_topic.*`, RViz, launch argument naming, and the migration document.
2. `test_ground_patch_parameter_surface.cpp`,
   `test_relocalization_efficiency_surface.cpp` (Task 5),
   `test_tiled_map_localization_surface.cpp` — surface tests that asserted the
   old flat field names or the old declaration site.

### Notes for Tasks 7-9

1. **Two runtime overrides replaced in-place config mutation.**
   `mapOptimization` used to do `loopClosureEnableFlag = false` and
   `savePCD = false` on itself when `mappingMode` was false. The config is now
   immutable, so those became `loop_closure_enabled_` / `save_map_enabled_`
   members initialised from the config and then flipped. Same behaviour, same
   warning text.
2. **`main()` needed one narrow predicate.** It read `MO->config_...` directly,
   which is no longer reachable now that `config_` is protected. Added
   `ParamServer::MappingMode()` rather than exposing the whole config.
3. **`publishOdometry` is the typed mapping output boundary.** The global
   odometry message is now built from `OdomTLidar` via `RosPoseAdapter`. The
   rotation is taken from `trans2Affine3f(transformTobeMapped)` (float) rather
   than the previous `tf2::setRPY` (double), so the published quaternion can
   differ by ~1e-7. That is float-serialisation noise, well below any baseline
   threshold, but it is the one place Task 9's replay could in principle show a
   numeric delta. The incremental odometry still uses its original slerp filter
   untouched.
4. **Parameter logging changed.** The constructor no longer logs each parameter
   via `util::param` (the shared loader does not log), and the trailing
   `usleep(100)` workaround was dropped along with the declarations. Startup
   output is quieter; no parameter value changed.
5. `test_unified_parameter_schema`'s `ReadCppParameterNames` now also recognises
   `ReadParam`, so the shared loader counts as a parameter reader. Both the
   mapping and relocalization reader assertions point at
   `src/common/slam_config.cpp` plus the node that consumes the nest.
6. Log strings that my bulk field rename had rewritten
   (`"mappingMode=false: ..."`) were restored to their original text; the two
   RCLCPP_WARN messages are byte-identical to pre-P2.2.

The branch is still NOT merged to `main`.

---

## Progress: Task 7 complete (2026-09-10)

Commit `bd1419f` `refactor(map): centralize artifact path conventions`. All 5
steps ran. `MapArtifactLayout` is the single source of truth for the on-disk
map layout, and `kGlobalMapFileName = "map.pcd"` is gone.

Verification at this checkpoint:

- Full serial build (`make -j1`, every target): exit 0, no errors.
- Full non-license CTest run: 42/46 pass. `uncrustify`, `lint_cmake`, `flake8`,
  `xmllint` still FAIL as on `main`; no functional test regressed.
- Step 4's regression set (`test_map_artifact_layout`, `test_tile_map_generator`,
  `test_global_map_provider`, `test_tiled_map_provider`,
  `test_localization_stability_common`) all pass.

### Changes outside the Task 7 file list

1. `src/localization_map/tile_manifest.cpp` — the manifest codec still spelled
   `manifest.yaml` itself while `MapArtifactLayout::TiledManifest()` also names
   it. Two independent constants for one file name is a correctness coupling, so
   the codec now uses `MapArtifactLayout::kManifestName`. Only the *name* is
   shared; the manifest contents are untouched.
2. `test_auto_relocalization_parameter_surface.cpp` — asserted the literal
   `sc_db.bin` lived in `src/mapOptimization.cpp`; it now asserts the saver calls
   `ScanContextDb()` and that the layout owns the name.
3. `CMakeLists.txt` — `test_tile_map_generator` needed to link
   `${PROJECT_NAME}_contracts` because the test fixtures now build a layout.

### Notes for Tasks 8-9

1. **One intentional default changed.** The empty `localization.map_path` used to
   resolve to `<PROJECT_SOURCE_DIR>/data/map.pcd`; it now resolves through
   `MapArtifactLayout(kDataPath).GlobalMap()` to
   `<PROJECT_SOURCE_DIR>/data/GlobalMap.pcd`. That is the plan's stated intent
   ("Remove `kGlobalMapFileName = "map.pcd"`"), and `map.pcd` never existed under
   `data/`, so the old default was dead. No configured path is affected.
2. **The shutdown auto-save keeps its legacy file names.** `visualizeGlobalMapThread`
   writes `cloudCorner.pcd`, `cloudSurf.pcd`, `cloudGlobal.pcd`, `cloudGround.pcd`,
   where the save-map service writes `CornerMap.pcd`, `SurfMap.pcd`,
   `GlobalMap.pcd`, `GroundMap.pcd`. P2.2 does not rename on-disk artifacts, so
   the four legacy names are exposed from the layout as `SavedCornerCloud()`,
   `SavedSurfaceCloud()`, `SavedGlobalCloud()`, `SavedGroundCloud()` instead of
   being concatenated at the call site. **This looks like a real defect**:
   nothing reads the `cloud*.pcd` names back, so a map saved only by Ctrl+C
   cannot be loaded by localization or tiled by the generator, which both look
   for `GlobalMap.pcd`. Fixing it means renaming files, which is explicitly out
   of P2.2 scope — worth a separate change.
3. `FromGlobalMap` rejects any global map not named `GlobalMap.pcd`, but it is
   deliberately *not* used on the user-configured `localization.map_path`, which
   may be any regular file. The artifact derivations there use
   `MapArtifactLayout(resolved_map_path_.parent_path())` instead, so a custom
   global map file name still works exactly as before.

The branch is still NOT merged to `main`.

---

## Progress: Task 8 complete (2026-09-12)

Commit `1bd98bc` `docs(config): publish P2.2 naming migration`. All 6 steps ran
except Step 3, which needed no change (see below).

Verification at this checkpoint:

- `python3 -m py_compile` on all three launch files: OK.
- Full serial build (`make -j1`, every target): exit 0, no errors.
- Full non-license CTest run: 42/46 pass. `uncrustify`, `lint_cmake`, `flake8`,
  `xmllint` still FAIL as on `main`; no functional test regressed.
- Step 5's named set (`test_unified_parameter_schema`,
  `test_lidar_imu_calibration`, `test_coordinate_contract_surface`) all pass.

### Step 3 required no change

Every topic-valued entry in `config/rviz2.rviz` already resolves to a canonical
configuration value. The 11 entries were checked mechanically
(`RvizTopicsResolveToCanonicalConfigValues` now enforces it), so no RViz edit
was needed rather than an edit being skipped. The leading-slash difference
(`/lio_sam/mapping/path` vs `lio_sam/mapping/path`) is the same ROS topic, so
the test compares with the slash normalized instead of rewriting a hand-tuned
RViz config.

### Gap found and closed

- `src/mapOptimization.cpp` still hardcoded `"lio_loop/loop_closure_detection"`
  for `subLoop`. Task 6 Step 3 said to replace every publisher/subscriber
  literal; this one was missed because the constructor call and the string sit
  on different lines, so the literal-rejection grep in
  `test_coordinate_contract_surface` did not see it. It now reads
  `config_.pipeline_topics.loop_closure_hint`; the string is unchanged.

### Notes for Task 9

1. **`test_unified_parameter_schema` gained a second contract table.**
   `kCanonicalTopicContracts` holds canonical keys with no legacy counterpart
   (the topics that used to be hardcoded in the node constructors).
   `CanonicalNamesForSurface` reads both tables; `LegacyNames` reads only the
   first, so these keys are never rejected by the legacy-name check.
2. **The loader's `ReadDerivedLegacy` now counts as a parameter reader** in
   `ReadCppParameterNames`, otherwise `pipeline_topic.imu_odometry_incremental`
   would look undeclared.
3. **Launch argument resolution was verified against real ROS 2 launch
   classes**, not just `py_compile`: with only `use_sim_time` set the canonical
   value is used; with `use_sim` explicitly set the legacy value wins. All
   defaults are unchanged (mapping `use_sim_time=false`, localization and
   nonground `use_sim_time=true`) and `publish_static_tf` keeps its per-file
   values (`true` in `localization.launch.py`, `false` in
   `localization_with_nonground.launch.py`).
4. **`docs/P2_2_MIGRATION.md` is tracked but `docs/` is in `.gitignore`**, so it
   needs `git add -f` on a fresh clone.
5. The legacy→canonical table in that document was generated from
   `kParameterContracts` rather than hand-written; regenerate it the same way if
   the contract table changes.

The branch is still NOT merged to `main`.
