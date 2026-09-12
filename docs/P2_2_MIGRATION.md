# P2.2 Parameter Key Migration

**Scope:** the P2.2 mapping/localization coordinate and naming convergence
(`docs/superpowers/specs/2026-09-10-mapping-localization-coordinate-naming-unification-design.md`).

This document tells an operator which parameter keys changed, which ones still
work, and what happens when both spellings are present. Nothing here changes a
ROS topic string, a TF topology, a disk file name or an algorithm: the two
repository YAML files produce exactly the same runtime graph as before P2.2.

## 1. Removal policy

> Legacy parameter aliases remain enabled throughout the P2.2 release. They may be removed only by a separately reviewed compatibility-cleanup change after P2.3 acceptance and a deployed-configuration audit.

## 2. Alias resolution rules

Every migrated key is read through one resolver with deterministic precedence:

1. The canonical key is set, the legacy key is not -> the canonical value is
   used, nothing is logged.
2. Only the legacy key is set -> the legacy value is used and a single `WARN` is
   emitted for that key, once per node.
3. Both are set to the same value -> the canonical value is used and a single
   `WARN` is emitted.
4. Both are set to different values -> startup is **refused** with
   `ParameterAliasConflict` naming both keys and both values. There is no
   silent winner.
5. An empty required frame or topic, a non-finite or out-of-range number, or a
   calibration that is not a rigid transform -> startup is refused with
   `std::invalid_argument`.

Shipping both spellings with the same value is safe but warns; prefer deleting
the legacy key.

### Example: legacy only (starts, warns)

```yaml
sensor_topic:
  odom_topic: "odometry/imu"     # legacy
```

```text
[WARN] Parameter 'sensor_topic.odom_topic' is deprecated; use
'output_topic.odometry' instead. The legacy alias remains supported for the
P2.2 release.
```

### Example: conflict (refuses to start)

```yaml
output_topic:
  odometry: "odometry/imu"
sensor_topic:
  odom_topic: "odometry/other"   # different value
```

```text
ParameterAliasConflict: conflicting parameter values: 'output_topic.odometry'
= odometry/imu but deprecated 'sensor_topic.odom_topic' = odometry/other;
set only one of them
```

## 3. Coordinate convention

Every cross-module rigid transform is named `parent_T_child` and means "the
pose of `child` expressed in `parent`", i.e. it maps child coordinates into the
parent frame. Only two operations are legal, and both keep the frame types
checkable at compile time:

```text
A_T_B * B_T_C = A_T_C
inverse(A_T_B) = B_T_A
```

Frame tags: `MapFrame`, `OdomFrame`, `BaseFrame`, `ImuFrame`, `LidarFrame`,
`LocalIntegrationFrame`. `LocalIntegrationFrame` is localization's internal
relative preintegration state; it is deliberately **not** `OdomFrame` so it can
never be mistaken for a ROS odom transform.

Setting a pose is unambiguously `map_T_base`. It is converted to
`map_T_lidar` through `base_T_lidar` for the matcher, and localization output is
converted back with `map_T_lidar * lidar_T_base = map_T_base`.

## 4. TF topology (unchanged)

Mapping:

```text
map --静态单位变换--> odom --动态--> base_link
                                  |--> imu
                                  `--> lidar
```

Localization:

```text
map --动态--> base_link
              |--> imu
              `--> lidar
```

Localization never looks up and never publishes an odom transform. That was
removed in `fe61c5e` and P2.2 does not reintroduce it.

## 5. Legacy to canonical key table

Surfaces column: where the canonical key must be present. "reader" is the C++
node or shared loader; "YAML" is the repository configuration file.

| Legacy key | Canonical key | Surfaces |
|---|---|---|
| `pointCloudTopic` | `sensor_topic.lidar_topic` | mapping reader, localization reader, mapping YAML, localization YAML |
| `imuTopic` | `sensor_topic.imu_topic` | mapping reader, localization reader, mapping YAML, localization YAML |
| `odomTopic` | `output_topic.odometry` | mapping reader, mapping YAML |
| `sensor_topic.odom_topic` | `output_topic.odometry` | mapping reader, mapping YAML |
| `gpsTopic` | `sensor_topic.gps_topic` | mapping reader, mapping YAML |
| `nonGroundCloudTopic` | `output_topic.non_ground_cloud_topic` | mapping reader, mapping YAML, localization YAML |
| `sensor_topic.non_ground_cloud_topic` | `output_topic.non_ground_cloud_topic` | mapping reader, mapping YAML, localization YAML |
| `output_topic.mapping_odom_topic` | `output_topic.odometry` | localization reader, localization YAML |
| `mapFrame` | `frames.map` | mapping reader, localization reader, mapping YAML, localization YAML |
| `odometryFrame` | `frames.odometry` | mapping reader, mapping YAML |
| `baselinkFrame` | `frames.base_link` | mapping reader, localization reader, mapping YAML, localization YAML |
| `lidarFrame` | `frames.lidar` | mapping reader, localization reader, mapping YAML, localization YAML |
| `mappingMode` | `system.mapping_mode` | mapping reader, mapping YAML, localization YAML |
| `numberOfCores` | `system.number_of_cores` | mapping reader, mapping YAML |
| `mappingProcessInterval` | `system.mapping_process_interval` | mapping reader, mapping YAML |
| `useImuHeadingInitialization` | `gps.use_imu_heading_initialization` | mapping reader, mapping YAML |
| `useGpsElevation` | `gps.use_elevation` | mapping reader, mapping YAML |
| `gpsCovThreshold` | `gps.covariance_threshold` | mapping reader, mapping YAML |
| `poseCovThreshold` | `gps.pose_covariance_threshold` | mapping reader, mapping YAML |
| `sensor` | `lidar.sensor_type` | mapping reader, localization reader, mapping YAML, localization YAML |
| `lidar.lidar_sensor_type` | `lidar.sensor_type` | localization reader, localization YAML |
| `N_SCAN` | `lidar.scan_lines` | mapping reader, localization reader, mapping YAML, localization YAML |
| `lidar.lidar_scan` | `lidar.scan_lines` | localization reader, localization YAML |
| `Horizon_SCAN` | `lidar.horizon_scan` | mapping reader, localization reader, mapping YAML, localization YAML |
| `lidar.lidar_horizon_scan` | `lidar.horizon_scan` | localization reader, localization YAML |
| `downsampleRate` | `lidar.downsample_rate` | mapping reader, mapping YAML |
| `lidarMinRange` | `lidar.min_range` | mapping reader, localization reader, mapping YAML, localization YAML |
| `lidar.lidar_use_min_distance` | `lidar.min_range` | localization reader, localization YAML |
| `lidarMaxRange` | `lidar.max_range` | mapping reader, localization reader, mapping YAML, localization YAML |
| `lidar.lidar_use_max_distance` | `lidar.max_range` | localization reader, localization YAML |
| `lidar.lidar_point_jump_span` | `lidar.point_jump_span` | localization reader, localization YAML |
| `lidar.lidar_point_time_scale` | `lidar.point_time_scale` | localization reader, localization YAML |
| `lidar.lidar_rotation_noise_std` | `lidar.rotation_noise_std` | localization reader, localization YAML |
| `lidar.lidar_position_noise_std` | `lidar.position_noise_std` | localization reader, localization YAML |
| `lidar.lidar_lower_angle` | `lidar.lower_angle_deg` | localization reader, localization YAML |
| `lidar.lidar_vertical_resolution` | `lidar.vertical_resolution_deg` | localization reader, localization YAML |
| `imuType` | `imu.type` | mapping reader, mapping YAML |
| `imuAccNoise` | `imu.acc_noise_std` | mapping reader, localization reader, mapping YAML, localization YAML |
| `imuGyrNoise` | `imu.gyro_noise_std` | mapping reader, localization reader, mapping YAML, localization YAML |
| `imuAccBiasN` | `imu.acc_bias_rw_noise_std` | mapping reader, localization reader, mapping YAML, localization YAML |
| `imu.acc_rw_noise_std` | `imu.acc_bias_rw_noise_std` | localization reader, localization YAML |
| `imuGyrBiasN` | `imu.gyro_bias_rw_noise_std` | mapping reader, localization reader, mapping YAML, localization YAML |
| `imu.gyro_rw_noise_std` | `imu.gyro_bias_rw_noise_std` | localization reader, localization YAML |
| `imuGravity` | `imu.gravity` | mapping reader, localization reader, mapping YAML, localization YAML |
| `gravity` | `imu.gravity` | localization reader, localization YAML |
| `imuRPYWeight` | `imu.rpy_weight` | mapping reader, mapping YAML |
| `extrinsicTrans` | `calibration.translation` | mapping reader, localization reader, mapping YAML, localization YAML |
| `extrinsicRot` | `calibration.rotation` | mapping reader, localization reader, mapping YAML, localization YAML |
| `extrinsicRPY` | `calibration.rpy` | mapping reader, localization reader, mapping YAML, localization YAML |
| `calibration.lidar_to_imu` | `calibration.translation` | localization reader, localization YAML |
| `savePCD` | `map_save.enabled` | mapping reader, mapping YAML |
| `generateScDb` | `map_save.generate_scan_context_db` | mapping reader, mapping YAML |
| `savePCDDirectory` | `map_save.directory` | mapping reader, mapping YAML |
| `savePCDResolution` | `map_save.resolution` | mapping reader, mapping YAML |
| `saveGroundPCD` | `map_save.save_ground_map` | mapping reader, mapping YAML |
| `saveGroundPCDResolution` | `map_save.ground_resolution` | mapping reader, mapping YAML |
| `groundPCDDirectory` | `map_save.ground_directory` | mapping reader, mapping YAML |
| `groundPCDDirectoryGlobal` | `map_save.ground_global_directory` | mapping reader, mapping YAML |
| `groundFinalUseOdom` | `map_save.ground_final_use_odometry` | mapping reader, mapping YAML |
| `groundGlobalLeafSize` | `map_save.ground_global_leaf_size` | mapping reader, mapping YAML |
| `alignSavedMapGroundToZero` | `map_save.align_ground_to_zero` | mapping reader, mapping YAML |
| `savedMapGroundZOffset` | `map_save.ground_z_offset` | mapping reader, mapping YAML |
| `segmentation_only_mode` | `ground_segmentation.only_mode` | mapping reader, mapping YAML, localization YAML, localization launch |
| `groundLeafSize` | `ground_segmentation.leaf_size` | mapping reader, mapping YAML, localization YAML |
| `groundScanStartIndex` | `ground_segmentation.scan_start_index` | mapping reader, mapping YAML, localization YAML |
| `groundScanEndIndex` | `ground_segmentation.scan_end_index` | mapping reader, mapping YAML, localization YAML |
| `groundAngleThreshold` | `ground_segmentation.angle_threshold_deg` | mapping reader, mapping YAML, localization YAML |
| `groundPlaneDistance` | `ground_segmentation.plane_distance` | mapping reader, mapping YAML, localization YAML |
| `groundPlaneMaxAngle` | `ground_segmentation.plane_max_angle_deg` | mapping reader, mapping YAML, localization YAML |
| `groundPlaneMinPoints` | `ground_segmentation.plane_min_points` | mapping reader, mapping YAML, localization YAML |
| `groundFlattenZ` | `ground_segmentation.flatten_z` | mapping reader, mapping YAML, localization YAML |
| `groundFlattenValue` | `ground_segmentation.flatten_value` | mapping reader, mapping YAML, localization YAML |
| `groundGridPatchEnable` | `ground_segmentation.grid_patch.enable` | mapping reader, mapping YAML, localization YAML |
| `groundGridPatchResolution` | `ground_segmentation.grid_patch.resolution` | mapping reader, mapping YAML, localization YAML |
| `groundGridPatchSearchRadius` | `ground_segmentation.grid_patch.search_radius` | mapping reader, mapping YAML, localization YAML |
| `groundGridPatchMaxSlopeDeg` | `ground_segmentation.grid_patch.max_slope_deg` | mapping reader, mapping YAML, localization YAML |
| `groundGridPatchMaxHeightRange` | `ground_segmentation.grid_patch.max_height_range` | mapping reader, mapping YAML, localization YAML |
| `groundGridPatchMaxRoughness` | `ground_segmentation.grid_patch.max_roughness` | mapping reader, mapping YAML, localization YAML |
| `groundGridPatchMaxPredictionMargin` | `ground_segmentation.grid_patch.max_prediction_margin` | mapping reader, mapping YAML, localization YAML |
| `groundGridPatchMinInlierRatio` | `ground_segmentation.grid_patch.min_inlier_ratio` | mapping reader, mapping YAML, localization YAML |
| `groundGridPatchMinNeighborCells` | `ground_segmentation.grid_patch.min_neighbor_cells` | mapping reader, mapping YAML, localization YAML |
| `groundGridPatchMinSupportQuadrants` | `ground_segmentation.grid_patch.min_support_quadrants` | mapping reader, mapping YAML, localization YAML |
| `edgeThreshold` | `feature_extraction.edge_threshold` | mapping reader, mapping YAML |
| `surfThreshold` | `feature_extraction.surface_threshold` | mapping reader, mapping YAML |
| `edgeFeatureMinValidNum` | `feature_extraction.min_edge_features` | mapping reader, mapping YAML |
| `surfFeatureMinValidNum` | `feature_extraction.min_surface_features` | mapping reader, mapping YAML |
| `odometrySurfLeafSize` | `voxel_filter.odometry_surface_leaf_size` | mapping reader, mapping YAML |
| `mappingCornerLeafSize` | `voxel_filter.mapping_corner_leaf_size` | mapping reader, mapping YAML |
| `mappingSurfLeafSize` | `voxel_filter.mapping_surface_leaf_size` | mapping reader, mapping YAML |
| `dynamicKeyframeFilterEnable` | `dynamic_filter.enable` | mapping reader, mapping YAML |
| `dynamicSurfKnnMaxDist` | `dynamic_filter.surface_knn_max_distance` | mapping reader, mapping YAML |
| `dynamicSurfPlaneMaxDist` | `dynamic_filter.surface_plane_max_distance` | mapping reader, mapping YAML |
| `globalStaticMapFilterEnable` | `static_map_filter.enable` | mapping reader, mapping YAML |
| `globalStaticVoxelSize` | `static_map_filter.voxel_size` | mapping reader, mapping YAML |
| `globalStaticMinKF` | `static_map_filter.min_keyframes` | mapping reader, mapping YAML |
| `globalStaticFilterCorners` | `static_map_filter.filter_corners` | mapping reader, mapping YAML |
| `z_tollerance` | `motion_constraint.z_tolerance` | mapping reader, mapping YAML |
| `rotation_tollerance` | `motion_constraint.rotation_tolerance` | mapping reader, mapping YAML |
| `surroundingkeyframeAddingDistThreshold` | `local_map.keyframe_distance_threshold` | mapping reader, mapping YAML |
| `surroundingkeyframeAddingAngleThreshold` | `local_map.keyframe_angle_threshold` | mapping reader, mapping YAML |
| `surroundingKeyframeDensity` | `local_map.keyframe_density` | mapping reader, mapping YAML |
| `surroundingKeyframeSearchRadius` | `local_map.search_radius` | mapping reader, mapping YAML |
| `loopClosureEnableFlag` | `loop_closure.enable` | mapping reader, localization reader, mapping YAML, localization YAML |
| `system.enable_loopclosure` | `loop_closure.enable` | localization reader, localization YAML |
| `loopClosureFrequency` | `loop_closure.frequency` | mapping reader, mapping YAML |
| `surroundingKeyframeSize` | `loop_closure.surrounding_keyframe_size` | mapping reader, mapping YAML |
| `historyKeyframeSearchRadius` | `loop_closure.history_search_radius` | mapping reader, mapping YAML |
| `historyKeyframeSearchTimeDiff` | `loop_closure.history_search_time_difference` | mapping reader, mapping YAML |
| `historyKeyframeSearchNum` | `loop_closure.history_search_count` | mapping reader, mapping YAML |
| `historyKeyframeFitnessScore` | `loop_closure.fitness_threshold` | mapping reader, mapping YAML |
| `loopclosure.candidate_local_map_left_range` | `loop_closure.candidate_local_map_left_range` | localization reader, localization YAML |
| `loopclosure.candidate_local_map_right_range` | `loop_closure.candidate_local_map_right_range` | localization reader, localization YAML |
| `loopclosure.loopclosure_local_map_left_range` | `loop_closure.loopclosure_local_map_left_range` | localization reader, localization YAML |
| `loopclosure.near_neighbor_distance_threshold` | `loop_closure.near_neighbor_distance_threshold` | localization reader, localization YAML |
| `loopclosure.registration_converge_threshold` | `loop_closure.registration_converge_threshold` | localization reader, localization YAML |
| `loopclosure.skip_near_keyframe_threshold` | `loop_closure.skip_near_keyframe_threshold` | localization reader, localization YAML |
| `loopclosure.skip_near_loopclosure_threshold` | `loop_closure.skip_near_loopclosure_threshold` | localization reader, localization YAML |
| `globalMapVisualizationSearchRadius` | `visualization.global_map_search_radius` | mapping reader, mapping YAML |
| `globalMapVisualizationPoseDensity` | `visualization.global_map_pose_density` | mapping reader, mapping YAML |
| `globalMapVisualizationLeafSize` | `visualization.global_map_leaf_size` | mapping reader, mapping YAML |

## 6. Topic classification

Keys are grouped by who owns the value:

- `sensor_topic.*` - device or upstream sensor input.
- `pipeline_topic.*` - package-internal protocol between nodes. Downstream
  consumers normally do not subscribe to these.
- `command_topic.*` - external control input, for example `/initialpose`.
- `output_topic.*` - results intended for downstream consumers.

The previously misclassified `sensor_topic.odom_topic` is now two explicit keys:
`output_topic.odometry` (the fused output) and
`pipeline_topic.imu_odometry_incremental` (the IMU preintegration delta).
`output_topic.mapping_odom_topic` is now localization's `output_topic.odometry`.

## 7. Map artifact layout

The on-disk layout is unchanged and is now defined in one place,
`include/common/map_artifact_layout.h`:

```text
<map_root>/GlobalMap.pcd
<map_root>/GroundMap.pcd
<map_root>/sc_db.bin
<map_root>/trajectory.pcd
<map_root>/transformations.pcd
<map_root>/CornerMap.pcd
<map_root>/SurfMap.pcd
<map_root>/<tile_subdirectory>/manifest.yaml
```

No file is renamed by P2.2. The manifest content, including
`source.global_map_name` and the `frame_id`, is byte-for-byte what it was.

One default did change: an empty `localization.map_path` used to resolve to
`<package>/data/map.pcd` and now resolves to `<package>/data/GlobalMap.pcd`
through the layout. `map.pcd` never existed under `data/`, so the old default
was dead. Any explicitly configured path is unaffected.

## 8. Launch argument naming

`use_sim_time` is the canonical launch argument for all three launch files.
`use_sim` is still accepted for one cycle: an explicitly supplied `use_sim`
wins over `use_sim_time`, otherwise the canonical value is used. Every default,
every static transform and every node name is unchanged.
