# Unified ROS Parameters Design

## Goal

Replace the project's mixed top-level camelCase ROS2 parameters with one
functionally grouped, snake_case schema shared by mapping and relocalization.
This is a hard cutover: legacy parameter names are removed rather than retained
as aliases.

## Scope

The migration covers:

- `config/mapping.yaml`
- `config/relocalization.yaml`
- every parameter declaration/read in `include/lio_sam/utility.hpp`
- every parameter declaration/read in `src/imageProjection.cpp`
- the relocalization parameter reads in `src/slam/system.cpp`
- parameter overrides in all launch files
- parameter-surface tests and launch/build smoke tests

Internal C++ member names such as `groundLeafSize` remain unchanged. Renaming
members would add unrelated behavioral risk; only the external ROS parameter
contract changes.

## Canonical Groups

| Group | Responsibility |
|---|---|
| `system` | operating mode, CPU count, processing cadence, global switches |
| `sensor_topic` | lidar, IMU, odometry, GPS, and non-ground topics |
| `frames` | map, odometry, base, IMU, and lidar frame IDs |
| `output` | relocalization output topics |
| `gps` | GPS initialization and covariance gates |
| `lidar` | model, scan geometry, range gates, sampling, timing, noise |
| `imu` | type, orientation use, biases, noise, random walk, gravity, weighting |
| `calibration` | lidar-to-IMU translation, rotation, and orientation matrices |
| `localization` | global map, Scan Context, consistency, ground-height constraint |
| `frontend` | relocalization feature extraction, registration, and fusion |
| `map_save` | map/GroundMap/Scan-Context export configuration |
| `ground_segmentation` | ground extraction, planar checks, output flattening |
| `ground_segmentation.grid_patch` | elevation-grid gap filling |
| `feature_extraction` | LIO-SAM edge/surface feature gates |
| `voxel_filter` | odometry and mapping downsampling |
| `dynamic_filter` | local geometric consistency filtering |
| `static_map_filter` | persistent global-static filtering |
| `motion_constraint` | vertical and rotational motion limits |
| `local_map` | keyframe insertion and surrounding-map construction |
| `loop_closure` | mapping and relocalization loop-closure controls |
| `visualization` | global-map visualization range and downsampling |

## Canonical Parameter Names

### Common topics and frames

| Legacy/current name | Canonical name |
|---|---|
| `pointCloudTopic`, `sensor_topic.lidar_topic` | `sensor_topic.lidar_topic` |
| `imuTopic`, `sensor_topic.imu_topic` | `sensor_topic.imu_topic` |
| `odomTopic` | `sensor_topic.odom_topic` |
| `gpsTopic` | `sensor_topic.gps_topic` |
| `nonGroundCloudTopic` | `sensor_topic.non_ground_cloud_topic` |
| `mapFrame`, `frames.map` | `frames.map` |
| `odometryFrame` | `frames.odometry` |
| `baselinkFrame`, `frames.base_link` | `frames.base_link` |
| `frames.imu` | `frames.imu` (relocalization only — see note) |
| `lidarFrame`, `frames.lidar` | `frames.lidar` |

### System, GPS, and lidar

| Legacy/current name | Canonical name |
|---|---|
| `mappingMode` | `system.mapping_mode` |
| `numberOfCores` | `system.number_of_cores` |
| `mappingProcessInterval` | `system.mapping_process_interval` |
| `useImuHeadingInitialization` | `gps.use_imu_heading_initialization` |
| `useGpsElevation` | `gps.use_elevation` |
| `gpsCovThreshold` | `gps.covariance_threshold` |
| `poseCovThreshold` | `gps.pose_covariance_threshold` |
| `sensor`, `lidar.lidar_sensor_type` | `lidar.sensor_type` |
| `N_SCAN`, `lidar.lidar_scan` | `lidar.scan_lines` |
| `Horizon_SCAN`, `lidar.lidar_horizon_scan` | `lidar.horizon_scan` |
| `downsampleRate` | `lidar.downsample_rate` |
| `lidarMinRange`, `lidar.lidar_use_min_distance` | `lidar.min_range` |
| `lidarMaxRange`, `lidar.lidar_use_max_distance` | `lidar.max_range` |
| `lidar.lidar_point_jump_span` | `lidar.point_jump_span` |
| `lidar.lidar_point_time_scale` | `lidar.point_time_scale` |
| `lidar.lidar_rotation_noise_std` | `lidar.rotation_noise_std` |
| `lidar.lidar_position_noise_std` | `lidar.position_noise_std` |
| `lidar.lidar_lower_angle` | `lidar.lower_angle_deg` |
| `lidar.lidar_vertical_resolution` | `lidar.vertical_resolution_deg` |

### IMU and calibration

| Legacy/current name | Canonical name |
|---|---|
| `imuType` | `imu.type` |
| `imuAccNoise`, `imu.acc_noise_std` | `imu.acc_noise_std` |
| `imuGyrNoise`, `imu.gyro_noise_std` | `imu.gyro_noise_std` |
| `imuAccBiasN`, `imu.acc_rw_noise_std` | `imu.acc_bias_rw_noise_std` |
| `imuGyrBiasN`, `imu.gyro_rw_noise_std` | `imu.gyro_bias_rw_noise_std` |
| `imuGravity`, `gravity` | `imu.gravity` (dual reader — see note) |
| `imuRPYWeight` | `imu.rpy_weight` |
| `imu.has_orientation` | `imu.has_orientation` |
| `imu.init_acc_bias` | `imu.init_acc_bias` |
| `imu.init_gyro_bias` | `imu.init_gyro_bias` |
| `imu.data_searcher_buffer_size` | `imu.data_searcher_buffer_size` |
| `extrinsicTrans` | `calibration.translation` |
| `extrinsicRot` | `calibration.rotation` |
| `extrinsicRPY` | `calibration.rpy` |
| `calibration.lidar_to_imu` | derived from `calibration.translation` and `calibration.rotation` |

`calibration.translation` contains three values. `calibration.rotation` and
`calibration.rpy` contain row-major 3x3 matrices. The relocalization system
constructs its 4x4 lidar-to-IMU transform from `translation` and `rotation`, so
mapping and localization no longer maintain two independent representations.

#### Direction convention (must be honored — verified against current code)

The two subsystems currently store the extrinsic in **opposite directions**, so
the canonical schema must fix one direction and require each consumer to derive
what it needs:

- Mapping (`utility.hpp`) uses `extRot` as `R^{lidar}_{imu}` — it rotates an
  IMU-frame vector into the lidar frame (`acc = extRot * acc`,
  `utility.hpp:470`).
- Relocalization (`system.cpp`) uses `calibration.lidar_to_imu` as the 4x4
  `T^{imu}_{lidar}` — it maps lidar-frame coordinates into the IMU frame
  (verified via `MapImuPoseToMapLidarPose = map_imu * calibration_lidar_to_imu_`,
  `localization.cpp:95-98`). Its rotation block is therefore `R^{imu}_{lidar}`,
  the transpose of mapping's `extRot`.

**Canonical definition:** `calibration.rotation` and `calibration.translation`
are defined as the lidar-to-IMU transform `T^{imu}_{lidar}` (i.e. the pose of the
lidar frame expressed in the IMU frame). Consumers derive as follows:

- Relocalization builds its 4x4 `calibration_lidar_to_imu_` directly by placing
  `calibration.rotation` in the top-left 3x3 block and `calibration.translation`
  in the top-right 3x1 block. No inversion.
- Mapping sets `extRot = calibration.rotation.transpose()` (because it needs
  `R^{lidar}_{imu}`) and `extTrans` from `calibration.translation`. `extRPY`
  (`extQRPY`) is derived from `calibration.rpy` unchanged, preserving the
  existing LIO-SAM behavior where the orientation extrinsic may differ from the
  acc/gyro rotation extrinsic.

Both YAML files carry identical `calibration.translation`/`rotation`/`rpy`
values. The canonical translation is `[0.0, 0.0, 0.10]` metres, selected from
the existing mapping calibration. The default identity rotation leaves both
consumers' rotations unchanged; the transpose only matters for a non-identity
mounting.

### Map export and ground processing

| Legacy/current name | Canonical name |
|---|---|
| `savePCD` | `map_save.enabled` |
| `generateScDb` | `map_save.generate_scan_context_db` |
| `savePCDDirectory` | `map_save.directory` |
| `savePCDResolution` | `map_save.resolution` |
| `saveGroundPCD` | `map_save.save_ground_map` |
| `saveGroundPCDResolution` | `map_save.ground_resolution` |
| `groundPCDDirectory` | `map_save.ground_directory` |
| `groundPCDDirectoryGlobal` | `map_save.ground_global_directory` |
| `groundFinalUseOdom` | `map_save.ground_final_use_odometry` |
| `groundGlobalLeafSize` | `map_save.ground_global_leaf_size` |
| `alignSavedMapGroundToZero` | `map_save.align_ground_to_zero` |
| `savedMapGroundZOffset` | `map_save.ground_z_offset` |
| `segmentation_only_mode` | `ground_segmentation.only_mode` |
| `groundLeafSize` | `ground_segmentation.leaf_size` |
| `groundScanStartIndex` | `ground_segmentation.scan_start_index` |
| `groundScanEndIndex` | `ground_segmentation.scan_end_index` |
| `groundAngleThreshold` | `ground_segmentation.angle_threshold_deg` |
| `groundPlaneDistance` | `ground_segmentation.plane_distance` |
| `groundPlaneMaxAngle` | `ground_segmentation.plane_max_angle_deg` |
| `groundPlaneMinPoints` | `ground_segmentation.plane_min_points` |
| `groundFlattenZ` | `ground_segmentation.flatten_z` |
| `groundFlattenValue` | `ground_segmentation.flatten_value` |

Grid-patch keys move below `ground_segmentation.grid_patch`:

- `groundGridPatchEnable` → `enable`
- `groundGridPatchResolution` → `resolution`
- `groundGridPatchSearchRadius` → `search_radius`
- `groundGridPatchMaxSlopeDeg` → `max_slope_deg`
- `groundGridPatchMaxHeightRange` → `max_height_range`
- `groundGridPatchMaxRoughness` → `max_roughness`
- `groundGridPatchMaxPredictionMargin` → `max_prediction_margin`
- `groundGridPatchMinInlierRatio` → `min_inlier_ratio`
- `groundGridPatchMinNeighborCells` → `min_neighbor_cells`
- `groundGridPatchMinSupportQuadrants` → `min_support_quadrants`

### Mapping filters and optimization

| Legacy name | Canonical name |
|---|---|
| `edgeThreshold` | `feature_extraction.edge_threshold` |
| `surfThreshold` | `feature_extraction.surface_threshold` |
| `edgeFeatureMinValidNum` | `feature_extraction.min_edge_features` |
| `surfFeatureMinValidNum` | `feature_extraction.min_surface_features` |
| `odometrySurfLeafSize` | `voxel_filter.odometry_surface_leaf_size` |
| `mappingCornerLeafSize` | `voxel_filter.mapping_corner_leaf_size` |
| `mappingSurfLeafSize` | `voxel_filter.mapping_surface_leaf_size` |
| `dynamicKeyframeFilterEnable` | `dynamic_filter.enable` |
| `dynamicSurfKnnMaxDist` | `dynamic_filter.surface_knn_max_distance` |
| `dynamicSurfPlaneMaxDist` | `dynamic_filter.surface_plane_max_distance` |
| `globalStaticMapFilterEnable` | `static_map_filter.enable` |
| `globalStaticVoxelSize` | `static_map_filter.voxel_size` |
| `globalStaticMinKF` | `static_map_filter.min_keyframes` |
| `globalStaticFilterCorners` | `static_map_filter.filter_corners` |
| `z_tollerance` | `motion_constraint.z_tolerance` |
| `rotation_tollerance` | `motion_constraint.rotation_tolerance` |

### Local map, loop closure, and visualization

| Legacy/current name | Canonical name |
|---|---|
| `surroundingkeyframeAddingDistThreshold` | `local_map.keyframe_distance_threshold` |
| `surroundingkeyframeAddingAngleThreshold` | `local_map.keyframe_angle_threshold` |
| `surroundingKeyframeDensity` | `local_map.keyframe_density` |
| `surroundingKeyframeSearchRadius` | `local_map.search_radius` |
| `loopClosureEnableFlag`, `system.enable_loopclosure` | `loop_closure.enable` (dual reader — see note) |
| `loopClosureFrequency` | `loop_closure.frequency` |
| `surroundingKeyframeSize` | `loop_closure.surrounding_keyframe_size` |
| `historyKeyframeSearchRadius` | `loop_closure.history_search_radius` |
| `historyKeyframeSearchTimeDiff` | `loop_closure.history_search_time_difference` |
| `historyKeyframeSearchNum` | `loop_closure.history_search_count` |
| `historyKeyframeFitnessScore` | `loop_closure.fitness_threshold` |
| `loopclosure.*` | `loop_closure.*` with unchanged snake_case leaf names |
| `globalMapVisualizationSearchRadius` | `visualization.global_map_search_radius` |
| `globalMapVisualizationPoseDensity` | `visualization.global_map_pose_density` |
| `globalMapVisualizationLeafSize` | `visualization.global_map_leaf_size` |
| `system.enable_visualize_global_map` | `visualization.enable_global_map` |
| `system.global_map_visualization_resolution` | `visualization.global_map_resolution` |

Existing `localization.*`, `frontend.*`, and `output.*` leaves that are already
snake_case remain unchanged unless explicitly mapped above. Existing
`system.keyframe_delta_*` and `system.tile_map_grid_size` also remain unchanged.

### Dual-reader parameters (two nodes, two edit sites)

Several canonical names unify a legacy pair where **each name is currently read
by a different node**. Consolidating them means both C++ read sites must be
retargeted to the canonical name, and both YAML files that a given node loads
must carry the canonical key. These are not single-reader renames:

| Canonical name | Mapping reader (legacy) | Relocalization reader (legacy) |
|---|---|---|
| `loop_closure.enable` | `loopClosureEnableFlag` (`utility.hpp:437`) | `system.enable_loopclosure` (`system.cpp:245`) |
| `imu.gravity` | `imuGravity` (`utility.hpp:364`, default 9.80511) | `gravity` (`system.cpp:171`, top-level) |

For `imu.gravity`, note the relocalization value is currently a **top-level**
`gravity` key in `relocalization.yaml`; migrating it into the `imu` group is a
structural move, not just a rename. Preserve each node's existing default
(mapping 9.80511, relocalization 9.805) unless the user unifies them.

### Single-node parameters that look shared

Some canonical names appear in the "Common" tables but are in fact read by only
one node today. Do not add a mirror reader to the other node during migration:

- `frames.imu` is read only by relocalization (`system.cpp:90`); mapping
  (`utility.hpp`) never declares an IMU frame. The "Scope" requirement to touch
  every `utility.hpp` declaration does not imply adding one here.

### Frontend group boundary (relocalization only)

The `frontend.*` group is already snake_case and is intentionally **excluded
from renaming** in this migration, but it is the single largest parameter group
(~30 leaves: `frontend.fusion_*`, `frontend.feature.*`,
`frontend.registration.*`, `frontend.registration_and_searcher_mode`). Because
the stated goal is "one unified schema," this group's boundary is declared here
explicitly: it is owned by relocalization (`system.cpp`), it is not part of the
camelCase cutover, and no mapping node reads it. Listing every leaf is out of
scope; the group is normalized by assertion, not by enumeration. A future change
may fold it into the shared schema, but this migration leaves it byte-for-byte
unchanged.

## YAML Rules

- Both YAML files use nested mappings; ROS2 exposes them as dotted parameter
  names.
- Shared hardware, topic, frame, IMU, lidar, and calibration keys use identical
  names and data shapes in both files.
- Mode-specific sections may be absent from the other file.
- Existing configured values are preserved, including the user's
  `localization.sc_top_n: 6`.
- No camelCase parameter key remains in either YAML.

## Failure and Compatibility Policy

This is intentionally backward incompatible. The nodes declare only canonical
names. A legacy YAML therefore cannot silently override a canonical default.
Array lengths for calibration are validated before constructing Eigen maps; an
invalid translation or matrix produces a clear startup error rather than
undefined behavior.

## Verification

1. A parameter-schema test asserts canonical names in both YAML files and C++
   readers and rejects every legacy parameter string.
2. YAML parsing verifies both files are structurally valid.
3. Build all mapping and relocalization executables that include
   `utility.hpp`, `imageProjection.cpp`, or `system.cpp`.
4. Launch-smoke mapping and relocalization configurations long enough to catch
   undeclared, wrong-type, and missing-parameter errors.
5. Confirm `ros2 param list` exposes only canonical project parameters.
6. Dual-reader retargeting: assert both `loop_closure.enable` read sites
   (`utility.hpp`, `system.cpp`) and both `imu.gravity` read sites resolve to
   the canonical name, and that neither node still declares its legacy key.
7. Calibration direction: with a deliberately non-identity
   `calibration.rotation`, assert mapping's `extRot` equals the transpose of the
   rotation block relocalization loads into `calibration_lidar_to_imu_`. This
   guards the imu↔lidar direction convention that identity defaults would
   otherwise hide.
