# Single-Layer Ground Map Grid Plan

**Goal:** Export GroundMap as a slope-preserving XY grid with at most one point per cell, without changing scans, feature matching, RViz or GlobalMap.

**Design:** Add mapping-only `map_save.ground_grid.enable` (default false for compatibility; true in mapping.yaml). Reuse the resolved service/config `ground_resolution` as cell size. This export mode replaces the existing append-plus-3D-voxel stage. Use fixed world XY cells, robust height-layer selection and local plane estimates at exact cell centers; retain unsupported/ambiguous gaps. Apply existing vertical offset and explicit flatten options afterward. Keep the legacy export when disabled.

- [x] Implement a small reusable ground-grid processor, with tests covering centers/unique XY, jittered duplicate frames, slopes, outliers/separate height layers, negative coordinates, interior holes, open boundaries and non-finite input.
- [x] Add the mapping parameter and wire both online-ground and export-ground inputs through the same final GroundMap export stage; retain legacy and no-ground-save behavior. Enabled gridding requires positive resolved ground_resolution, with a clear save error for zero.
- [x] Add actual export tests for slope preservation, cell-center alignment and uniqueness, resolution override and legacy behavior; update parameter-schema coverage and brief docs.
- [x] Root reviews, builds and installs with the host ROS toolchain using the existing build directory and limited parallelism; test in an isolated ROS domain, remove only this task's temporary artifacts. No user ROS process management, rosbag replay, commits or unrelated changes.

## Review notes

- Added regressions for asymmetric scan sampling, true slopes spanning absolute height bands, narrow raised ground, coarse-grid slope boundaries, bridged competing layers, and holes beside a height discontinuity. Each identified defect was reproduced before its fix.
- Measured cells accept center predictions only when the plane agrees at their actual representative XY; uncertain neighboring layers never replace the measured layer. Genuine cell-local ramps are checked before absolute height-band selection.
- Filling remains limited to unobserved one-cell interior gaps with four immediately adjacent observed cells agreeing with the plane. No boundary growth, iterative hole growth, or refill of rejected measured cells.
- Wide grid indices and a physical spatial bucket index handle tiny resolutions without enumerating huge empty grids. Store input indices temporarily, then release them and the layer buffers. Fill candidates are visited once from their observed left neighbor, without a separate candidate map or arbitrary map-size cutoff.
- Root confirmed the seven core initialization/optimization/pose-correction function bodies match their recorded baseline (normalizing the previously approved transformed-cache Clear rename).

## Validation

- Host ROS Humble toolchain, existing build directory, serial build: all four production mapping executables rebuilt; no runtime nodes started or stopped.
- 115 related tests passed: processor 25, actual save handler/export 32, legacy ground patcher 2, configuration 35, parameter schema 13, transformed cache 8. Final processor and export tests ran again against the final CMake binaries in ROS domain 88.
- New processor/header/tests passed ament_cpplint with the repository's existing copyright/header-guard/include-order/C++11 exemptions. `git diff --check` passed.
- Synthetic slope, layering, hole and real export-path checks are covered; no complete rosbag replay or field validation was performed.
- Installation verified: SHA-256 of all four installed production binaries matches the build output; installed configuration and new public header match source. Removed this task's Harness scratch directory, root verification directory, and retained temporary caches from both export-test runs.
