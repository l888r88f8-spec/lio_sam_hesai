# GroundMap single-layer grid export

`map_save.ground_grid.enable` (default `false`; enabled in
`config/mapping.yaml`) changes only the final GroundMap artifact written by
`/lio_sam/save_map_options`. Matching inputs, original keyframes, GlobalMap,
RViz and the legacy export path are unchanged.

When enabled, the resolved `map_save.ground_resolution` (or a positive service
override) becomes the final XY cell size. The processor:

- groups raw measurements into fixed world-origin cells using `floor`;
- selects one robust height layer per cell and leaves equal competing layers
  empty instead of averaging them;
- writes one finite point at each exact cell center, ordered by `(ix, iy)`;
- estimates center Z from a local plane when supported, otherwise uses the
  selected layer mean, preserving slopes;
- fills an entire enclosed four-connected unobserved component only when its
  total cell count is at most `map_save.ground_grid.max_hole_cells` (default
  14 and enabled by the same `ring_plane.grid_patch.enable` switch). Larger
  components, open boundaries, components touching a rejected observed cell,
  and height discontinuities stay completely empty; a large hole is never
  filled in pieces;
- validates the whole component against all original accepted boundary cells
  and each hole cell's radius support before committing any prediction, so
  newly filled points are never fitting evidence and a step is never bridged.

The hole limit counts connected cells, not the component's bounding-box width
or height, and it is set in the mapping configuration: the default of 14 fills
holes of up to 14 connected cells, while setting it to 1 fills only single-cell
holes. `map_save.ground_grid.max_hole_cells` must be an integer of at least 1.
The original GroundMap PCD is not rewritten automatically; the rule applies
only when a new final GroundMap is exported.

The ordinary 3D voxel stage is skipped in this mode. `align_ground_to_zero`
and `ring_plane.flatten_z` are still applied afterward, so leaving
`flatten_z: false` retains the real ramp height. Enabling the mode with a
resolved resolution of zero fails the save with a clear error before any
previous artifact is replaced.

The mapping configuration currently enables the mode with a 0.1 m spacing:

```yaml
map_save:
  ground_resolution: 0.1
  ground_grid:
    enable: true
    max_hole_cells: 14
ring_plane:
  flatten_z: false
```

Restart the mapping launch after editing these settings. Call
`/lio_sam/save_map_options` with `save_ground_map: true` and
`ground_resolution: -1.0` to inherit the configured spacing, or supply a
positive value for that save. Setting `ground_grid.enable: false` restores
legacy ground export. Synthetic grid points have intensity zero; the original
scan intensity is not resampled.

## Ground scan mesh (trial)

`map_save.ground_mesh` is an optional pre-grid stage for
`Current`/`RingPlane` export. It rasterizes constrained triangles between
adjacent scan rows and appends only global grid cells that are absent from the
measured ground points; existing measurements and the final single-layer /
hole-fill behavior are unchanged.

```yaml
map_save:
  ground_mesh:
    enable: true                 # trial default in config/mapping.yaml
    max_edge_length: 0.5         # base edge threshold [m]
    edge_length_per_meter: 0.25  # distance-adaptive edge limit [m/m]
    max_range_ratio: 2.0         # adjacent range-discontinuity ratio, >= 1
```

The C++ defaults keep the stage off. Enabling it requires a positive resolved
`ground_resolution` (including when `ground_grid.enable` is false) and the
`Current`/`RingPlane` projection cache. Unsupported modes, missing projection
metadata, or a non-positive resolution fail before prior map artifacts are
replaced. When a save request has `save_ground_map: false`, mesh mode and
resolution limits are not applied. Mesh slope and height-layer spans reuse
`ring_plane.grid_patch.max_slope_deg` and `max_height_range`.

Each triangle edge must be no longer than `max_edge_length +
edge_length_per_meter * min(endpoint sensor ranges)`. Only classification 1
vertices inside the configured output-ring range are used. World-frame slope
and range-ratio checks reject discontinuities; competing height layers are
left empty. Every cell gets at most one vote per frame. If static persistence
filtering is active, mesh samples also need its minimum keyframe support.

Rasterization stops with an error before prior artifacts are replaced if a
triangle bounding box exceeds one million cells, a frame exceeds two million
sample cells, or the accumulated mesh exceeds four million cells. Increase
the ground spacing for such exports; spacing is never silently changed.
