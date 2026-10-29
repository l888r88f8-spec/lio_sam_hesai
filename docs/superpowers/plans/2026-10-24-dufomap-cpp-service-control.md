# DUFOMap Native C++ Service Control Implementation Plan

## Goal

Replace the save-path Python DUFOMap subprocess with a directly linked C++20
adapter over the pinned official UFOMap headers, while adding per-request
`dufomap_mode` control to both save services and a cache-preparation switch.

## Constraints

- Pin DUFOMap `9e239ddd5995136e14f5212f33382a6ebc59e518` and UFOMap
  `fcb20f445d257280c039c07ab756c9d5b37ebd78`.
- Vendor only the official headers needed by
  `ufo/map/ufomap.hpp` + `ufo/map/integration/integration.hpp`, plus licenses
  and source metadata. Do not edit vendored headers.
- Keep the project at C++17; the native backend target alone is C++20. Hide all
  UFOMap types behind PIMPL.
- Never import Python, pybind11, or NumPy in the production save path.
- Read cached scans one at a time. Keep original target point coordinates and
  intensity; DUFOMap labels never merge points.
- Reject `num_threads == 1` (upstream serial miss TODO); allow `d_p` 0..4.
- `timeout_seconds` is cooperative: checked between frames and classification
  batches, never inside one upstream integration call.
- Preserve all pre-existing uncommitted work, including the deleted launch
  documentation and user-edited `mapping.yaml` values/blank lines.

## Steps

1. Add failing contract tests for mode parsing, effective-on cache rules,
   service fields, controller forwarding, and the native backend's real
   static/moving/unknown, range, validation, and timeout behavior.
2. Vendor the fixed official headers and add a standalone C++20 PIMPL target.
3. Implement the native backend and cooperative timeouts.
4. Add `dufomap_mode` to `SaveMap.srv` and `SaveMapOptions.srv`; centralize
   resolution in `save_map_request`; pass it through both callbacks and the
   RViz controller.
5. Add `dufomap_export.cache_scans`; prepare the cache for
   `enable || cache_scans || ground-export`, but clean up only after a
   successful enabled export.
6. Rewire `ExportMap` to resolve the effective mode before touching files,
   validate `static_map_filter` against that mode, verify scan completeness,
   run one native backend instance, and defer old-artifact removal until the
   backend succeeds.
7. Keep legacy Python worker/installer as manual replay tools; mark their
   config parameters unused by production and stop requiring Python at
   startup.
8. Build, run focused and full CTest, install, and verify installed service
   serialization, native linkage, licenses, and absence of Python runtime
   dependencies.

## Validation

- Real native synthetic wall/moving/unknown scenes and range/error/timeout
  cases.
- Service/config/on/off/invalid/cache-preparation/missing-cache integration
  tests against the real C++ backend.
- Existing ground/RingPlane and save-service compatibility tests remain green.
- Installed `mapOptimization` imports no Python module for export; installed
  headers/licenses are present.
