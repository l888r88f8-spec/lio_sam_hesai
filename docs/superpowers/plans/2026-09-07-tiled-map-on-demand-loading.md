# Tiled Map On-Demand Loading Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a backward-compatible tiled-map mode that supports automatic relocalization and tracking without keeping the complete global and ground maps resident in memory.

**Architecture:** Introduce a `localization_map` module containing a shared tile index, a validated YAML manifest, an atomic tile generator, a bounded cache, and `GlobalMapProvider`/`TiledMapProvider` implementations behind one interface. `Localization` consumes immutable provider snapshots; tiled I/O runs on one worker, candidate initialization may wait with a timeout, and tracking only polls completed snapshots at frame-safe points.

**Tech Stack:** C++17, PCL, yaml-cpp, Eigen, ROS 2 Humble/rclcpp, ament_cmake_gtest, Python 3 integration harness, rosbag2.

**Design:** `docs/superpowers/specs/2026-09-07-tiled-map-on-demand-loading-design.md`

---

## File map

New production files:

- `include/localization_map/tile_index.h`: stable XY tile key and coordinate conversion.
- `include/localization_map/tile_manifest.h`: manifest data model and load/save contract.
- `src/localization_map/tile_manifest.cpp`: YAML parsing, path containment, file-size validation.
- `include/localization_map/tile_map_generator.h`: generator options and result types.
- `src/localization_map/tile_map_generator.cpp`: deterministic PCD splitting, overview generation, atomic directory publication.
- `src/generate_tiled_map.cpp`: offline conversion CLI for existing map directories.
- `include/localization_map/tile_cache.h`: bounded logical-tile LRU cache.
- `src/localization_map/tile_cache.cpp`: pinning, two-budget eviction, counters.
- `include/localization_map/map_provider.h`: immutable snapshot, diagnostics, request result, provider interface.
- `include/localization_map/global_map_provider.h` and `src/localization_map/global_map_provider.cpp`: current full-map behavior behind the interface.
- `include/localization_map/tiled_map_provider.h` and `src/localization_map/tiled_map_provider.cpp`: worker queue, tile loading, candidate waits, tracking polling and ground-map snapshots.

New tests:

- `test/test_tile_index.cpp`
- `test/test_tile_manifest.cpp`
- `test/test_tile_map_generator.cpp`
- `test/test_tile_cache.cpp`
- `test/test_global_map_provider.cpp`
- `test/test_tiled_map_provider.cpp`
- `test/test_tiled_map_localization_surface.cpp`
- `test/test_tiled_map_integration.py`

Modified integration points:

- `include/slam/config_parameters.h`, `src/slam/system.cpp`, `config/relocalization.yaml`: localization mode and cache settings.
- `include/lio_sam/utility.hpp`, `config/mapping.yaml`, `src/mapOptimization.cpp`: optional tiled export settings and invocation.
- `include/slam/localization.h`, `src/slam/localization.cpp`: replace direct global/tile ownership with the provider.
- `include/slam/system.h`, `src/slam/system.cpp`: map diagnostics and overview/local-map publication.
- `test/integration/stability_common.py`, `test/integration/localization_regression.py`, `scripts/run_localization_stability.sh`: tiled runtime parameters and supported replay mode.
- `CMakeLists.txt`, `package.xml`: yaml-cpp, production sources, CLI, tests and install rules.
- `docs/HANDOFF.md`: final commands, measurements and behavior.

## Task 1: Shared tile index and configuration contract

**Files:**
- Create: `include/localization_map/tile_index.h`
- Create: `test/test_tile_index.cpp`
- Modify: `include/slam/config_parameters.h`
- Modify: `src/slam/system.cpp`
- Modify: `config/relocalization.yaml`
- Modify: `include/lio_sam/utility.hpp`
- Modify: `config/mapping.yaml`
- Modify: `test/test_unified_parameter_schema.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing tile-index tests**

Create `test/test_tile_index.cpp` with exact boundary coverage:

```cpp
#include <gtest/gtest.h>

#include "localization_map/tile_index.h"

using localization_map::TileIndex;
using localization_map::TileIndexForPosition;

TEST(TileIndex, UsesExplicitOriginAndFloorForPositiveCoordinates) {
  EXPECT_EQ(TileIndexForPosition(0.0, 0.0, 100.0, 0.0, 0.0),
            (TileIndex{0, 0}));
  EXPECT_EQ(TileIndexForPosition(99.999, 100.0, 100.0, 0.0, 0.0),
            (TileIndex{0, 1}));
}

TEST(TileIndex, UsesFloorForNegativeCoordinates) {
  EXPECT_EQ(TileIndexForPosition(-0.001, -100.001, 100.0, 0.0, 0.0),
            (TileIndex{-1, -2}));
}

TEST(TileIndex, AppliesNonZeroOrigin) {
  EXPECT_EQ(TileIndexForPosition(49.0, -51.0, 100.0, 50.0, -50.0),
            (TileIndex{-1, -1}));
}

TEST(TileIndex, GeneratesDeterministicSquareNeighborhood) {
  const auto indices = localization_map::SquareNeighborhood({2, -3}, 1);
  ASSERT_EQ(indices.size(), 9U);
  EXPECT_EQ(indices.front(), (TileIndex{1, -4}));
  EXPECT_EQ(indices.back(), (TileIndex{3, -2}));
}
```

- [ ] **Step 2: Register the test and prove RED**

Add this target inside `BUILD_TESTING`:

```cmake
ament_add_gtest(test_tile_index test/test_tile_index.cpp)
target_include_directories(test_tile_index PRIVATE include)
```

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target test_tile_index -- -j1
```

Expected: compilation fails because `localization_map/tile_index.h` does not exist.

- [ ] **Step 3: Implement the shared index type**

Create a header-only value type with this public surface:

```cpp
#ifndef LOCALIZATION_MAP__TILE_INDEX_H_
#define LOCALIZATION_MAP__TILE_INDEX_H_

#include <cmath>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace localization_map {

struct TileIndex {
  int x{0};
  int y{0};

  friend bool operator==(const TileIndex& lhs, const TileIndex& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y;
  }
  friend bool operator<(const TileIndex& lhs, const TileIndex& rhs) {
    return lhs.x < rhs.x || (lhs.x == rhs.x && lhs.y < rhs.y);
  }
};

struct TileIndexHash {
  std::size_t operator()(const TileIndex& index) const noexcept {
    const auto x = std::hash<int>{}(index.x);
    const auto y = std::hash<int>{}(index.y);
    return x ^ (y + 0x9e3779b9U + (x << 6U) + (x >> 2U));
  }
};

inline TileIndex TileIndexForPosition(double x, double y, double tile_size_m,
                                      double origin_x, double origin_y) {
  if (!std::isfinite(tile_size_m) || tile_size_m <= 0.0) {
    throw std::invalid_argument("tile_size_m must be finite and positive");
  }
  return {static_cast<int>(std::floor((x - origin_x) / tile_size_m)),
          static_cast<int>(std::floor((y - origin_y) / tile_size_m))};
}

inline std::vector<TileIndex> SquareNeighborhood(const TileIndex& center,
                                                 int radius) {
  if (radius < 0) {
    throw std::invalid_argument("tile radius must be non-negative");
  }
  std::vector<TileIndex> result;
  result.reserve(static_cast<std::size_t>((2 * radius + 1) *
                                          (2 * radius + 1)));
  for (int x = center.x - radius; x <= center.x + radius; ++x) {
    for (int y = center.y - radius; y <= center.y + radius; ++y) {
      result.push_back({x, y});
    }
  }
  return result;
}

inline std::string TileFileStem(const TileIndex& index) {
  return std::to_string(index.x) + "_" + std::to_string(index.y);
}

}  // namespace localization_map

#endif  // LOCALIZATION_MAP__TILE_INDEX_H_
```

- [ ] **Step 4: Add exact parameter members and readers**

Add these defaults to `ConfigParameters`:

```cpp
std::string localization_map_mode_{"auto"};
int localization_tiled_active_radius_{1};
int localization_tiled_prefetch_radius_{2};
int localization_tiled_max_cached_tiles_{25};
int localization_tiled_max_cache_mib_{1024};
int localization_tiled_load_timeout_ms_{1000};
```

Read the same fields in `System::InitConfigParameters()` under these exact ROS names:

```cpp
util::param(this, "localization.map_mode", config.localization_map_mode_,
            std::string("auto"));
util::param(this, "localization.tiled_map.active_radius_tiles",
            config.localization_tiled_active_radius_, 1);
util::param(this, "localization.tiled_map.prefetch_radius_tiles",
            config.localization_tiled_prefetch_radius_, 2);
util::param(this, "localization.tiled_map.max_cached_tiles",
            config.localization_tiled_max_cached_tiles_, 25);
util::param(this, "localization.tiled_map.max_cache_mib",
            config.localization_tiled_max_cache_mib_, 1024);
util::param(this, "localization.tiled_map.load_timeout_ms",
            config.localization_tiled_load_timeout_ms_, 1000);
```

Add these `ParamServer` members and declare/get calls:

```cpp
bool saveTiledMap;
double tiledMapSizeMeters;
std::string tiledMapOutputSubdirectory;
double tiledMapOverviewResolutionMeters;
```

```cpp
declare_parameter("map_save.save_tiled_map", false);
get_parameter("map_save.save_tiled_map", saveTiledMap);
declare_parameter("map_save.tiled_map.tile_size_m", 100.0);
get_parameter("map_save.tiled_map.tile_size_m", tiledMapSizeMeters);
declare_parameter("map_save.tiled_map.output_subdirectory", "tiled");
get_parameter("map_save.tiled_map.output_subdirectory",
              tiledMapOutputSubdirectory);
declare_parameter("map_save.tiled_map.overview_resolution_m", 5.0);
get_parameter("map_save.tiled_map.overview_resolution_m",
              tiledMapOverviewResolutionMeters);
```

Mirror these keys and defaults in both YAML files. Extend
`test_unified_parameter_schema.cpp` to assert all exact keys and to assert
`save_tiled_map: false`.

- [ ] **Step 5: Validate parameters before map preparation**

At the end of `System::InitConfigParameters()`, reject invalid values with
`std::invalid_argument`. Use the exact conditions:

```cpp
const auto& mode = config.localization_map_mode_;
if (mode != "auto" && mode != "global" && mode != "tiled") {
  throw std::invalid_argument(
      "localization.map_mode must be auto, global, or tiled");
}
const int active_width = 2 * config.localization_tiled_active_radius_ + 1;
const int minimum_cached_tiles = active_width * active_width;
if (config.localization_tiled_active_radius_ < 0 ||
    config.localization_tiled_prefetch_radius_ <
        config.localization_tiled_active_radius_ ||
    config.localization_tiled_max_cached_tiles_ < minimum_cached_tiles ||
    config.localization_tiled_max_cache_mib_ <= 0 ||
    config.localization_tiled_load_timeout_ms_ <= 0) {
  throw std::invalid_argument("invalid localization.tiled_map configuration");
}
```

- [ ] **Step 6: Run focused tests and commit**

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target \
  test_tile_index test_unified_parameter_schema -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  -R '^(test_tile_index|test_unified_parameter_schema)$' --output-on-failure
```

Expected: 2/2 tests pass.

Commit:

```bash
git add CMakeLists.txt include/localization_map/tile_index.h \
  include/slam/config_parameters.h include/lio_sam/utility.hpp \
  src/slam/system.cpp config/mapping.yaml config/relocalization.yaml
git add test/test_unified_parameter_schema.cpp
git add -f test/test_tile_index.cpp
git commit -m "feat(map): define tiled map coordinates and parameters"
```

## Task 2: Version-one manifest with safe path validation

**Files:**
- Create: `include/localization_map/tile_manifest.h`
- Create: `src/localization_map/tile_manifest.cpp`
- Create: `test/test_tile_manifest.cpp`
- Modify: `CMakeLists.txt`
- Modify: `package.xml`

- [ ] **Step 1: Write manifest parsing tests**

Use a temporary directory and cover a valid manifest, unknown schema,
duplicate index, `../` path escape, missing tile, declared byte mismatch, invalid
point type and invalid tile size. The valid-case assertion must be:

```cpp
const auto manifest = localization_map::TileManifest::Load(root);
EXPECT_EQ(manifest.schema_version, 1);
EXPECT_EQ(manifest.frame_id, "map");
EXPECT_DOUBLE_EQ(manifest.tile_size_m, 100.0);
EXPECT_EQ(manifest.origin, (std::array<double, 2>{0.0, 0.0}));
ASSERT_EQ(manifest.global_tiles.size(), 1U);
EXPECT_EQ(manifest.global_tiles.at({-1, 0}).point_count, 2U);
EXPECT_EQ(manifest.overview->path, "overview.pcd");
```

Each rejection must assert a stable message fragment:

```cpp
EXPECT_THROW_WITH_MESSAGE(load(), "unsupported schema_version");
EXPECT_THROW_WITH_MESSAGE(load(), "duplicate global tile index");
EXPECT_THROW_WITH_MESSAGE(load(), "escapes tiled map root");
EXPECT_THROW_WITH_MESSAGE(load(), "tile file is missing");
EXPECT_THROW_WITH_MESSAGE(load(), "tile byte size mismatch");
```

Implement `EXPECT_THROW_WITH_MESSAGE` locally as a helper that catches
`std::runtime_error` and checks `what()` with `std::string::find`.

- [ ] **Step 2: Add yaml-cpp and prove RED**

Add:

```cmake
find_package(yaml-cpp REQUIRED)
target_sources(${PROJECT_NAME}_relocalization_lib PRIVATE
  src/localization_map/tile_manifest.cpp)
target_link_libraries(${PROJECT_NAME}_relocalization_lib yaml-cpp)
```

Add `<depend>yaml-cpp</depend>` to `package.xml`. Register
`test_tile_manifest` with `src/localization_map/tile_manifest.cpp`, PCL and
`yaml-cpp`; do not link that test to the production library, so the parser stays
isolated. Build the target and expect failure because the manifest header is absent.

- [ ] **Step 3: Define the manifest model**

Expose these exact types:

```cpp
namespace localization_map {

struct TileFileMetadata {
  std::filesystem::path path;
  std::uint64_t point_count{0};
  std::uintmax_t bytes{0};
};

struct OverviewMetadata {
  std::filesystem::path path;
  double resolution_m{0.0};
};

struct SourceMetadata {
  std::string global_map_name;
  std::uintmax_t global_map_bytes{0};
  std::string ground_map_name;
  std::uintmax_t ground_map_bytes{0};
};

struct TileManifest {
  static constexpr int kSchemaVersion = 1;

  int schema_version{kSchemaVersion};
  std::string frame_id{"map"};
  std::string point_type{"XYZI"};
  double tile_size_m{100.0};
  std::array<double, 2> origin{0.0, 0.0};
  SourceMetadata source;
  std::map<TileIndex, TileFileMetadata> global_tiles;
  std::map<TileIndex, TileFileMetadata> ground_tiles;
  std::optional<OverviewMetadata> overview;

  static TileManifest Load(const std::filesystem::path& root);
  void Save(const std::filesystem::path& root) const;
};

}  // namespace localization_map
```

- [ ] **Step 4: Implement deterministic load/save and containment checks**

Implement the parser with explicit key reads; do not deserialize arbitrary
types. Resolve every tile and overview path using this containment rule:

```cpp
std::filesystem::path ResolveContainedPath(
    const std::filesystem::path& root,
    const std::filesystem::path& relative) {
  if (relative.empty() || relative.is_absolute()) {
    throw std::runtime_error("manifest path must be non-empty and relative");
  }
  const auto canonical_root = std::filesystem::weakly_canonical(root);
  const auto canonical_path =
      std::filesystem::weakly_canonical(canonical_root / relative);
  const auto mismatch = std::mismatch(canonical_root.begin(),
                                      canonical_root.end(),
                                      canonical_path.begin(),
                                      canonical_path.end());
  if (mismatch.first != canonical_root.end()) {
    throw std::runtime_error("manifest path escapes tiled map root");
  }
  return canonical_path;
}
```

Load-time validation order is schema, scalar values, duplicate indices, path
containment, existence, regular-file status and declared byte size. Point count
is validated after PCD decoding in the loader, so startup does not decode every
tile. `Save()` sorts map entries naturally through `std::map` and emits all
required keys in the order shown in the design.

- [ ] **Step 5: Run focused tests and commit**

Run the target and test:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target test_tile_manifest -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  -R '^test_tile_manifest$' --output-on-failure
```

Expected: 1/1 passes.

Commit the header, implementation, test, CMake and package manifest as:

```bash
git add CMakeLists.txt package.xml include/localization_map/tile_manifest.h \
  src/localization_map/tile_manifest.cpp
git add -f test/test_tile_manifest.cpp
git commit -m "feat(map): validate tiled map manifests"
```

## Task 3: Deterministic atomic map generator and CLI

**Files:**
- Create: `include/localization_map/tile_map_generator.h`
- Create: `src/localization_map/tile_map_generator.cpp`
- Create: `src/generate_tiled_map.cpp`
- Create: `test/test_tile_map_generator.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write generator tests**

Construct a six-point global cloud spanning positive, negative and exact
boundary coordinates plus a three-point ground cloud. Save them as
`GlobalMap.pcd` and `GroundMap.pcd` under a temporary root. Assert:

```cpp
localization_map::TileMapGeneratorOptions options;
options.map_directory = root;
options.output_subdirectory = "tiled";
options.tile_size_m = 100.0;
options.origin = {0.0, 0.0};
options.overview_resolution_m = 5.0;

const auto result = localization_map::GenerateTileMap(options);
EXPECT_EQ(result.global_input_points, 6U);
EXPECT_EQ(result.global_output_points, 6U);
EXPECT_EQ(result.ground_input_points, 3U);
EXPECT_EQ(result.ground_output_points, 3U);
EXPECT_TRUE(std::filesystem::exists(root / "tiled/manifest.yaml"));
EXPECT_FALSE(std::filesystem::exists(root / "tiled.incomplete"));
```

Reload every manifest tile and assert each point maps back to that tile index.
Generate into a second root and assert the two normalized manifests are equal.
Pre-create `tiled/manifest.yaml` and assert generation refuses overwrite.
Inject an invalid input path and assert neither final nor incomplete output is
left behind.

- [ ] **Step 2: Define generator API and prove RED**

Expose:

```cpp
struct TileMapGeneratorOptions {
  std::filesystem::path map_directory;
  std::filesystem::path output_subdirectory{"tiled"};
  double tile_size_m{100.0};
  std::array<double, 2> origin{0.0, 0.0};
  double overview_resolution_m{5.0};
  std::string frame_id{"map"};
};

struct TileMapGeneratorResult {
  std::uint64_t global_input_points{0};
  std::uint64_t global_output_points{0};
  std::uint64_t ground_input_points{0};
  std::uint64_t ground_output_points{0};
  std::size_t tile_count{0};
  std::filesystem::path output_directory;
};

TileMapGeneratorResult GenerateTileMap(
    const TileMapGeneratorOptions& options);
```

Register and build the generator test. Expected RED is an undefined
`GenerateTileMap` symbol.

- [ ] **Step 3: Implement split, overview and atomic publication**

Implementation sequence is fixed:

1. Validate numeric options and ensure the output subdirectory is a single
   relative path below `map_directory`.
2. Refuse when the final output exists.
3. Remove only a same-root stale `<output>.incomplete` directory after verifying
   its resolved parent equals the map root.
4. Load `GlobalMap.pcd`; load `GroundMap.pcd` when present.
5. Partition into `std::map<TileIndex, PCLPointCloudXYZI>` using
   `TileIndexForPosition()`.
6. Save sorted binary PCDs under `.incomplete/global` and
   `.incomplete/ground`; populate metadata from saved files.
7. Voxel-filter the global cloud at `overview_resolution_m` and save
   `overview.pcd`.
8. Assert input/output point counts match for both unfiltered tile sets.
9. Save and reload the manifest for validation.
10. Rename `.incomplete` to the final directory on the same filesystem.

Use an RAII cleanup guard that removes only the validated `.incomplete` path
on exceptions. Do not invoke shell commands for directory creation or removal.

- [ ] **Step 4: Add and test the offline CLI**

The executable accepts exactly:

```text
lio_sam_hesai_generate_tiled_map MAP_DIRECTORY
  [--output-subdirectory tiled]
  [--tile-size-m 100]
  [--origin-x 0]
  [--origin-y 0]
  [--overview-resolution-m 5]
  [--frame-id map]
```

Unknown flags, missing values and non-numeric values exit 64. Generation
failures print one line to stderr and exit 1. Success prints the final directory,
tile count and point-count totals and exits 0. Register and install the target:

```cmake
target_sources(${PROJECT_NAME}_relocalization_lib PRIVATE
  src/localization_map/tile_map_generator.cpp)
add_executable(${PROJECT_NAME}_generate_tiled_map
  src/generate_tiled_map.cpp)
target_link_libraries(${PROJECT_NAME}_generate_tiled_map
  ${PROJECT_NAME}_relocalization_lib)
install(TARGETS ${PROJECT_NAME}_generate_tiled_map
  DESTINATION lib/${PROJECT_NAME})
```

Run it against a temporary copy of the test fixture and verify a second run
fails without changing the first manifest.

- [ ] **Step 5: Run tests and commit**

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  -R '^(test_tile_index|test_tile_manifest|test_tile_map_generator)$' \
  --output-on-failure
```

Expected: 3/3 pass.

Commit:

```bash
git add CMakeLists.txt include/localization_map/tile_map_generator.h \
  src/localization_map/tile_map_generator.cpp src/generate_tiled_map.cpp
git add -f test/test_tile_map_generator.cpp
git commit -m "feat(map): generate deterministic tiled map bundles"
```

## Task 4: Bounded logical-tile LRU cache

**Files:**
- Create: `include/localization_map/tile_cache.h`
- Create: `src/localization_map/tile_cache.cpp`
- Create: `test/test_tile_cache.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write cache behavior tests**

Use small fake payloads with explicit byte estimates. Cover:

- hit and miss counters;
- one logical index holding global and ground clouds counts as one tile;
- LRU eviction order;
- pinned entries are never evicted;
- insertion fails when only pinned entries remain;
- both tile-count and byte budgets;
- replacing an entry updates bytes without corrupting LRU order;
- concurrent snapshots return internally consistent counters.

The budget rejection test must use this form:

```cpp
localization_map::TileCache cache({2U, 100U});
ASSERT_TRUE(cache.Insert({0, 0}, Payload(40U)));
cache.Pin({0, 0});
ASSERT_TRUE(cache.Insert({1, 0}, Payload(40U)));
cache.Pin({1, 0});
EXPECT_FALSE(cache.Insert({2, 0}, Payload(10U)));
const auto snapshot = cache.Snapshot();
EXPECT_EQ(snapshot.cached_tiles, 2U);
EXPECT_EQ(snapshot.bytes, 80U);
EXPECT_EQ(snapshot.budget_rejections, 1U);
```

- [ ] **Step 2: Define cache types and prove RED**

The public surface is:

```cpp
struct TilePayload {
  PCLPointCloudXYZI::ConstPtr global;
  PCLPointCloudXYZI::ConstPtr ground;
  std::size_t bytes{0};
};

struct TileCacheConfig {
  std::size_t max_tiles{25};
  std::size_t max_bytes{1024U * 1024U * 1024U};
};

struct TileCacheSnapshot {
  std::size_t cached_tiles{0};
  std::size_t bytes{0};
  std::uint64_t hits{0};
  std::uint64_t misses{0};
  std::uint64_t evictions{0};
  std::uint64_t budget_rejections{0};
};

class TileCache {
 public:
  explicit TileCache(TileCacheConfig config);
  std::shared_ptr<const TilePayload> Find(const TileIndex& index);
  bool Insert(const TileIndex& index, TilePayload payload);
  void Pin(const TileIndex& index);
  void Unpin(const TileIndex& index);
  void ClearPins();
  TileCacheSnapshot Snapshot() const;
};
```

Add the production source and isolated test target:

```cmake
target_sources(${PROJECT_NAME}_relocalization_lib PRIVATE
  src/localization_map/tile_cache.cpp)
ament_add_gtest(test_tile_cache
  test/test_tile_cache.cpp
  src/localization_map/tile_cache.cpp)
target_include_directories(test_tile_cache PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(test_tile_cache ${PCL_LIBRARIES})
```

Build the target and expect an undefined `TileCache` implementation.

- [ ] **Step 3: Implement eviction with one mutex**

Store entries in `std::unordered_map<TileIndex, Entry, TileIndexHash>` and an
LRU `std::list<TileIndex>`. Each entry stores its list iterator and pin count.
All public methods lock one private mutex. `Find()` moves a hit to the front.
`Insert()` computes the projected count/bytes, then repeatedly evicts from the
back while skipping pinned entries; if no unpinned entry can satisfy both
budgets, leave the cache unchanged and increment `budget_rejections`.

`TilePayload::bytes` must be calculated by the PCD loader as:

```cpp
payload.bytes =
    (payload.global ? payload.global->size() * sizeof(PCLPointXYZI) : 0U) +
    (payload.ground ? payload.ground->size() * sizeof(PCLPointXYZI) : 0U);
```

- [ ] **Step 4: Run focused tests and commit**

Run `test_tile_cache` 20 times to expose ordering/race mistakes:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target test_tile_cache -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  -R '^test_tile_cache$' --repeat until-fail:20 --output-on-failure
```

Expected: all 20 iterations pass.

Commit:

```bash
git add CMakeLists.txt include/localization_map/tile_cache.h \
  src/localization_map/tile_cache.cpp
git add -f test/test_tile_cache.cpp
git commit -m "feat(map): add bounded tiled map cache"
```

## Task 5: Map-provider interface and full-map compatibility adapter

**Files:**
- Create: `include/localization_map/map_provider.h`
- Create: `include/localization_map/global_map_provider.h`
- Create: `src/localization_map/global_map_provider.cpp`
- Create: `test/test_global_map_provider.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write full-map provider tests**

Create a temporary global/ground PCD fixture. Assert prepare success, overview
equals the global map, candidate maps are bounded to 50 meters in XYZ, tracking
snapshots retain the existing 100-meter half-extent and 50-meter refresh
threshold, ground prefers `GroundMap.pcd`, missing ground falls back to global,
and invalid global PCD fails preparation with a stable error.

- [ ] **Step 2: Define the provider contract**

Use these exact public types:

```cpp
enum class MapMode { kGlobal, kTiled };
enum class MapRequestState { kReady, kUnchanged, kPending, kUnavailable };

struct MapSnapshot {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PCLPointCloudXYZI::ConstPtr registration_cloud;
  PCLPointCloudXYZI::ConstPtr ground_cloud;
  std::vector<TileIndex> indices;
  std::vector<TileIndex> missing_indices;
  bool degraded{false};
};

struct MapRequestResult {
  MapRequestState state{MapRequestState::kUnavailable};
  std::shared_ptr<const MapSnapshot> snapshot;
  bool previous_snapshot_covers_center{false};
  std::string error;
};

struct MapDiagnostics {
  MapMode mode{MapMode::kGlobal};
  int manifest_version{0};
  std::size_t active_tiles{0};
  std::size_t cached_tiles{0};
  double cache_mib{0.0};
  std::uint64_t cache_hits{0};
  std::uint64_t cache_misses{0};
  std::uint64_t load_failures{0};
  double last_load_ms{0.0};
  std::string load_state{"idle"};
  std::string last_error;
};

class MapProvider {
 public:
  virtual ~MapProvider() = default;
  virtual bool Prepare(std::string* error) = 0;
  virtual MapRequestResult LoadCandidate(const Mat4d& map_lidar_pose,
                                         std::chrono::milliseconds timeout) = 0;
  virtual MapRequestResult RequestTracking(const Mat4d& map_lidar_pose,
                                           std::optional<Vec2d> velocity_xy,
                                           bool force_rebuild) = 0;
  virtual PCLPointCloudXYZI::ConstPtr Overview() const = 0;
  virtual MapDiagnostics Diagnostics() const = 0;
  virtual void Shutdown() = 0;
};
```

Add the production implementation and isolated provider test target:

```cmake
target_sources(${PROJECT_NAME}_relocalization_lib PRIVATE
  src/localization_map/global_map_provider.cpp)
ament_add_gtest(test_global_map_provider
  test/test_global_map_provider.cpp
  src/localization_map/global_map_provider.cpp)
target_include_directories(test_global_map_provider PRIVATE
  include ${PCL_INCLUDE_DIRS})
target_link_libraries(test_global_map_provider ${PCL_LIBRARIES})
```

Build the new test and expect missing provider symbols.

- [ ] **Step 3: Move current global behavior behind `GlobalMapProvider`**

Define `GlobalMapProviderConfig` with global path, optional ground path and
`global_voxel_size{0.3F}`. Move `LoadPcdAsXYZI`, full-map XY kd-tree, candidate
radius search, crop-box tracking logic and ground source selection out of
`Localization` without changing constants or mathematical conditions. Global
mode continues returning its loaded global cloud from `Overview()` regardless
of the tiled-only overview-loading optimization.

`RequestTracking()` returns:

- `kReady` with a new snapshot on first request, force rebuild or refresh;
- `kUnchanged` when the current crop remains valid;
- `kUnavailable` with an error if preparation failed.

`Shutdown()` is an idempotent no-op. `Diagnostics()` reports `kGlobal`, zero
tile/cache fields and a stable `ready/error` state.

- [ ] **Step 4: Run compatibility tests and commit**

Run:

```bash
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  -R '^(test_global_map_provider|test_ground_map_source|test_relocalization_efficiency_surface)$' \
  --output-on-failure
```

Expected: all selected tests pass.

Commit:

```bash
git add CMakeLists.txt include/localization_map/map_provider.h \
  include/localization_map/global_map_provider.h \
  src/localization_map/global_map_provider.cpp
git add -f test/test_global_map_provider.cpp
git commit -m "refactor(map): isolate full-map localization provider"
```

## Task 6: Asynchronous tiled provider

**Files:**
- Create: `include/localization_map/tiled_map_provider.h`
- Create: `src/localization_map/tiled_map_provider.cpp`
- Create: `test/test_tiled_map_provider.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write deterministic worker tests with an injected loader**

Inject `std::function<TileLoadResult(const TileIndex&)>` so tests never depend
on disk timing. Cover:

- preparation loads manifest and optional overview but no tile;
- first tracking request returns `kPending`, later poll returns `kReady`;
- no loader call runs on the requesting thread;
- duplicate candidate/tracking requests load each index once;
- candidate waits only to its timeout;
- center global failure returns `kUnavailable`;
- neighbor global failure returns a degraded ready snapshot with the missing
  index;
- ground failure leaves global data and marks degraded;
- priorities run candidate, active tracking, then prefetch;
- committing a new active snapshot pins it and unpins the previous set;
- cache remains within count and byte budgets;
- shutdown joins the worker and is idempotent.

For the nonblocking assertion, record `std::this_thread::get_id()` inside the
fake loader and compare it with the request thread ID.

- [ ] **Step 2: Define provider configuration and loader result**

Add:

```cpp
struct TiledMapProviderConfig {
  std::filesystem::path root;
  int active_radius{1};
  int prefetch_radius{2};
  std::size_t max_cached_tiles{25};
  std::size_t max_cache_bytes{1024U * 1024U * 1024U};
  bool load_overview{false};
  float registration_voxel_size{0.25F};
};

struct TileLoadResult {
  TileIndex index;
  TilePayload payload;
  bool global_present{false};
  bool global_ok{false};
  bool ground_present{false};
  bool ground_ok{false};
  std::string error;
};
```

The production constructor accepts config; the test constructor additionally
accepts the injected loader.

Add the production source and link its isolated test with yaml-cpp, PCL and
threads:

```cmake
target_sources(${PROJECT_NAME}_relocalization_lib PRIVATE
  src/localization_map/tiled_map_provider.cpp)
ament_add_gtest(test_tiled_map_provider
  test/test_tiled_map_provider.cpp
  src/localization_map/tile_manifest.cpp
  src/localization_map/tile_cache.cpp
  src/localization_map/tiled_map_provider.cpp)
target_include_directories(test_tiled_map_provider PRIVATE
  include ${PCL_INCLUDE_DIRS})
target_link_libraries(test_tiled_map_provider
  yaml-cpp ${PCL_LIBRARIES} Threads::Threads)
```

- [ ] **Step 3: Implement production tile decoding**

For each index, look up manifest metadata. A missing manifest entry means an
intentionally empty sparse tile. A declared file is loaded with the shared
XYZ/XYZI converter, checked against declared point count, converted to immutable
cloud pointers and counted in bytes. Return failures through `TileLoadResult`;
the worker must not throw across its thread boundary.

- [ ] **Step 4: Implement request queue and snapshots**

Maintain three FIFO queues with fixed priority: candidate, active tracking,
prefetch. A request has a monotonically increasing ID, center index, ordered
index set, completion state and condition variable. Per-index in-flight state
coalesces duplicates.

Candidate behavior:

```cpp
const auto request_id = Enqueue(center, active_indices, Priority::kCandidate);
if (!WaitUntilTerminal(request_id, timeout)) {
  return {MapRequestState::kUnavailable, nullptr, false,
          "candidate tile load timed out"};
}
return BuildResult(request_id);
```

Tracking behavior never waits:

```cpp
if (const auto ready = TakeCompletedTracking(center)) {
  EnqueuePrefetch(center, velocity_xy);
  return ready.value();
}
EnsureTrackingRequestQueued(center);
return {MapRequestState::kPending, nullptr,
        ActiveSnapshotContains(center), ""};
```

`BuildResult()` waits for every requested index to be
`ready/failed/not-present`, requires a valid center global tile, merges all
successful global clouds, merges successful ground clouds, and records failed
noncenter indices. Publish only a `std::shared_ptr<const MapSnapshot>`.
For `kReady`, set `previous_snapshot_covers_center` to true. For candidate
timeout, center failure and budget rejection, set it to false.

Within the prefetch queue, when `velocity_xy` has nonzero finite norm, sort
indices by descending dot product between `(tile_center - vehicle_position)`
and normalized velocity; use `TileIndex::operator<` as the deterministic tie
breaker. Without valid velocity, use tile-index order.

- [ ] **Step 5: Implement active pinning and bounded shutdown**

When `RequestTracking()` returns a new ready snapshot, clear previous pins and
pin all new snapshot indices before exposing it. Prefetch never pins. Shutdown
sets `stopping_`, clears queued prefetch work, notifies the worker and joins it.
The destructor calls `Shutdown()`.

- [ ] **Step 6: Run race-sensitive tests and commit**

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target test_tiled_map_provider -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  -R '^test_tiled_map_provider$' --repeat until-fail:50 \
  --output-on-failure
```

Expected: all 50 iterations pass without hang.

Commit:

```bash
git add CMakeLists.txt include/localization_map/tiled_map_provider.h \
  src/localization_map/tiled_map_provider.cpp
git add -f test/test_tiled_map_provider.cpp
git commit -m "feat(map): load tiled localization maps asynchronously"
```

## Task 7: Integrate providers into localization and Scan Context

**Files:**
- Modify: `include/slam/localization.h`
- Modify: `src/slam/localization.cpp`
- Create: `test/test_tiled_map_localization_surface.cpp`
- Modify: `test/test_relocalization_efficiency_surface.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing integration surface contract**

The source contract must require:

```cpp
ExpectToken("include/slam/localization.h",
            "std::unique_ptr<localization_map::MapProvider> map_provider_");
ExpectToken("src/slam/localization.cpp", "ResolveMapMode");
ExpectToken("src/slam/localization.cpp", "map_provider_->LoadCandidate");
ExpectToken("src/slam/localization.cpp", "map_provider_->RequestTracking");
ExpectToken("src/slam/localization.cpp", "MapRequestState::kPending");
ExpectNoToken("src/slam/localization.cpp",
              "Scan Context auto relocalization does not support tile maps");
ExpectNoToken("include/slam/localization.h", "bool use_tile_map_");
ExpectNoToken("include/slam/localization.h", "local_map_data_");
```

Register and run the test. Expected: it fails on all new provider tokens.

- [ ] **Step 2: Construct the selected provider**

Add `ResolveMapMode(mode, map_path)` with exact behavior:

- `global`: path must be a regular file;
- `tiled`: path must be a directory containing `manifest.yaml`;
- `auto`: regular file selects global, directory selects tiled, any other path
  is an error;
- a directory with an invalid manifest remains tiled and fails preparation; it
  never falls back to a sibling PCD.

Construct `GlobalMapProvider` with the existing derived ground-map path or
`TiledMapProvider` with cache settings. Store only
`std::unique_ptr<localization_map::MapProvider> map_provider_` in
`Localization`; delete the old tile maps, indices, booleans, full clouds and
kd-trees after migration.

- [ ] **Step 3: Route map preparation and Scan Context paths**

`PrepareMap()` calls provider preparation first. Derive default `sc_db.bin` as:

- global: sibling of the PCD;
- tiled: parent of the `tiled` directory, preserving the approved compatible
  layout.

Do not load full global/ground PCDs in `Localization`. Remove the tile-mode
Scan Context rejection.

- [ ] **Step 4: Route candidate and tracking maps**

Replace `LoadCandidateLocalMap()` calls with:

```cpp
const auto candidate_map = map_provider_->LoadCandidate(
    candidate_lidar_pose,
    std::chrono::milliseconds(
        ConfigParameters::Instance().localization_tiled_load_timeout_ms_));
if (candidate_map.state != localization_map::MapRequestState::kReady ||
    !candidate_map.snapshot ||
    !candidate_map.snapshot->registration_cloud ||
    candidate_map.snapshot->registration_cloud->empty()) {
  continue;
}
matcher->AddCloudToLocalMap(
    {*candidate_map.snapshot->registration_cloud});
```

Replace tracking loading with `RequestTracking()`, passing the XY components
of `last_nav_state_.V_` after initialization and `std::nullopt` before a valid
navigation state exists. Handle states exactly:

- `kReady`: update matcher and replace `active_map_snapshot_`;
- `kUnchanged`: continue with current matcher;
- `kPending`: continue only when `previous_snapshot_covers_center` is true;
  otherwise preserve the current health/recovery failure path; do not perform
  map I/O;
- `kUnavailable`: log the provider error, fail initialization or preserve the
  existing timeout/recovery behavior during tracking.

- [ ] **Step 5: Route ground estimation and visualization snapshots**

Build a local XY kd-tree whenever `active_map_snapshot_` changes, preferring its
nonempty ground cloud and falling back to registration cloud. Keep the existing
radius, percentile and minimum-point calculation unchanged. Return provider
overview from `GetGlobalCloudMap()` and the active registration snapshot from a
new `GetLocalCloudMap()` accessor.

- [ ] **Step 6: Run unit and full functional tests**

Run:

```bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  --output-on-failure \
  -E '^(copyright|cpplint|flake8|lint_cmake|uncrustify|xmllint)$'
```

Expected: every enabled functional test passes, including the new surface
contract. Confirm `test_relocalization_efficiency_surface` still locks the
ordered Scan Context evaluation and 50-meter candidate extent through the
provider implementation rather than the removed localization method.

- [ ] **Step 7: Commit**

```bash
git add CMakeLists.txt include/slam/localization.h \
  src/slam/localization.cpp \
  test/test_relocalization_efficiency_surface.cpp
git add -f test/test_tiled_map_localization_surface.cpp
git commit -m "feat(localization): use tiled maps for relocalization and tracking"
```

## Task 8: Mapping export, diagnostics and visualization

**Files:**
- Modify: `src/mapOptimization.cpp`
- Modify: `include/slam/localization.h`
- Modify: `src/slam/localization.cpp`
- Modify: `include/slam/system.h`
- Modify: `src/slam/system.cpp`
- Modify: `config/rviz2.rviz`
- Modify: `test/test_tiled_map_localization_surface.cpp`

- [ ] **Step 1: Add failing export and diagnostic contracts**

Require source tokens for `saveTiledMap`, `GenerateTileMap`, and every approved
diagnostic key:

```text
map_mode
map_manifest_version
map_active_tile_count
map_cached_tile_count
map_cache_mib
map_cache_hits
map_cache_misses
map_tile_load_ms
map_tile_load_failures
map_tile_load_state
map_tile_load_error
```

Also require a new `localization_local_cloud_map_pub_` and require
`System::Run()` to publish `GetLocalCloudMap()` through it, while
`GetGlobalCloudMap()` remains the source for
`lio_sam/localization/global_map`.

- [ ] **Step 2: Invoke tiled generation after successful standard export**

After `GlobalMap.pcd`, optional `GroundMap.pcd` and optional `sc_db.bin` are
complete, execute only when `saveTiledMap` is true:

```cpp
bool tiledMapSaved = !saveTiledMap;
if (saveTiledMap && ret == 0 && groundSaved && scanContextDbSaved) {
  localization_map::TileMapGeneratorOptions options;
  options.map_directory = saveMapDirectory;
  options.output_subdirectory = tiledMapOutputSubdirectory;
  options.tile_size_m = tiledMapSizeMeters;
  options.overview_resolution_m = tiledMapOverviewResolutionMeters;
  options.frame_id = mapFrame;
  try {
    const auto result = localization_map::GenerateTileMap(options);
    tiledMapSaved = true;
    RCLCPP_INFO(get_logger(), "Saved %zu tiled map cells to %s",
                result.tile_count, result.output_directory.c_str());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "Failed to save tiled map: %s", error.what());
  }
}
res->success =
    (ret == 0) && groundSaved && scanContextDbSaved && tiledMapSaved;
```

When the option is false, no `tiled` directory is created and service success
remains governed by the existing outputs.

- [ ] **Step 3: Append provider diagnostics in both health paths**

Add `Localization::MapDiagnostics()` and a `System::AppendMapDiagnostics()`
helper. Call it from both `PublishLocalizationHealth()` and
`PublishRuntimeDiagnostics()` so diagnostics remain visible when health logic
is disabled. Convert enum values to stable lowercase strings. Raise an OK
diagnostic to WARN when the current snapshot is degraded or a nonfatal tile
failure is active; raise it to ERROR when the provider reports unavailable and
no usable snapshot remains. Do not lower an existing health ERROR. Diagnostic
errors contain the manifest-relative tile path or index and error class; strip
the absolute map-directory prefix before publishing.

- [ ] **Step 4: Publish overview and active matching map**

In global mode preserve the current first-subscriber publication. In tiled
mode `GetGlobalCloudMap()` returns overview only when visualization is enabled.
Publish the active registration snapshot on the new local-map publisher
only when its pointer changes, avoiding a full point-cloud copy on every timer
tick.

The local-map publisher is new and must be created exactly as:

```cpp
localization_local_cloud_map_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>(
        "lio_sam/localization/local_map",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
```

Change `Localization::GetGlobalCloudMap()` and the new `GetLocalCloudMap()` to
return `PCLPointCloudXYZI::ConstPtr`; return `nullptr` when unavailable. Update
`config/rviz2.rviz` with a disabled-by-default PointCloud2 display for
`/lio_sam/localization/local_map`, leaving current lidar and global displays
unchanged.

- [ ] **Step 5: Run tests and commit**

Run the full functional CTest selection. Expected: all pass and the source
contract contains every diagnostic key.

Commit:

```bash
git add src/mapOptimization.cpp include/slam/localization.h \
  src/slam/localization.cpp include/slam/system.h src/slam/system.cpp \
  config/rviz2.rviz test/test_tiled_map_localization_surface.cpp
git commit -m "feat(map): export and diagnose tiled localization maps"
```

## Task 9: Tiled replay harness and fault injection

**Files:**
- Create: `test/test_tiled_map_integration.py`
- Modify: `test/integration/stability_common.py`
- Modify: `test/integration/localization_regression.py`
- Modify: `scripts/run_localization_stability.sh`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write failing hermetic harness tests**

Test that runtime parameter generation accepts an optional tiled directory and
writes:

```yaml
localization:
  map_mode: tiled
  map_path: /absolute/path/to/tiled
```

Test dataset validation requires `manifest.yaml` only in tiled mode, leaves
legacy validation unchanged, and rejects a tiled directory outside the selected
map root. Test the shell entrypoint recognizes `tiled-quick` and
`tiled-faults`.

- [ ] **Step 2: Extend the supported script modes**

The usage becomes:

```text
Usage: scripts/run_localization_stability.sh
  {quick|record-baseline|memory|tf-continuity|tiled-generate|tiled-quick|tiled-faults}
```

Behavior:

- `tiled-generate`: run the installed generator against the configured map root;
- `tiled-quick`: require `<map>/tiled/manifest.yaml`, generate tiled runtime
  parameters and invoke the existing 180-second collector;
- `tiled-faults`: copy only the small manifest and selected fixture tiles into
  a temporary directory, then run corrupt-manifest, missing-center and
  missing-neighbor cases without modifying the real map.

All temporary fault directories stay below the script's `mktemp` result and are
removed by the existing trap unless `LIO_SAM_KEEP_RESULTS=1`.

- [ ] **Step 3: Add observable tiled invariants**

Extend diagnostic collection with the approved map keys. `tiled-quick` fails
when:

- observed `map_mode` is not `tiled`;
- `map_cached_tile_count > 25`;
- `map_cache_mib > 1024`;
- `map_tile_load_failures != 0` in the healthy dataset;
- automatic relocalization does not reach OK under the existing deadline;
- existing odometry and health thresholds fail.

`tiled-faults` asserts corrupt manifest and missing center refuse readiness,
while missing neighbor produces a degraded diagnostic with the exact missing
index and no crash.

- [ ] **Step 4: Run hermetic tests and commit**

Run:

```bash
python3 -m unittest -v test.test_tiled_map_integration
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  -R '^(test_tiled_map_integration|test_localization_regression|test_localization_stability_common)$' \
  --output-on-failure
```

Expected: all selected tests pass without ROS graph or external bag access.

Commit:

```bash
git add CMakeLists.txt scripts/run_localization_stability.sh \
  test/integration/stability_common.py \
  test/integration/localization_regression.py
git add -f test/test_tiled_map_integration.py
git commit -m "test(map): add tiled localization replay coverage"
```

## Task 10: Real data acceptance, measurements and handoff

**Files:**
- Modify: `docs/HANDOFF.md`
- Modify: `test/integration/baselines/hesai_sim.yaml` only if a separately
  reviewed tiled metric requires a new explicit key; do not relax existing
  global thresholds.

- [ ] **Step 1: Build and run all functional tests**

Run:

```bash
cd /home/robot/slam_ws
cmake --build build/lio_sam_hesai -- -j1
ctest --test-dir build/lio_sam_hesai --output-on-failure \
  -E '^(copyright|cpplint|flake8|lint_cmake|uncrustify|xmllint)$'
cmake --install build/lio_sam_hesai
```

Expected: build/install succeed and all enabled functional tests pass.

- [ ] **Step 2: Reconfirm the legacy path**

Run:

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
LIO_SAM_KEEP_RESULTS=1 scripts/run_localization_stability.sh quick
```

Expected: exit 0, `error: null`, `valid_odom_ratio >= 0.9`, monotonic odometry,
no error samples and no regression against existing maxima.

- [ ] **Step 3: Generate and inspect `hesai_sim2/tiled`**

Before generation, confirm `/home/robot/map/hesai_sim2/tiled` does not already
exist. If it exists, stop and ask the user whether to preserve it or choose a
new output subdirectory; do not overwrite it.

Run:

```bash
scripts/run_localization_stability.sh tiled-generate
```

Inspect `manifest.yaml`, sum declared point counts, and compare them to the PCD
headers. Expected: global and ground counts are conserved, every file remains
under the tiled root, overview exists and a second generator invocation refuses
overwrite.

- [ ] **Step 4: Run tiled quick and fault acceptance**

Run:

```bash
LIO_SAM_KEEP_RESULTS=1 scripts/run_localization_stability.sh tiled-quick
LIO_SAM_KEEP_RESULTS=1 scripts/run_localization_stability.sh tiled-faults
```

Expected: both exit 0; tiled quick reaches automatic localization without
`/initialpose`; fault cases match the documented readiness/degraded semantics.

- [ ] **Step 5: Compare memory and latency**

Capture from both global and tiled runs:

- map preparation duration;
- process RSS after readiness and at replay end;
- peak cached tile count/MiB;
- maximum frame processing duration and timeout count;
- first odometry time, maximum odometry gap and valid odometry ratio.

Acceptance requires tiled mode not to retain the full global/ground clouds,
never exceed configured cache budgets, add no frame timeout, and remain within
the existing functional baseline. Record exact observed values rather than an
unqualified statement.

- [ ] **Step 6: Update handoff and run integrity checks**

Document mode selection, directory layout, conversion command, rollback to
`map_mode: global`, diagnostic keys, measured results and known operational
limits in `docs/HANDOFF.md`.

Run:

```bash
git diff --check
git status --short
git log -12 --oneline
```

Expected: no whitespace errors; only the handoff/baseline changes intended for
this task remain.

- [ ] **Step 7: Commit final evidence**

```bash
git add -f docs/HANDOFF.md
git add test/integration/baselines/hesai_sim.yaml
git commit -m "docs(map): record tiled localization verification"
```

If the baseline file did not change, omit it from `git add`. Do not commit
generated maps, bags, logs, result JSON or RSS CSV files.

## Final acceptance checklist

- [ ] Old YAML and old map directories still select global mode without edits.
- [ ] `map_mode: tiled` never loads complete GlobalMap/GroundMap clouds or their
  full-map kd-trees.
- [ ] Cold-start Scan Context and manual initialization both work in tiled mode.
- [ ] Tracking requests never perform disk I/O on the localization thread.
- [ ] The active 3×3 and prefetched 5×5 behavior uses the shared index formula.
- [ ] Cache tile count and MiB remain within configured limits under replay.
- [ ] Center, neighbor, ground, overview, timeout and manifest failures have the
  approved semantics and diagnostics.
- [ ] `lio_sam/localization/global_map` publishes overview and
  `lio_sam/localization/local_map` exposes the actual active matching map in
  tiled mode.
- [ ] Mapping creates no tiled output by default and creates an atomic validated
  bundle when explicitly enabled.
- [ ] Full functional CTest and legacy/tiled replay acceptance pass.
- [ ] Handoff contains exact measurements and rollback instructions.
