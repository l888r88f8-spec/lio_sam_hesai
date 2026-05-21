# Ground Grid Patch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace scan-line-first ground patching with local elevation-grid ground patching so bridge decks, ramps, and slopes remain usable 3D navigation ground without artificial layered steps.

**Architecture:** Keep the existing `imageProjection.cpp` ground segmentation as the source of measured ground points. Add an optional BEV elevation-grid patch stage that samples only inside locally continuous traversable ground cells, using local plane fitting and residual checks so it does not bridge curb edges, bridge sides, slope breaks, steps, or holes. Keep parameters minimal: enable flag, grid resolution, search radius, and max traversable slope.

**Tech Stack:** ROS 2 Humble, C++17, PCL, OpenCV `cv::Mat`, existing `ParamServer` YAML parameter flow.

---

## Files

- Modify: `include/lio_sam/utility.hpp`
  - Add grid patch parameters.
  - Keep current horizontal patch parameters for comparison, but default horizontal patch can be disabled after grid patch is enabled.
- Modify: `config/mapping.yaml`
  - Add four grid patch parameters near ground segmentation settings.
  - Disable horizontal and vertical patching by default once grid patch is active.
- Modify: `src/imageProjection.cpp`
  - Add small internal helpers for local grid indexing, local plane fitting, residual/slope gating, and patch point generation.
  - Call grid patching after measured ground extraction and before downsampling.
- Test/Verify: no unit test framework currently exists for this package. Use compile verification and runtime ROS topic inspection with a slope/bridge bag or live run.

## Design Constraints

- Do not flatten ground Z.
- Do not generate points over known non-ground cells.
- Do not use a global horizontal plane as a traversability assumption.
- Allow sloped bridge/ramp surfaces up to `groundGridPatchMaxSlopeDeg`.
- Prefer under-patching over wrong cross-boundary patching.
- Patch points are for ground output/map density only; they must not feed back into `groundMat` classification for the same frame.

---

### Task 1: Add Minimal Grid Patch Parameters

**Files:**
- Modify: `include/lio_sam/utility.hpp`
- Modify: `config/mapping.yaml`

- [ ] **Step 1: Add member variables in `utility.hpp`**

Insert after existing ground patch parameters:

```cpp
bool groundGridPatchEnable;
float groundGridPatchResolution;
float groundGridPatchSearchRadius;
float groundGridPatchMaxSlopeDeg;
```

- [ ] **Step 2: Read parameters in `ParamServer`**

Insert near the existing ground patch parameter reads:

```cpp
declare_parameter("groundGridPatchEnable", false);
get_parameter("groundGridPatchEnable", groundGridPatchEnable);
declare_parameter("groundGridPatchResolution", 0.10);
get_parameter("groundGridPatchResolution", groundGridPatchResolution);
declare_parameter("groundGridPatchSearchRadius", 0.40);
get_parameter("groundGridPatchSearchRadius", groundGridPatchSearchRadius);
declare_parameter("groundGridPatchMaxSlopeDeg", 25.0);
get_parameter("groundGridPatchMaxSlopeDeg", groundGridPatchMaxSlopeDeg);
```

- [ ] **Step 3: Update `mapping.yaml` defaults**

Use these values in the ground segmentation block:

```yaml
    distanceForPatchBetweenRings: 0.0            # 纵向ring间补点最大距离，0表示关闭
    groundPatchHorizontalEnable: false           # 横向ring内补点，保留用于对比
    groundPatchHorizontalMaxDistance: 0.6        # 横向补点允许跨越的最大端点距离，单位米
    groundGridPatchEnable: true                  # 是否启用局部高程栅格补点
    groundGridPatchResolution: 0.10              # 高程栅格补点分辨率，单位米
    groundGridPatchSearchRadius: 0.40            # 空栅格局部平面搜索半径，单位米
    groundGridPatchMaxSlopeDeg: 25.0             # 允许补点的最大局部坡度，单位度
```

- [ ] **Step 4: Build**

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/robot/slam_ws/install/setup.bash
colcon build --symlink-install --packages-select lio_sam_hesai
```

Expected: package builds. Existing PCL/Boost notes may still appear; no compile errors.

---

### Task 2: Add Local Plane Utility

**Files:**
- Modify: `src/imageProjection.cpp`

- [ ] **Step 1: Add helper struct inside `ImageProjection`**

Place before patching methods:

```cpp
struct GroundPlaneModel
{
    float a = 0.0f;
    float b = 0.0f;
    float c = 1.0f;
    float d = 0.0f;
};
```

- [ ] **Step 2: Add plane evaluation helpers**

Add:

```cpp
bool evaluatePlaneZ(const GroundPlaneModel &plane,
                    float x, float y, float *z) const
{
    if (z == nullptr || std::fabs(plane.c) < kMinPlaneNorm)
        return false;
    *z = -(plane.a * x + plane.b * y + plane.d) / plane.c;
    return std::isfinite(*z);
}

float groundPlaneSlopeDeg(const GroundPlaneModel &plane) const
{
    const float norm_xy = std::sqrt(plane.a * plane.a + plane.b * plane.b);
    const float norm_z = std::fabs(plane.c);
    return std::atan2(norm_xy, norm_z) / kDegToRad;
}
```

- [ ] **Step 3: Add local plane fit using PCL**

Add a helper that fits a plane to measured ground neighbors:

```cpp
bool fitLocalGroundPlane(const pcl::PointCloud<PointType>::Ptr &points,
                         GroundPlaneModel *plane) const
{
    if (!plane || points == nullptr || points->size() < 6)
        return false;

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    pcl::SACSegmentation<PointType> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(80);
    segmentation.setDistanceThreshold(groundPlaneDistance);
    segmentation.setInputCloud(points);
    segmentation.segment(inliers, coefficients);

    if (coefficients.values.size() < 4 || inliers.indices.size() < 6)
        return false;

    plane->a = coefficients.values[0];
    plane->b = coefficients.values[1];
    plane->c = coefficients.values[2];
    plane->d = coefficients.values[3];

    return groundPlaneSlopeDeg(*plane) <= groundGridPatchMaxSlopeDeg;
}
```

- [ ] **Step 4: Build**

Run the same `colcon build` command.

Expected: package builds.

---

### Task 3: Build Occupied Ground Grid From Measured Points

**Files:**
- Modify: `src/imageProjection.cpp`

- [ ] **Step 1: Add grid cell data structs**

Place near `GroundPlaneModel`:

```cpp
struct GroundGridKey
{
    int ix = 0;
    int iy = 0;

    bool operator==(const GroundGridKey &other) const
    {
        return ix == other.ix && iy == other.iy;
    }
};

struct GroundGridKeyHash
{
    std::size_t operator()(const GroundGridKey &key) const
    {
        const std::size_t h1 = std::hash<int>()(key.ix);
        const std::size_t h2 = std::hash<int>()(key.iy);
        return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }
};

struct GroundGridCell
{
    std::vector<int> point_indices;
};
```

- [ ] **Step 2: Add coordinate conversion helpers**

Add:

```cpp
GroundGridKey groundGridKeyForPoint(const PointType &point) const
{
    const float res = std::max(groundGridPatchResolution, 0.02f);
    return {
        static_cast<int>(std::floor(point.x / res)),
        static_cast<int>(std::floor(point.y / res))
    };
}

PointType groundGridCellCenter(const GroundGridKey &key, float z) const
{
    const float res = std::max(groundGridPatchResolution, 0.02f);
    PointType point;
    point.x = (static_cast<float>(key.ix) + 0.5f) * res;
    point.y = (static_cast<float>(key.iy) + 0.5f) * res;
    point.z = z;
    point.intensity = 0.0f;
    return point;
}
```

- [ ] **Step 3: Add grid builder**

Add:

```cpp
using GroundGridMap =
    std::unordered_map<GroundGridKey, GroundGridCell, GroundGridKeyHash>;

void buildMeasuredGroundGrid(const pcl::PointCloud<PointType>::Ptr &groundInput,
                             GroundGridMap *grid) const
{
    if (!grid)
        return;
    grid->clear();
    if (groundInput == nullptr)
        return;

    for (int i = 0; i < static_cast<int>(groundInput->points.size()); ++i)
    {
        const PointType &point = groundInput->points[i];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z))
        {
            continue;
        }
        (*grid)[groundGridKeyForPoint(point)].point_indices.push_back(i);
    }
}
```

- [ ] **Step 4: Include headers**

Ensure `src/imageProjection.cpp` includes:

```cpp
#include <unordered_map>
#include <unordered_set>
```

- [ ] **Step 5: Build**

Run `colcon build --symlink-install --packages-select lio_sam_hesai`.

Expected: package builds.

---

### Task 4: Generate Grid Patch Points Only In Local Continuous Areas

**Files:**
- Modify: `src/imageProjection.cpp`

- [ ] **Step 1: Add neighbor collection helper**

Add:

```cpp
void collectNeighborGroundPoints(const GroundGridKey &key,
                                 const pcl::PointCloud<PointType>::Ptr &groundInput,
                                 const GroundGridMap &grid,
                                 pcl::PointCloud<PointType>::Ptr neighbors) const
{
    neighbors->clear();
    if (groundInput == nullptr)
        return;

    const float res = std::max(groundGridPatchResolution, 0.02f);
    const int cell_radius = std::max(
        1, static_cast<int>(std::ceil(groundGridPatchSearchRadius / res)));

    for (int dx = -cell_radius; dx <= cell_radius; ++dx)
    {
        for (int dy = -cell_radius; dy <= cell_radius; ++dy)
        {
            GroundGridKey neighbor_key{key.ix + dx, key.iy + dy};
            const auto iter = grid.find(neighbor_key);
            if (iter == grid.end())
                continue;

            for (const int index : iter->second.point_indices)
            {
                if (index >= 0 && index < static_cast<int>(groundInput->points.size()))
                    neighbors->push_back(groundInput->points[index]);
            }
        }
    }
}
```

- [ ] **Step 2: Add candidate empty-cell collection**

Add:

```cpp
using GroundGridKeySet =
    std::unordered_set<GroundGridKey, GroundGridKeyHash>;

void collectGridPatchCandidates(const GroundGridMap &grid,
                                GroundGridKeySet *candidates) const
{
    if (!candidates)
        return;
    candidates->clear();
    const float res = std::max(groundGridPatchResolution, 0.02f);
    const int cell_radius = std::max(
        1, static_cast<int>(std::ceil(groundGridPatchSearchRadius / res)));

    for (const auto &entry : grid)
    {
        const GroundGridKey &occupied_key = entry.first;
        for (int dx = -cell_radius; dx <= cell_radius; ++dx)
        {
            for (int dy = -cell_radius; dy <= cell_radius; ++dy)
            {
                if (dx == 0 && dy == 0)
                    continue;

                GroundGridKey candidate{occupied_key.ix + dx, occupied_key.iy + dy};
                if (grid.find(candidate) != grid.end())
                    continue;

                candidates->insert(candidate);
            }
        }
    }
}
```

- [ ] **Step 3: Add patch generation loop**

Add:

```cpp
void appendGridGroundPatches(pcl::PointCloud<PointType>::Ptr groundOutput)
{
    if (!groundGridPatchEnable ||
        groundGridPatchResolution <= 0.0f ||
        groundGridPatchSearchRadius <= 0.0f ||
        groundOutput == nullptr ||
        groundOutput->empty())
    {
        return;
    }

    GroundGridMap grid;
    buildMeasuredGroundGrid(groundOutput, &grid);
    if (grid.empty())
        return;

    GroundGridKeySet candidates;
    collectGridPatchCandidates(grid, &candidates);
    if (candidates.empty())
        return;

    pcl::PointCloud<PointType>::Ptr neighbors(new pcl::PointCloud<PointType>());
    std::vector<PointType> patch_points;
    const std::size_t max_patch_points = groundOutput->size() * 2;

    for (const GroundGridKey &candidate : candidates)
    {
        collectNeighborGroundPoints(candidate, groundOutput, grid, neighbors);
        GroundPlaneModel plane;
        if (!fitLocalGroundPlane(neighbors, &plane))
            continue;

        float z = 0.0f;
        PointType center = groundGridCellCenter(candidate, 0.0f);
        if (!evaluatePlaneZ(plane, center.x, center.y, &z))
            continue;

        center.z = z;
        patch_points.push_back(center);
        if (patch_points.size() >= max_patch_points)
            break;
    }

    for (const auto &point : patch_points)
        groundOutput->push_back(point);
}
```

- [ ] **Step 4: Confirm container safety**

Check that `collectGridPatchCandidates()` only reads `grid` while iterating it, and that `appendGridGroundPatches()` never inserts into `grid` after candidate collection. The only growing container during patch generation should be `patch_points`.

- [ ] **Step 5: Build**

Run `colcon build --symlink-install --packages-select lio_sam_hesai`.

Expected: package builds.

---

### Task 5: Integrate Grid Patching Into Ground Extraction

**Files:**
- Modify: `src/imageProjection.cpp`

- [ ] **Step 1: Call grid patching after measured ground extraction**

In `extractGroundPointsFromPlane`, replace:

```cpp
appendHorizontalGroundPatches(coefficients, groundOutput);
appendConfirmedGroundPatches(coefficients, groundOutput);
appendGroundPatchToSensorCenter(coefficients, groundOutput);
```

with:

```cpp
appendGridGroundPatches(groundOutput);
appendHorizontalGroundPatches(coefficients, groundOutput);
appendConfirmedGroundPatches(coefficients, groundOutput);
appendGroundPatchToSensorCenter(coefficients, groundOutput);
```

- [ ] **Step 2: Call grid patching in fallback paths**

In `extractLocalGroundFromScan`, after `collectMeasuredGroundSeeds(groundOutput);`, call:

```cpp
appendGridGroundPatches(groundOutput);
```

before horizontal/vertical/center patching.

- [ ] **Step 3: Keep patch output out of classification**

Do not write generated grid patch points back into `groundMat`, `rangeMat`, or `fullCloud`.

- [ ] **Step 4: Build**

Run `colcon build --symlink-install --packages-select lio_sam_hesai`.

Expected: package builds.

---

### Task 6: Runtime Verification On Bridge/Slope Data

**Files:**
- No code changes.

- [ ] **Step 1: Start mapping**

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/robot/slam_ws/install/setup.bash
ros2 launch lio_sam_hesai mapping.launch.py
```

- [ ] **Step 2: Inspect current-frame ground output**

Run in another terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/robot/slam_ws/install/setup.bash
ros2 topic hz /lio_sam/deskew/cloud_info
```

Expected: topic publishes at lidar frame rate.

- [ ] **Step 3: Visual check in RViz**

Enable:

```text
/lio_sam/mapping/ground_cloud_global
/lio_sam/mapping/map_global
/lio_sam/cloud_nonground
```

Expected on bridge/ramp:

```text
ground cloud follows bridge/ramp Z
no fixed-Z flattening
no staircase sheets at downhill sections
no filled bridge side/curb/drop-off
```

- [ ] **Step 4: Parameter sweep**

Try only these three changes first:

```yaml
groundGridPatchResolution: 0.08
groundGridPatchSearchRadius: 0.35
groundGridPatchMaxSlopeDeg: 30.0
```

Expected:

```text
smaller resolution increases density
smaller search radius reduces cross-boundary filling
larger max slope keeps steeper bridge/ramp surfaces
```

- [ ] **Step 5: Regression check**

Set:

```yaml
groundGridPatchEnable: false
groundPatchHorizontalEnable: false
distanceForPatchBetweenRings: 0.0
```

Expected: output returns to measured-only ground plus center patch behavior. This is the fallback mode if patching creates navigation artifacts.

---

## Self-Review

- Spec coverage: plan covers bridge/ramp true-Z preservation, 3D navigation ground output, minimal parameters, and avoiding cross-boundary patching.
- Placeholder scan: no implementation step contains TBD/TODO placeholders.
- Type consistency: all new names use `groundGridPatch*`; helper names are consistent across tasks.
- Risk: Task 4 may generate too many candidates on large dense frames; a hard cap is included. If runtime load is high, the candidate loop should be limited to cells adjacent to occupied cells rather than all cells in the search radius.
