# Code Defect Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the iVox batch-add early exit, remove two unbuilt SLAM source files, and stop `utility.hpp` from importing the entire `std` namespace.

**Architecture:** Preserve all existing runtime interfaces and algorithms. Add one behavioral gtest around the affected iVox overload and one repository-contract gtest around the cleanup invariants, then make only the production changes required to turn those tests green.

**Tech Stack:** C++17, GoogleTest via `ament_cmake_gtest`, ROS 2 Humble, PCL, CMake/colcon.

---

## File Structure

- Create `test/test_ivox_map_batch_add.cpp`: executable regression for the batch-add control flow.
- Create `test/test_code_hygiene.cpp`: source-tree contract for public-header namespace isolation and dead-file removal.
- Modify `CMakeLists.txt`: register and link the two new gtest targets.
- Modify `src/ivox_map/ivox_map.cpp`: continue the batch after classifying a no-downsample point.
- Modify `include/lio_sam/utility.hpp`: remove the namespace directive and explicitly qualify standard-library names.
- Delete `src/slam/frontend.cpp`: remove unbuilt legacy frontend implementation.
- Delete `src/slam/loop_closure.cpp`: remove unbuilt legacy loop-closure implementation.

### Task 1: Reproduce the iVox batch early-exit defect

**Files:**
- Create: `test/test_ivox_map_batch_add.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing behavior test**

Create `test/test_ivox_map_batch_add.cpp` with:

```cpp
#include "ivox_map/ivox_map.h"

#include <gtest/gtest.h>

#include <vector>

namespace {

PCLPointXYZI MakePoint(float x, float y, float z) {
  PCLPointXYZI point{};
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = 1.0F;
  return point;
}

}  // namespace

TEST(IVoxMapBatchAdd, ContinuesAfterNoDownsamplePoint) {
  IVoxMap::Options options;
  options.resolution_ = 1.0F;
  options.nearby_type_ = IVoxMap::NearbyType::CENTER;
  IVoxMap map(options);

  IVoxMap::PointVector world_points{
      MakePoint(0.0F, 0.0F, 0.0F),
      MakePoint(3.0F, 0.0F, 0.0F),
  };
  std::vector<IVoxMap::PointVector> nearest_points(world_points.size());
  nearest_points.front().push_back(MakePoint(2.0F, 2.0F, 2.0F));

  map.AddPoints(world_points, nearest_points);

  IVoxMap::PointVector closest_points;
  EXPECT_TRUE(map.GetClosestPoint(
      world_points.back(), closest_points, 1U, 0.1F));
}
```

In the `if(BUILD_TESTING)` block of `CMakeLists.txt`, immediately after
`test_bounded_async`, add:

```cmake
  ament_add_gtest(test_ivox_map_batch_add
    test/test_ivox_map_batch_add.cpp
    src/ivox_map/ivox_map.cpp
    src/ivox_map/voxel_grid_node.cpp)
  target_include_directories(test_ivox_map_batch_add PRIVATE
    include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_ivox_map_batch_add
    ${PCL_LIBRARIES} ${GLOG_LIBRARIES})
```

- [ ] **Step 2: Build the new test target**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lio_sam_hesai
```

Expected: package build succeeds and creates the `test_ivox_map_batch_add`
CTest entry.

- [ ] **Step 3: Run the test and verify RED**

Run:

```bash
ctest --test-dir build/lio_sam_hesai \
  -R '^test_ivox_map_batch_add$' --output-on-failure
```

Expected: FAIL in `IVoxMapBatchAdd.ContinuesAfterNoDownsamplePoint` because
`GetClosestPoint` returns false. The existing `return` exits before either
collected point is inserted.

- [ ] **Step 4: Commit the regression test**

Run:

```bash
git add CMakeLists.txt
git add -f test/test_ivox_map_batch_add.cpp
git commit -m "test(ivox): reproduce batch add early exit"
```

### Task 2: Keep processing the iVox batch

**Files:**
- Modify: `src/ivox_map/ivox_map.cpp`
- Test: `test/test_ivox_map_batch_add.cpp`

- [ ] **Step 1: Make the minimal control-flow fix**

In the no-downsample branch of
`IVoxMap::AddPoints(const PointVector&, const std::vector<PointVector>&)`,
replace only the loop exit:

```cpp
            if (fabs(dis_to_center.x()) > 0.5 * options_.resolution_ &&
                fabs(dis_to_center.y()) > 0.5 * options_.resolution_ &&
                fabs(dis_to_center.z()) > 0.5 * options_.resolution_) {
                point_no_need_downsample.emplace_back(point_world);
                continue;
            }
```

- [ ] **Step 2: Run the focused test and verify GREEN**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lio_sam_hesai
ctest --test-dir build/lio_sam_hesai \
  -R '^test_ivox_map_batch_add$' --output-on-failure
```

Expected: PASS, 1 test run and 0 failures.

- [ ] **Step 3: Commit the fix**

Run:

```bash
git add src/ivox_map/ivox_map.cpp
git commit -m "fix(ivox): continue batch after no-downsample point"
```

### Task 3: Lock the code-cleanup contracts

**Files:**
- Create: `test/test_code_hygiene.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing source-tree contract**

Create `test/test_code_hygiene.cpp` with:

```cpp
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {

std::filesystem::path ProjectPath(const std::string& relative_path) {
  return std::filesystem::path(PROJECT_SOURCE_DIR) / relative_path;
}

std::string ReadProjectFile(const std::string& relative_path) {
  std::ifstream file(ProjectPath(relative_path));
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

}  // namespace

TEST(CodeHygiene, PublicUtilityHeaderDoesNotImportStdNamespace) {
  const auto utility = ProjectPath("include/lio_sam/utility.hpp");
  ASSERT_TRUE(std::filesystem::exists(utility));
  EXPECT_EQ(
      ReadProjectFile("include/lio_sam/utility.hpp")
          .find("using namespace std;"),
      std::string::npos);
}

TEST(CodeHygiene, UnbuiltLegacySlamSourcesAreAbsent) {
  EXPECT_FALSE(std::filesystem::exists(
      ProjectPath("src/slam/frontend.cpp")));
  EXPECT_FALSE(std::filesystem::exists(
      ProjectPath("src/slam/loop_closure.cpp")));
}
```

In the `if(BUILD_TESTING)` block of `CMakeLists.txt`, immediately after the
iVox regression target, add:

```cmake
  ament_add_gtest(test_code_hygiene test/test_code_hygiene.cpp)
```

- [ ] **Step 2: Build the new contract test**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lio_sam_hesai
```

Expected: package build succeeds and creates the `test_code_hygiene` CTest
entry.

- [ ] **Step 3: Run the test and verify RED**

Run:

```bash
ctest --test-dir build/lio_sam_hesai \
  -R '^test_code_hygiene$' --output-on-failure
```

Expected: FAIL. The header still contains `using namespace std;`, and both
legacy source paths still exist.

- [ ] **Step 4: Commit the contract test**

Run:

```bash
git add CMakeLists.txt
git add -f test/test_code_hygiene.cpp
git commit -m "test: lock code cleanup contracts"
```

### Task 4: Remove namespace pollution and dead sources

**Files:**
- Modify: `include/lio_sam/utility.hpp`
- Delete: `src/slam/frontend.cpp`
- Delete: `src/slam/loop_closure.cpp`
- Test: `test/test_code_hygiene.cpp`

- [ ] **Step 1: Remove the namespace directive and qualify standard names**

Delete:

```cpp
using namespace std;
```

Apply these exact declaration changes in `ParamServer`:

```cpp
    std::string pointCloudTopic;
    std::string imuTopic;
    std::string odomTopic;
    std::string gpsTopic;

    std::string lidarFrame;
    std::string baselinkFrame;
    std::string odometryFrame;
    std::string mapFrame;

    std::string savePCDDirectory;
    std::string groundPCDDirectory;
    std::string groundPCDDirectoryGlobal;

    std::vector<double> extRotV;
    std::vector<double> extRPYV;
    std::vector<double> extTransV;
```

Qualify the two remaining square-root calls:

```cpp
        if (std::sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() +
                      q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
```

```cpp
    return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
```

```cpp
    return std::sqrt((p1.x-p2.x)*(p1.x-p2.x) +
                     (p1.y-p2.y)*(p1.y-p2.y) +
                     (p1.z-p2.z)*(p1.z-p2.z));
```

- [ ] **Step 2: Delete the two unbuilt source files**

Delete exactly:

```text
src/slam/frontend.cpp
src/slam/loop_closure.cpp
```

- [ ] **Step 3: Run the contract test and verify GREEN**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lio_sam_hesai
ctest --test-dir build/lio_sam_hesai \
  -R '^test_code_hygiene$' --output-on-failure
```

Expected: PASS, 1 CTest entry run and 0 failures.

- [ ] **Step 4: Confirm no unqualified header declarations remain**

Run:

```bash
rg -n 'using namespace std;|^[[:space:]]+(string|vector)<*' \
  include/lio_sam/utility.hpp
```

Expected: no output and exit status 1.

- [ ] **Step 5: Commit the cleanup**

Run:

```bash
git add include/lio_sam/utility.hpp \
  src/slam/frontend.cpp src/slam/loop_closure.cpp
git commit -m "refactor: remove known code defects"
```

### Task 5: Verify the complete subproject

**Files:**
- Verify: `CMakeLists.txt`
- Verify: `include/lio_sam/utility.hpp`
- Verify: `src/ivox_map/ivox_map.cpp`
- Verify: `test/test_ivox_map_batch_add.cpp`
- Verify: `test/test_code_hygiene.cpp`

- [ ] **Step 1: Verify the source contracts directly**

Run:

```bash
rg -n 'using namespace std;|src/slam/(frontend|loop_closure)\\.cpp' \
  include/lio_sam/utility.hpp CMakeLists.txt
test ! -e src/slam/frontend.cpp
test ! -e src/slam/loop_closure.cpp
```

Expected: `rg` produces no output and both file-existence checks succeed.

- [ ] **Step 2: Run a clean package build**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lio_sam_hesai \
  --cmake-clean-cache
```

Expected: `lio_sam_hesai` finishes successfully. Existing CMake/PCL/Boost/TBB
warnings may remain; no new compile or link errors are allowed.

- [ ] **Step 3: Run the two new tests together**

Run:

```bash
ctest --test-dir build/lio_sam_hesai \
  -R '^test_(ivox_map_batch_add|code_hygiene)$' --output-on-failure
```

Expected: 2 tests passed, 0 tests failed.

- [ ] **Step 4: Run all functional and accepted lint tests**

Run:

```bash
ctest --test-dir build/lio_sam_hesai --output-on-failure \
  -E '^(copyright|cpplint|flake8|lint_cmake|uncrustify|xmllint)$'
```

Expected: 17 tests passed, 0 tests failed.

- [ ] **Step 5: Check patch integrity and branch contents**

Run:

```bash
git diff --check main...HEAD
git status --short --branch
git log --oneline main..HEAD
```

Expected: `git diff --check` has no output; the status shows only intentionally
untracked local build artifacts, if present; the branch log contains the spec,
plan, tests, and implementation commits.
