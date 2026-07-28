# Localization Stability Validation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a reproducible black-box localization regression and RSS soak-test framework for the external `hesai_sim` rosbag and `hesai_sim2` map, with a self-hosted Orin CI entrypoint.

**Architecture:** Python 3 and `rclpy` manage a launch process and `ros2 bag play`, collect only scalar odom/diagnostic metrics, and compare them to a reviewed YAML baseline. A shared module validates external inputs, generates a temporary portable parameter file, creates an isolated ROS domain, and tears down process groups. The normal CTest suite exercises all non-ROS, non-bag logic; replay and RSS runs are explicit scripts for local machines and self-hosted CI.

**Tech Stack:** Python 3.10, PyYAML, rclpy, ROS 2 Humble CLI, CTest, `/proc`, GitHub Actions self-hosted arm64 runner.

---

## File structure

- `test/integration/stability_common.py` — validates the external bag/map, creates a temporary parameters file, starts/stops child process groups, and creates result directories.
- `test/integration/stability_baseline.py` — parses the versioned baseline, validates its schema, converts observed scalar metrics into a candidate, and makes deterministic comparisons.
- `test/integration/localization_regression.py` — quick replay CLI; subscribes to odom, diagnostics and clock, emits `observed.json`, optionally emits a candidate baseline.
- `test/integration/localization_rss.py` — long-running RSS CLI; samples only the relocalization process’s `VmRSS` and writes CSV/summary JSON.
- `test/test_localization_stability_common.py` — isolated tests for data validation, parameter rewriting and process-free result handling.
- `test/test_localization_stability_baseline.py` — schema and boundary tests for baseline handling.
- `test/test_localization_stability_rss.py` — `/proc` parser and linear-slope tests.
- `test/integration/baselines/hesai_sim.yaml` — reviewed bag identity, quick replay window and metric/RSS limits.
- `test/integration/README.md` — developer and CI operator instructions.
- `scripts/run_localization_stability.sh` — only supported shell entrypoint for quick, record-baseline and memory modes.
- `.github/workflows/localization-stability.yml` — manual/daily quick regression and weekly/manual RSS test on an Orin runner.
- `CMakeLists.txt` — registers only the three hermetic Python unit suites with CTest.

### Task 1: External-data and runtime primitives (TDD)

**Files:**
- Create: `test/integration/stability_common.py`
- Create: `test/test_localization_stability_common.py`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing hermetic test suite**

Create `test/test_localization_stability_common.py` with a `unittest` suite. It must use `tempfile.TemporaryDirectory`, write this minimal metadata fixture, and never import `rclpy`:

```python
METADATA = """rosbag2_bagfile_information:
  duration: {nanoseconds: 756378139407}
  message_count: 46875
  topics_with_message_count:
    - topic_metadata: {name: /imu, type: sensor_msgs/msg/Imu}
      message_count: 42617
    - topic_metadata: {name: /lidar_points, type: sensor_msgs/msg/PointCloud2}
      message_count: 4258
  relative_file_paths: [sample.db3]
"""
```

The tests must assert all of the following behaviours:

```python
identity = validate_dataset(bag_dir, map_dir)
self.assertEqual(identity.duration_ns, 756378139407)
self.assertEqual(identity.topic_counts["/imu"], 42617)
self.assertEqual(identity.topic_counts["/lidar_points"], 4258)

with self.assertRaisesRegex(DatasetError, "GroundMap.pcd"):
    validate_dataset(bag_dir, incomplete_map_dir)

runtime = write_runtime_parameters(template, map_dir, output_dir)
text = runtime.read_text(encoding="utf-8")
self.assertIn(f'map_path: "{map_dir / "GlobalMap.pcd"}"', text)
self.assertIn(f'ground_map_path: "{map_dir / "GroundMap.pcd"}"', text)
self.assertIn(f'sc_db_path: "{map_dir / "sc_db.bin"}"', text)

with self.assertRaisesRegex(DatasetError, "exactly once"):
    write_runtime_parameters(duplicate_map_path_template, map_dir, output_dir)
```

- [ ] **Step 2: Register the test and prove it fails**

Append this exact block inside `if(BUILD_TESTING)` in `CMakeLists.txt`:

```cmake
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_localization_stability_common
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_localization_stability_common.py)
  set_tests_properties(test_localization_stability_common PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test/integration")
```

Run:

```bash
cd /home/robot/slam_ws
colcon build --packages-select lio_sam_hesai
ctest --test-dir build/lio_sam_hesai -R test_localization_stability_common --output-on-failure
```

Expected: failure because `stability_common` does not exist.

- [ ] **Step 3: Implement the shared module**

Create `test/integration/stability_common.py` with these public definitions and behaviour:

```python
from __future__ import annotations

import os
import re
import signal
from dataclasses import dataclass
from pathlib import Path

import yaml


class DatasetError(RuntimeError):
    pass


@dataclass(frozen=True)
class DatasetIdentity:
    duration_ns: int
    message_count: int
    topic_counts: dict[str, int]
    db3_bytes: int
    map_points: int
    map_bytes: int


def _pcd_points(path: Path) -> int:
    for line in path.read_text(encoding="latin-1").splitlines():
        if line.startswith("POINTS "):
            return int(line.split()[1])
        if line == "DATA binary":
            break
    raise DatasetError(f"{path}: missing PCD POINTS header")


def validate_dataset(bag_dir: Path, map_dir: Path) -> DatasetIdentity:
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        raise DatasetError(f"missing {metadata_path}")
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    info = metadata.get("rosbag2_bagfile_information", {})
    db3_files = sorted(bag_dir.glob("*.db3"))
    if len(db3_files) != 1:
        raise DatasetError(f"expected exactly one .db3 in {bag_dir}")
    required = ("GlobalMap.pcd", "GroundMap.pcd", "sc_db.bin")
    for name in required:
        if not (map_dir / name).is_file():
            raise DatasetError(f"missing {map_dir / name}")
    counts = {
        item["topic_metadata"]["name"]: int(item["message_count"])
        for item in info.get("topics_with_message_count", [])
    }
    return DatasetIdentity(
        duration_ns=int(info["duration"]["nanoseconds"]),
        message_count=int(info["message_count"]),
        topic_counts=counts,
        db3_bytes=db3_files[0].stat().st_size,
        map_points=_pcd_points(map_dir / "GlobalMap.pcd"),
        map_bytes=(map_dir / "GlobalMap.pcd").stat().st_size,
    )


def write_runtime_parameters(template: Path, map_dir: Path, output_dir: Path) -> Path:
    text = template.read_text(encoding="utf-8")
    replacements = {
        "map_path": map_dir / "GlobalMap.pcd",
        "ground_map_path": map_dir / "GroundMap.pcd",
        "sc_db_path": map_dir / "sc_db.bin",
    }
    for key, value in replacements.items():
        pattern = rf'^(?P<indent>\\s*){key}:.*$'
        text, count = re.subn(
            pattern, rf'\\g<indent>{key}: "{value}"', text,
            count=1, flags=re.MULTILINE)
        if count != 1 or len(re.findall(pattern, template.read_text(encoding="utf-8"), re.MULTILINE)) != 1:
            raise DatasetError(f"parameter {key} must occur exactly once")
    output_dir.mkdir(parents=True, exist_ok=True)
    runtime = output_dir / "relocalization.runtime.yaml"
    runtime.write_text(text, encoding="utf-8")
    return runtime


def terminate_process_group(pid: int, timeout_seconds: float = 10.0) -> None:
    try:
        os.killpg(pid, signal.SIGINT)
    except ProcessLookupError:
        return
```

Keep process waiting in the caller so this function never calls `sleep` or hides a failed termination.

- [ ] **Step 4: Run the test and the package build**

Run the Step 2 commands again. Expected: `test_localization_stability_common` passes. Then run:

```bash
python3 -m py_compile test/integration/stability_common.py test/test_localization_stability_common.py
```

- [ ] **Step 5: Commit**

```bash
git add -f CMakeLists.txt test/integration/stability_common.py test/test_localization_stability_common.py
git commit -m "test(stability): add external data validation helpers"
```

### Task 2: Baseline schema and deterministic comparisons (TDD)

**Files:**
- Create: `test/integration/stability_baseline.py`
- Create: `test/test_localization_stability_baseline.py`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write failing schema/comparison tests**

Create a baseline fixture in the test with this complete shape:

```python
BASELINE = {
    "schema_version": 1,
    "dataset": {
        "duration_ns": 756378139407,
        "message_count": 46875,
        "topic_counts": {"/imu": 42617, "/lidar_points": 4258},
        "map_points": 178455,
    },
    "quick": {
        "start_offset_seconds": 0.0,
        "duration_seconds": 180.0,
        "rate": 1.0,
        "startup_grace_seconds": 30.0,
        "first_odom_max_seconds": 60.0,
        "min_valid_odom_ratio": 0.95,
        "max_odom_gap_seconds": 1.0,
        "ok_deadline_seconds": 90.0,
        "max_error_samples": 0,
        "max_non_ok_gap_seconds": 5.0,
        "health_score_min": 0.5,
        "reg_fitness_max": 1.0,
        "covariance_scale_max": 100.0,
        "lidar_rate_min_hz": 5.0,
        "imu_rate_min_hz": 50.0,
    },
    "memory": {
        "warmup_seconds": 300.0,
        "duration_seconds": 7200.0,
        "sample_period_seconds": 5.0,
        "max_rss_mib": 4096.0,
        "max_slope_mib_per_hour": 64.0,
    },
}
```

The tests must prove `load_baseline()` rejects an unknown schema version and a missing required key; `compare_quick_metrics()` accepts metrics on the boundary; and it reports every violated key when passed this input:

```python
{"first_odom_seconds": 61.0, "valid_odom_ratio": 0.90,
 "max_odom_gap_seconds": 2.0, "first_ok_seconds": None,
 "error_samples": 1, "max_non_ok_gap_seconds": 6.0,
 "min_health_score": 0.2, "max_reg_fitness": 1.2,
 "max_covariance_scale": 101.0, "min_lidar_rate_hz": 4.0,
 "min_imu_rate_hz": 40.0, "timestamp_monotonic": False}
```

- [ ] **Step 2: Register the failing test**

Add this CMake block beside Task 1’s test:

```cmake
  add_test(NAME test_localization_stability_baseline
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_localization_stability_baseline.py)
  set_tests_properties(test_localization_stability_baseline PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test/integration")
```

Run the focused CTest command and confirm import failure.

- [ ] **Step 3: Implement the baseline module**

Create `test/integration/stability_baseline.py`. It must:

1. call `yaml.safe_load` and require `schema_version == 1`;
2. require every key in the exact fixture above;
3. return the parsed mapping only after validation;
4. implement `compare_quick_metrics(baseline, observed) -> list[str]` with one human-readable item for every violated limit;
5. append `timestamp_monotonic must be true` unless the observed value is exactly `True`; and
6. implement `write_candidate(path, identity, observed)` which writes a candidate with the observed dataset identity and conservative limits:
   `first_odom_max_seconds=max(observed*1.25, observed+2.0)`,
   `min_valid_odom_ratio=max(0.0, observed*0.90)`,
   `max_odom_gap_seconds=max(observed*1.25, observed+0.1)`,
   and the other scalar bounds widened by 10% in the safe direction.

The comparison must never alter the loaded baseline. An empty `first_ok_seconds` and a non-finite scalar are violations, never treated as zero.

- [ ] **Step 4: Make the unit suite green**

Run:

```bash
cd /home/robot/slam_ws
colcon build --packages-select lio_sam_hesai
ctest --test-dir build/lio_sam_hesai -R test_localization_stability_baseline --output-on-failure
```

Expected: all baseline tests pass and no bag is required.

- [ ] **Step 5: Commit**

```bash
git add -f CMakeLists.txt test/integration/stability_baseline.py test/test_localization_stability_baseline.py
git commit -m "test(stability): add localization metric baseline contract"
```

### Task 3: Quick replay collector and baseline capture

**Files:**
- Create: `test/integration/localization_regression.py`
- Create: `test/integration/baselines/hesai_sim.yaml`
- Modify: `test/integration/stability_common.py`

- [ ] **Step 1: Add collector tests without a bag**

Extend `test/test_localization_stability_baseline.py` with direct tests for a `MetricCollector` class using only scalar method calls:

```python
collector.clock(100.0)
collector.odom(101.5, 0.0, 0.0, 0.0, 1.0)
collector.diagnostic(102.0, 0, {"health_score": "0.8", "reg_fitness": "0.2",
                                "covariance_scale": "1.0", "lidar_rate_hz": "10.0",
                                "imu_rate_hz": "100.0", "timestamp_monotonic": "true"})
metrics = collector.finish(110.0)
self.assertEqual(metrics["first_odom_seconds"], 1.5)
self.assertEqual(metrics["first_ok_seconds"], 2.0)
self.assertTrue(metrics["timestamp_monotonic"])
```

Add invalid quaternion, NaN diagnostic number, non-target diagnostic name and no-OK cases; each must be reflected in the finished metrics instead of raising from a callback.

- [ ] **Step 2: Run the new test and confirm it fails**

Run the focused `ctest -R test_localization_stability_baseline` command. Expected: import/attribute failure for `MetricCollector`.

- [ ] **Step 3: Implement the replay CLI**

Implement `localization_regression.py` with:

```text
python3 localization_regression.py \
  --bag /absolute/bag --map /absolute/map/dir \
  --params-template /absolute/relocalization.yaml \
  --baseline /absolute/hesai_sim.yaml --output-dir /absolute/results
```

The CLI must accept `--record-baseline` but reject it when `--baseline` is also supplied. It must create an output directory containing `launch.log`, `bag-play.log`, `observed.json` and, only in record mode, `candidate-baseline.yaml`.

The `MetricCollector` must calculate exactly these JSON fields:

```text
first_odom_seconds, valid_odom_ratio, max_odom_gap_seconds,
first_ok_seconds, error_samples, max_non_ok_gap_seconds,
min_health_score, max_reg_fitness, max_covariance_scale,
min_lidar_rate_hz, min_imu_rate_hz, timestamp_monotonic
```

Use `rclpy` subscriptions to `/clock`, `lio_sam/localization/odometry`, and `/diagnostics`. Accept an odom only if all position/quaternion components are finite and the quaternion norm is within `1e-3` of one. From every `DiagnosticArray`, inspect only a status whose name equals `lio_sam_hesai: localization`; collect scalar values by key and treat malformed values as non-finite observations. A diagnostic level of `0` is OK and level `2` is ERROR.

Before playback, call `validate_dataset`, create the temporary parameter file, allocate `ROS_DOMAIN_ID` from `200 + (os.getpid() % 30)`, then start these process groups with that environment:

```text
ros2 launch lio_sam_hesai localization_with_nonground.launch.py
  use_sim:=true use_rviz:=false publish_static_tf:=true
  params_file:=${results_dir}/relocalization.runtime.yaml

ros2 bag play "${bag_dir}" --clock 100 --rate "${quick_rate}"
  --start-offset "${quick_start_offset_seconds}"
  --disable-keyboard-controls
```

Wait `startup_grace_seconds` in wall time while verifying the launch process remains alive. Stop bag playback only after the `/clock` delta reaches `duration_seconds`; then stop both process groups, write observed metrics, and call `compare_quick_metrics`. A failed comparison exits 1 after writing all artifacts. Every `finally` path must terminate and `wait()` for both child process groups.

- [ ] **Step 4: Verify collector logic and syntax**

Run:

```bash
python3 -m py_compile test/integration/localization_regression.py
cd /home/robot/slam_ws
colcon build --packages-select lio_sam_hesai
ctest --test-dir build/lio_sam_hesai -R 'test_localization_stability_(common|baseline)' --output-on-failure
```

Expected: all hermetic tests pass; no replay has started.

- [ ] **Step 5: Capture and review the real candidate baseline**

Run the collector against the confirmed external inputs:

```bash
source /opt/ros/humble/setup.bash
source /home/robot/slam_ws/install/setup.bash
python3 test/integration/localization_regression.py \
  --bag /home/robot/rosbag2/hesai_sim \
  --map /home/robot/map/hesai_sim2 \
  --params-template config/relocalization.yaml \
  --output-dir /tmp/lio-sam-hesai-baseline \
  --record-baseline
```

Inspect `observed.json`, `candidate-baseline.yaml`, `launch.log` and `bag-play.log`. Copy the candidate into `test/integration/baselines/hesai_sim.yaml` only if it contains the expected dataset identity (`756378139407 ns`, `46875` messages, `/imu=42617`, `/lidar_points=4258`) and the run contains a finite first odom plus at least one `OK` diagnostic. Re-run in normal comparison mode using that baseline and require exit code zero.

- [ ] **Step 6: Commit**

```bash
git add -f test/integration/localization_regression.py \
  test/integration/stability_common.py \
  test/integration/stability_baseline.py \
  test/integration/baselines/hesai_sim.yaml
git commit -m "test(stability): add hesai simulation replay regression"
```

### Task 4: RSS sampler and Orin thresholds (TDD)

**Files:**
- Create: `test/integration/localization_rss.py`
- Create: `test/test_localization_stability_rss.py`
- Modify: `CMakeLists.txt`
- Modify: `test/integration/baselines/hesai_sim.yaml`

- [ ] **Step 1: Write failing RSS parser/slope tests**

Create tests for these exact cases:

```python
self.assertEqual(parse_vmrss_kib("Name:\tnode\\nVmRSS:\t 12345 kB\\n"), 12345)
with self.assertRaisesRegex(RuntimeError, "VmRSS"):
    parse_vmrss_kib("Name:\tnode\\n")
self.assertAlmostEqual(linear_slope_mib_per_hour([(0.0, 100.0), (3600.0, 112.0)]), 12.0)
self.assertAlmostEqual(linear_slope_mib_per_hour([(0.0, 100.0), (60.0, 100.0)]), 0.0)
```

Also assert `summarize_samples(samples, warmup_seconds)` discards all points before warmup, rejects fewer than two remaining samples, and reports `peak_rss_mib` plus `slope_mib_per_hour`.

- [ ] **Step 2: Register and run the failing test**

Add the third `add_test`/`PYTHONPATH` pair for `test_localization_stability_rss` and run it. Expected: import failure.

- [ ] **Step 3: Implement the memory CLI**

Implement `localization_rss.py` by importing `validate_dataset`, `write_runtime_parameters` and `terminate_process_group` from `stability_common.py`. Allocate `ROS_DOMAIN_ID` with `200 + (os.getpid() % 30)`; launch `localization_with_nonground.launch.py` with `use_sim:=true`, `use_rviz:=false`, `publish_static_tf:=true` and the generated parameter file; then launch `ros2 bag play "${bag_dir}" --loop --clock 100 --disable-keyboard-controls`. Find the relocalization child PID by recursively reading the `children` file below the launch subprocess’s `/proc/${launch_pid}/task/${launch_pid}` directory, and sample only that process’s `/proc/${relocalization_pid}/status` every `sample_period_seconds` using `time.monotonic()`.

It must write these files even when a limit fails:

```text
rss.csv: elapsed_seconds,rss_mib
rss-summary.json: peak_rss_mib,slope_mib_per_hour,sample_count,
                  warmup_seconds,duration_seconds,exit_reason
```

After the configured duration, compare `peak_rss_mib <= max_rss_mib` and `slope_mib_per_hour <= max_slope_mib_per_hour`; exit 1 with both observed and allowed values if either fails. The collector must not subscribe to point clouds or retain any ROS messages.

- [ ] **Step 4: Run hermetic checks**

```bash
python3 -m py_compile test/integration/localization_rss.py
cd /home/robot/slam_ws
colcon build --packages-select lio_sam_hesai
ctest --test-dir build/lio_sam_hesai -R test_localization_stability_rss --output-on-failure
```

Expected: RSS tests pass without an external bag.

- [ ] **Step 5: Establish the Orin baseline only on the Orin runner**

Run a configured soak test on the Jetson Orin NX with the shared camera/planning workload enabled:

```bash
python3 test/integration/localization_rss.py \
  --bag /home/robot/rosbag2/hesai_sim \
  --map /home/robot/map/hesai_sim2 \
  --params-template config/relocalization.yaml \
  --baseline test/integration/baselines/hesai_sim.yaml \
  --output-dir /tmp/lio-sam-hesai-rss
```

Set the baseline’s `memory.max_rss_mib` and `memory.max_slope_mib_per_hour` only after reviewing its CSV and summary. Each limit must be strictly above the observed value and leave the configured 16GB shared-memory reserve; commit neither a desktop-derived limit nor a result with fewer than two post-warmup samples.

- [ ] **Step 6: Commit**

```bash
git add -f CMakeLists.txt test/integration/localization_rss.py \
  test/test_localization_stability_rss.py test/integration/baselines/hesai_sim.yaml
git commit -m "test(stability): add localization RSS soak checker"
```

### Task 5: Stable developer entrypoint, documentation and CI

**Files:**
- Create: `scripts/run_localization_stability.sh`
- Create: `test/integration/README.md`
- Create: `.github/workflows/localization-stability.yml`

- [ ] **Step 1: Add the shell entrypoint**

Create an executable `scripts/run_localization_stability.sh` with this command contract:

```bash
scripts/run_localization_stability.sh quick
scripts/run_localization_stability.sh record-baseline
scripts/run_localization_stability.sh memory
```

It must set `data_root=${LIO_SAM_TESTDATA_ROOT:-/home/robot}`, require `${data_root}/rosbag2/hesai_sim` and `${data_root}/map/hesai_sim2`, source `/opt/ros/humble/setup.bash` and `/home/robot/slam_ws/install/setup.bash`, create `mktemp -d /tmp/lio-sam-hesai-stability.XXXXXX`, and install `trap 'rm -rf "$results_dir"' EXIT` only when `LIO_SAM_KEEP_RESULTS` is not `1`. Dispatch `quick` to the normal collector, `record-baseline` to the collector with `--record-baseline`, and `memory` to the RSS collector. Any other mode prints usage and exits 64.

- [ ] **Step 2: Document the exact operator workflow**

Write `test/integration/README.md` with the three shell commands above; the required disk footprint; the relationship between `/home/robot/rosbag2/hesai_sim` and `/home/robot/map/hesai_sim2`; the expected result files; instructions to review a candidate baseline in a separate commit; and an explicit statement that only `record-baseline` may produce a candidate, while CI never updates baselines.

- [ ] **Step 3: Add self-hosted GitHub Actions workflow**

Create `.github/workflows/localization-stability.yml` with exactly two jobs:

```yaml
name: Localization stability
on:
  workflow_dispatch:
  schedule:
    - cron: "15 2 * * *"

jobs:
  quick:
    runs-on: [self-hosted, linux, arm64, orin]
    steps:
      - uses: actions/checkout@v4
      - run: test -d "${LIO_SAM_TESTDATA_ROOT}/rosbag2/hesai_sim"
      - run: test -d "${LIO_SAM_TESTDATA_ROOT}/map/hesai_sim2"
      - run: ./scripts/run_localization_stability.sh quick
      - uses: actions/upload-artifact@v4
        if: always()
        with: {name: localization-quick, path: /tmp/lio-sam-hesai-stability.*}
  memory:
    if: github.event_name == 'workflow_dispatch'
    runs-on: [self-hosted, linux, arm64, orin]
    steps:
      - uses: actions/checkout@v4
      - run: ./scripts/run_localization_stability.sh memory
      - uses: actions/upload-artifact@v4
        if: always()
        with: {name: localization-rss, path: /tmp/lio-sam-hesai-stability.*}
```

Do not add this workflow to a PR trigger. The test-data checks must fail rather than silently skip in CI.

- [ ] **Step 4: Verify static artifacts and existing tests**

Run:

```bash
bash -n scripts/run_localization_stability.sh
python3 - <<'PY'
import pathlib, yaml
yaml.safe_load(pathlib.Path('.github/workflows/localization-stability.yml').read_text())
PY
cd /home/robot/slam_ws
colcon build --symlink-install --packages-select lio_sam_hesai
ctest --test-dir build/lio_sam_hesai --output-on-failure \
  -E '^(copyright|cpplint|flake8|lint_cmake|uncrustify|xmllint)$'
```

Expected: shell/YAML syntax passes and all enabled CTest targets pass.

- [ ] **Step 5: Commit**

```bash
chmod +x scripts/run_localization_stability.sh
git add -f scripts/run_localization_stability.sh test/integration/README.md \
  .github/workflows/localization-stability.yml
git commit -m "ci: add scheduled localization stability regression"
```

### Task 6: End-to-end acceptance and review

**Files:**
- Modify only if a verification result exposes a concrete defect in Tasks 1–5.

- [ ] **Step 1: Execute the quick regression twice**

Run `scripts/run_localization_stability.sh quick` twice from a clean build. Preserve both result directories with `LIO_SAM_KEEP_RESULTS=1`. Expected: both runs exit zero and their observed metrics independently satisfy the committed `hesai_sim.yaml` bounds.

- [ ] **Step 2: Run the configured RSS acceptance test on Orin**

Run `scripts/run_localization_stability.sh memory` on the designated Orin with the shared workload. Expected: CSV has at least two post-warmup samples; summary stays within committed bounds; all child processes exit.

- [ ] **Step 3: Review the final diff and run complete verification**

Run:

```bash
git diff --check HEAD~5..HEAD
cd /home/robot/slam_ws
colcon build --symlink-install --packages-select lio_sam_hesai
ctest --test-dir build/lio_sam_hesai --output-on-failure \
  -E '^(copyright|cpplint|flake8|lint_cmake|uncrustify|xmllint)$'
```

Expected: no whitespace errors, successful build, and all enabled tests passing. Then request the required spec-compliance and code-quality subagent reviews before declaring the subproject complete.
