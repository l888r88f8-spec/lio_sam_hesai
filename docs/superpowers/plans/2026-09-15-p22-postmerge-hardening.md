# P2.2 Post-Merge Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the six correctness/contract defects left after the P2.2 fix and review rounds were merged into local `main`, then re-accept the round with unit tests, a serial clean build, the 44-test convention CTest gate, and `tiled-faults`.

**Architecture:** No new components. A dead duplicate config guard is removed and pinned; the tiled fault fixture gets one center contract; `MetricCollector`'s tracking gate becomes a current-state flag instead of a latch; map identity moves from byte counts to named-artifact SHA-256 digests carried inside `DatasetIdentity` (schema stays 1, legacy references fail closed); reference poses/timestamps get finite + rigid-transform validation on both record and load; and the `/initialpose` replay publishes only once a subscriber is matched, driven by non-blocking ticks.

**Tech Stack:** C++17 / rclcpp / ament_cmake_gtest / GTest, Python 3.10 `unittest`, rosbag2 replay harness, colcon (serial), CTest, flake8.

**Design:** `docs/superpowers/specs/2026-09-15-p22-postmerge-hardening-design.md`

---

## Preconditions

- Work in the existing worktree `/home/robot/slam_ws/src/lio_sam_hesai/.worktrees/p22-postmerge-fixes`.
- Branch `fix/p22-postmerge-hardening`, base `740dab9` (local `main` is the same commit).
- `docs/` and `test/` are listed in `.gitignore`, but both documents referenced here are already tracked and this round only edits tracked files, so no `git add -f` is needed.
- Build tree: `/tmp/p22_hardening` (its own `--build-base`/`--install-base`). It needs about 4.4 GB and `/tmp` has 7.7 GB. A from-scratch serial build takes about 28 minutes; incremental `cmake --build` runs take seconds to a couple of minutes.
- Do not touch the copyright job, the pre-existing repo-wide lint baseline (`E501`, `E402`, `E203`), P2.3, disk artifact names, or `docs/HANDOFF.md` before Task 8.
- `source /opt/ros/humble/setup.bash` before every build/CTest/replay command.

### Coverage of the approved order

| Approved item | Plan task |
| --- | --- |
| 1. duplicate `tile_size_m` guard + config guard test | Task 1 |
| 2. `prepare_tiled_fault_case(corrupt-manifest, center_index=None)` | Task 2 |
| 3. `MetricCollector` tracking gate stickiness | Task 3 |
| 4a. map identity → named-artifact SHA-256, schema stays 1 | Task 4 |
| 4b. reference NaN/Inf/rigid/time validation | Task 5 |
| 5. `/initialpose` matched-subscriber gate, bool callback | Task 6 |
| 6. unit tests, serial build, 44-test CTest, `tiled-faults` | Task 7 |
| 7. new E303 cleanup + HANDOFF re-acceptance | Task 8 |

## File map

- `src/common/slam_config.cpp` — remove the duplicate `tile_size_m` guard, fix indentation (Task 1).
- `test/test_slam_config.cpp` — new `MappingRejectsNonPositiveOrNonFiniteTileSize` (Task 1).
- `test/integration/stability_common.py` — fault center contract (Task 2); named-artifact identity (Task 4); reference validation helpers (Task 5).
- `test/test_tiled_map_integration.py` — corrupt-manifest center test (Task 2); E303 cleanup (Task 8).
- `test/integration/localization_regression.py` — tracking gate (Task 3); identity call sites (Task 4); `/initialpose` subscriber gate (Task 6).
- `test/test_localization_regression.py` — tracking gate tests (Task 3); identity/fail-closed tests (Task 4); reference validation tests (Task 5); subscriber-gate tests (Task 6).
- `test/test_localization_stability_common.py` — artifact identity tests (Task 4).
- `test/test_localization_stability_baseline.py` — `DatasetIdentity` construction (Task 4).
- `docs/HANDOFF.md` — merged status + §9.9 re-acceptance record (Task 8).

---

## Task 1: One `tile_size_m` guard, pinned by a config test

**Files:**
- Modify: `src/common/slam_config.cpp:765-773`
- Test: `test/test_slam_config.cpp` (insert after `MappingRejectsInvalidConsumedRuntimeParameters`, which ends at line 280)

- [ ] **Step 1: Write the guard test**

Insert after line 280 of `test/test_slam_config.cpp`:

```cpp
TEST(SlamConfig, MappingRejectsNonPositiveOrNonFiniteTileSize) {
  // tile_map_generator.cpp rejects a non-positive tile size when generation
  // runs; the configuration has to reject it up front instead of accepting a
  // config that can only fail later.
  for (const double tile_size :
       {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
    SCOPED_TRACE(tile_size);
    EXPECT_THROW(LoadMapping({rclcpp::Parameter(
                     "map_save.tiled_map.tile_size_m", tile_size)}),
                 std::invalid_argument);
  }
  const SlamConfig accepted = LoadMapping(
      {rclcpp::Parameter("map_save.tiled_map.tile_size_m", 25.0)});
  EXPECT_DOUBLE_EQ(accepted.map_save.tiled_map.tile_size_m, 25.0);
}
```

- [ ] **Step 2: Build once, then run the test and verify it passes**

This is the first build of the round; expect roughly 28 minutes.

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai/.worktrees/p22-postmerge-fixes
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/p22_hardening/log build --symlink-install \
  --packages-select lio_sam_hesai \
  --build-base /tmp/p22_hardening/build --install-base /tmp/p22_hardening/install \
  --cmake-args -DBUILD_TESTING=ON --executor sequential --parallel-workers 1
ctest --test-dir /tmp/p22_hardening/build/lio_sam_hesai \
  -R '^test_slam_config$' --output-on-failure
```

Expected: build exit code 0 (`Summary: 1 package finished`), then `1/1 Test #... Passed`. The test passes at this point because the guard already exists; this step pins the contract before the refactor.

- [ ] **Step 3: Prove the new test is not vacuous (mutation check)**

Replace the first guard in `src/common/slam_config.cpp`:

```cpp
  RequirePositive(map_save.tiled_map.tile_size_m,
                  "map_save.tiled_map.tile_size_m");
```

with `RequireFinite(map_save.tiled_map.tile_size_m, "map_save.tiled_map.tile_size_m");`, then:

```bash
cmake --build /tmp/p22_hardening/build/lio_sam_hesai --target test_slam_config -- -j1
ctest --test-dir /tmp/p22_hardening/build/lio_sam_hesai \
  -R '^test_slam_config$' --output-on-failure
```

Expected: FAIL, with `Expected: LoadMapping(...) throws an exception of type std::invalid_argument` for the `0` and `-1` sub-cases. Restore `RequirePositive` and re-run the same two commands; expected: PASS again.

- [ ] **Step 4: Remove the duplicate guard**

In `src/common/slam_config.cpp`, replace lines 765-773:

```cpp
  
  RequirePositive(map_save.tiled_map.tile_size_m,

                  "map_save.tiled_map.tile_size_m");
RequirePositive(system.mapping_process_interval,
                  "system.mapping_process_interval");
  RequirePositive(imu.gravity, "imu.gravity");
  RequireNonNegative(map_save.tiled_map.tile_size_m,
                     "map_save.tiled_map.tile_size_m");
  RequirePositive(map_save.tiled_map.overview_resolution_m,
                  "map_save.tiled_map.overview_resolution_m");
```

with:

```cpp
  RequirePositive(map_save.tiled_map.tile_size_m,
                  "map_save.tiled_map.tile_size_m");
  RequirePositive(system.mapping_process_interval,
                  "system.mapping_process_interval");
  RequirePositive(imu.gravity, "imu.gravity");
  RequirePositive(map_save.tiled_map.overview_resolution_m,
                  "map_save.tiled_map.overview_resolution_m");
```

- [ ] **Step 5: Re-run the test**

```bash
cmake --build /tmp/p22_hardening/build/lio_sam_hesai --target test_slam_config -- -j1
ctest --test-dir /tmp/p22_hardening/build/lio_sam_hesai \
  -R '^test_slam_config$' --output-on-failure
```

Expected: `1/1 Test #... Passed`. Confirm `grep -n "tile_size_m" src/common/slam_config.cpp` now shows exactly one `RequirePositive` and no `RequireNonNegative`.

- [ ] **Step 6: Commit**

```bash
git add src/common/slam_config.cpp test/test_slam_config.cpp
git commit -m "fix(config): keep one tile_size_m guard and pin it"
```

---

## Task 2: Every tiled fault case requires a center

**Files:**
- Modify: `test/integration/stability_common.py:397-411`
- Test: `test/test_tiled_map_integration.py` (insert after `test_fault_fixture_requires_a_declared_center`, which ends at line 195)

- [ ] **Step 1: Write the failing test**

Insert after line 195 of `test/test_tiled_map_integration.py`:

```python
    def test_corrupt_manifest_requires_a_center(self):
        # A None center used to reach the tuple() below and raise TypeError
        # instead of the documented DatasetError.
        self._write_small_tiled_map()
        with self.assertRaises(DatasetError):
            prepare_tiled_fault_case(
                self.tiled, self.root / "fault-none", "corrupt-manifest",
                center_index=None)
        self.assertFalse((self.root / "fault-none").exists())
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai/.worktrees/p22-postmerge-fixes
python3 test/test_tiled_map_integration.py \
  TiledMapIntegrationTest.test_corrupt_manifest_requires_a_center
```

Expected: ERROR, `TypeError: 'NoneType' object is not iterable` raised by `stability_common.py:430`.

- [ ] **Step 3: Make the center unconditional**

In `test/integration/stability_common.py`, replace:

```python
    if center_index is None and fault_case != "corrupt-manifest":
        raise DatasetError(
            "Tiled fault center index is required; pass the replay reference tile"
        )
```

with:

```python
    if center_index is None:
        raise DatasetError(
            "Tiled fault center index is required; pass the replay "
            "reference tile"
        )
```

and extend the docstring above the checks:

```python
    `center_index` is required for every fault case, `corrupt-manifest`
    included: the map's first tile says nothing about where the replay actually
    drives, and using it produced fixtures the trajectory never reached. The
    center is validated against the manifest before anything is written.
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
python3 test/test_tiled_map_integration.py \
  TiledMapIntegrationTest.test_corrupt_manifest_requires_a_center
python3 test/test_tiled_map_integration.py
```

Expected: the focused run is `OK` (1 test), the full file is `OK`.

- [ ] **Step 5: Commit**

```bash
git add test/integration/stability_common.py test/test_tiled_map_integration.py
git commit -m "fix(integration): require a tiled fault center for every case"
```

---

## Task 3: The tracking gate reflects the current state

**Files:**
- Modify: `test/integration/localization_regression.py:191-200`
- Test: `test/test_localization_regression.py` (`MetricCollectorTest`, insert after `test_invalid_quaternion_does_not_count_as_valid_odom`, which ends at line 130)

- [ ] **Step 1: Write the failing tests**

Insert after line 130 of `test/test_localization_regression.py`:

```python
    def test_capture_pose_gate_closes_again_on_a_warning(self):
        collector = MetricCollector(capture_pose=True)

        collector.clock(100.0)
        collector.diagnostic(
            101.0, 0, "lio_sam_hesai: localization", {"is_tracking": "true"})
        collector.diagnostic(
            102.0, 1, "lio_sam_hesai: localization", {"is_tracking": "true"})
        collector.odom(103.0, 0, 0, 0, 0, 0, 0, 1)

        self.assertIsNone(collector.captured_pose())

    def test_capture_pose_gate_closes_when_is_tracking_is_not_true(self):
        collector = MetricCollector(capture_pose=True)

        collector.clock(100.0)
        collector.diagnostic(
            101.0, 0, "lio_sam_hesai: localization", {"is_tracking": "true"})
        collector.diagnostic(
            102.0, 0, "lio_sam_hesai: localization", {"is_tracking": "false"})
        collector.odom(103.0, 0, 0, 0, 0, 0, 0, 1)

        self.assertIsNone(collector.captured_pose())

    def test_capture_pose_records_only_while_tracking_is_confirmed(self):
        collector = MetricCollector(capture_pose=True)

        collector.clock(100.0)
        collector.diagnostic(101.0, 2, "lio_sam_hesai: localization", {})
        collector.odom(101.5, 0, 0, 0, 0, 0, 0, 1)
        self.assertIsNone(collector.captured_pose())

        collector.diagnostic(
            102.0, 0, "lio_sam_hesai: localization", {"is_tracking": "true"})
        collector.odom(102.5, 1, 2, 3, 0, 0, 0, 1)
        self.assertIsNotNone(collector.captured_pose())
        self.assertEqual(collector.captured_pose()[0], 102.5)
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai/.worktrees/p22-postmerge-fixes
PYTHONPATH=test/integration python3 test/test_localization_regression.py \
  MetricCollectorTest.test_capture_pose_gate_closes_again_on_a_warning \
  MetricCollectorTest.test_capture_pose_gate_closes_when_is_tracking_is_not_true \
  MetricCollectorTest.test_capture_pose_records_only_while_tracking_is_confirmed
```

Expected: FAIL for the first two (`0 != None : Odometry was captured` style assertion errors) because `_tracking_confirmed` latches; the third already passes.

- [ ] **Step 3: Make the gate a current-state flag**

In `test/integration/localization_regression.py`, replace lines 191-200:

```python
        if numeric_level == 0:
            if self._first_ok_seconds is None:
                self._first_ok_seconds = timestamp
            # Healthy tracking, not just "a diagnostic arrived".
            if isinstance(values, Mapping) and \
                    str(values.get("is_tracking", "")).lower() == "true":
                self._tracking_confirmed = True
        else:
            if numeric_level == 2:
                self._error_samples += 1
```

with:

```python
        if numeric_level == 0 and self._first_ok_seconds is None:
            self._first_ok_seconds = timestamp
        if numeric_level == 2:
            self._error_samples += 1
        # The gate describes the *current* tracking state, not "tracking was
        # seen once": a later WARN/ERROR or an is_tracking != true sample must
        # close it again, or a pose captured after tracking was lost would be
        # recorded as the stable reference pose.
        self._tracking_confirmed = (
            numeric_level == 0 and
            isinstance(values, Mapping) and
            str(values.get("is_tracking", "")).lower() == "true"
        )
```

Also update `captured_pose()`'s docstring (lines 166-171) to:

```python
        """
        (bag_seconds, map_T_base) of the first valid tracking sample, or None.

        Tracking is confirmed by the most recent decodable target diagnostic,
        so an initialization sample published with is_tracking=false is never
        mistaken for a stable pose. The timestamp is the replay clock, which is
        the bag's own time base -- an absolute value near 1.8e9, not an offset
        into the bag.
        """
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
PYTHONPATH=test/integration python3 test/test_localization_regression.py
```

Expected: `OK` (the whole file, including the new gate tests).

- [ ] **Step 5: Commit**

```bash
git add test/integration/localization_regression.py test/test_localization_regression.py
git commit -m "fix(integration): unstick the reference tracking gate"
```

---

## Task 4: Map identity is named-artifact SHA-256

**Files:**
- Modify: `test/integration/stability_common.py` (constants, `DatasetIdentity`, `validate_dataset`, `_dataset_identity_payload`, delete `map_fingerprint`, `make_tiled_fault_reference`, `load_tiled_fault_reference`)
- Modify: `test/integration/localization_regression.py` (import + two call sites)
- Modify: `test/test_localization_stability_common.py`
- Modify: `test/test_localization_regression.py`
- Modify: `test/test_localization_stability_baseline.py`

- [ ] **Step 1: Write the failing identity tests**

In `test/test_localization_stability_common.py`, add `import hashlib` to the imports at the top (before `import signal`), then replace the two `map_bytes` assertions and add three tests.

Replace lines 80-81:

```python
        self.assertEqual(identity.map_points, 4)
        self.assertEqual(identity.map_bytes, (self.map_dir / GLOBAL_MAP_NAME).stat().st_size)
```

with:

```python
        self.assertEqual(identity.map_points, 4)
        self.assertEqual(
            dict(identity.map_artifacts),
            {
                GLOBAL_MAP_NAME: hashlib.sha256(
                    (self.map_dir / GLOBAL_MAP_NAME).read_bytes()).hexdigest(),
                GROUND_MAP_NAME: hashlib.sha256(b"ground map").hexdigest(),
                SCAN_CONTEXT_DB_NAME: hashlib.sha256(
                    b"scan context").hexdigest(),
            },
        )
        with self.assertRaises(TypeError):
            identity.map_artifacts[GLOBAL_MAP_NAME] = "0" * 64
```

Replace lines 90-91:

```python
        self.assertEqual(identity.map_points, 7)
        self.assertEqual(identity.map_bytes, global_map.stat().st_size)
```

with:

```python
        self.assertEqual(identity.map_points, 7)
        self.assertEqual(
            identity.map_artifacts[GLOBAL_MAP_NAME],
            hashlib.sha256(global_map.read_bytes()).hexdigest())
```

Insert after `test_validate_dataset_exposes_immutable_topic_counts` (which ends at line 125):

```python
    def test_map_identity_distinguishes_same_size_artifacts(self):
        before = validate_dataset(self.bag_dir, self.map_dir)
        (self.map_dir / GROUND_MAP_NAME).write_bytes(b"6round map")

        after = validate_dataset(self.bag_dir, self.map_dir)

        self.assertEqual((self.map_dir / GROUND_MAP_NAME).stat().st_size,
                         len(b"ground map"))
        self.assertNotEqual(before.map_artifacts[GROUND_MAP_NAME],
                            after.map_artifacts[GROUND_MAP_NAME])

    def test_map_identity_includes_the_tiled_manifest(self):
        tiled = self.map_dir / "tiled"
        tiled.mkdir()
        manifest = b"schema_version: 1\n"
        (tiled / "manifest.yaml").write_bytes(manifest)

        identity = validate_dataset(
            self.bag_dir, self.map_dir, tiled_dir=tiled)

        self.assertEqual(identity.map_artifacts["tiled/manifest.yaml"],
                         hashlib.sha256(manifest).hexdigest())
```

- [ ] **Step 2: Run the identity tests to verify they fail**

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai/.worktrees/p22-postmerge-fixes
PYTHONPATH=test/integration python3 test/test_localization_stability_common.py
```

Expected: ERROR/FAIL, `AttributeError: 'DatasetIdentity' object has no attribute 'map_artifacts'`.

- [ ] **Step 3: Implement the artifact identity**

In `test/integration/stability_common.py`, replace line 30:

```python
_REQUIRED_MAP_FILES = ("GlobalMap.pcd", "GroundMap.pcd", "sc_db.bin")
```

with:

```python
_REQUIRED_MAP_FILES = ("GlobalMap.pcd", "GroundMap.pcd", "sc_db.bin")
# The tiled manifest is named relative to the map root, not by its local
# path, so the identity is stable across machines.
_TILED_MANIFEST_ARTIFACT = "tiled/manifest.yaml"
_ARTIFACT_CHUNK_BYTES = 1024 * 1024
```

Replace the `DatasetIdentity` dataclass (lines 20-27):

```python
@dataclass(frozen=True)
class DatasetIdentity:
    duration_ns: int
    message_count: int
    topic_counts: Mapping[str, int]
    db3_bytes: int
    map_points: int
    map_artifacts: Mapping[str, str]
```

Insert the digest helpers immediately after the dataclass:

```python
def _artifact_sha256(path):
    """Return the lowercase SHA-256 of one map artifact."""
    digest = hashlib.sha256()
    try:
        with Path(path).open("rb") as artifact:
            for chunk in iter(lambda: artifact.read(_ARTIFACT_CHUNK_BYTES),
                              b""):
                digest.update(chunk)
    except OSError as error:
        raise DatasetError(
            f"Unable to hash map artifact {path}: {error}") from error
    return digest.hexdigest()


def map_artifacts(map_root, tiled_dir=None):
    """
    Return artifact name -> SHA-256 for every file the map identity covers.

    A byte count is not an identity: two different maps of the same size would
    compare equal. The names are canonical paths relative to the map root, so
    the same map hashes the same way on every machine and through a symlink.
    """
    map_root = Path(map_root)
    artifacts = {name: map_root / name for name in _REQUIRED_MAP_FILES}
    if tiled_dir is not None:
        artifacts[_TILED_MANIFEST_ARTIFACT] = Path(tiled_dir) / "manifest.yaml"
    return MappingProxyType(
        {name: _artifact_sha256(path) for name, path in artifacts.items()})
```

In `validate_dataset`, hoist the tiled path and compute the digests. Replace lines 41-54:

```python
def validate_dataset(bag_dir: Path, map_dir: Path, *, tiled_dir=None) -> DatasetIdentity:
    bag_path = Path(bag_dir)
    map_path = Path(map_dir)
    _require_directory(bag_path, "Bag")
    _require_directory(map_path, "Map")
    if tiled_dir is not None:
        tiled_path = Path(tiled_dir)
        _require_directory(tiled_path, "Tiled map")
        resolved_map = map_path.resolve()
        resolved_tiled = tiled_path.resolve()
```

with:

```python
def validate_dataset(bag_dir: Path, map_dir: Path, *, tiled_dir=None) -> DatasetIdentity:
    bag_path = Path(bag_dir)
    map_path = Path(map_dir)
    tiled_path = Path(tiled_dir) if tiled_dir is not None else None
    _require_directory(bag_path, "Bag")
    _require_directory(map_path, "Map")
    if tiled_path is not None:
        _require_directory(tiled_path, "Tiled map")
        resolved_map = map_path.resolve()
        resolved_tiled = tiled_path.resolve()
```

Replace lines 77-92:

```python
    required_maps = _required_map_files(map_path)
    global_map = required_maps["GlobalMap.pcd"]
    try:
        db3_bytes = db3_files[0].stat().st_size
        map_bytes = global_map.stat().st_size
    except OSError as error:
        raise DatasetError(f"Unable to stat dataset file: {error}") from error

    return DatasetIdentity(
        duration_ns=duration_ns,
        message_count=message_count,
        topic_counts=MappingProxyType(topic_counts),
        db3_bytes=db3_bytes,
        map_points=_pcd_points(global_map),
        map_bytes=map_bytes,
    )
```

with:

```python
    required_maps = _required_map_files(map_path)
    global_map = required_maps["GlobalMap.pcd"]
    try:
        db3_bytes = db3_files[0].stat().st_size
    except OSError as error:
        raise DatasetError(f"Unable to stat dataset file: {error}") from error

    return DatasetIdentity(
        duration_ns=duration_ns,
        message_count=message_count,
        topic_counts=MappingProxyType(topic_counts),
        db3_bytes=db3_bytes,
        map_points=_pcd_points(global_map),
        map_artifacts=map_artifacts(map_path, tiled_path),
    )
```

Replace `_dataset_identity_payload` (lines 775-784):

```python
def _dataset_identity_payload(identity):
    return {
        "duration_ns": int(identity.duration_ns),
        "message_count": int(identity.message_count),
        "topic_counts": {str(k): int(v)
                         for k, v in dict(identity.topic_counts).items()},
        "db3_bytes": int(identity.db3_bytes),
        "map_points": int(identity.map_points),
        "artifacts": {str(name): str(digest) for name, digest in
                      dict(identity.map_artifacts).items()},
    }
```

Delete `map_fingerprint` (lines 787-814) entirely and replace `make_tiled_fault_reference` (lines 834-851) with:

```python
def make_tiled_fault_reference(identity, bag_seconds, map_T_base,
                               warmup_seconds=10.0):
    map_T_base = _require_matrix4(map_T_base, "map_T_base")
    if not math.isfinite(bag_seconds) or bag_seconds < 0.0:
        raise DatasetError(
            "reference bag time must be finite and non-negative")
    if warmup_seconds <= 0.0:
        raise DatasetError("reference warmup must be positive")
    offset = max(0.0, bag_seconds - warmup_seconds)
    return TiledFaultReference(
        dataset=_dataset_identity_payload(identity),
        bag_seconds=float(bag_seconds),
        map_T_base=tuple(tuple(row) for row in map_T_base),
        offset_seconds=offset,
        warmup_seconds=float(warmup_seconds),
    )
```

Replace the whole `load_tiled_fault_reference` function (lines 869-901) with:

```python
def load_tiled_fault_reference(path, identity):
    path = Path(path)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise DatasetError(f"Unable to read tiled fault reference {path}: {error}") from error
    payload = _require_mapping(payload, "tiled fault reference")
    if payload.get("schema_version") != _TILE_INDEX_SCHEMA_VERSION:
        raise DatasetError(
            f"Unsupported tiled fault reference schema: {payload.get('schema_version')!r}")

    recorded = _require_mapping(payload.get("dataset"), "reference dataset")
    expected = _dataset_identity_payload(identity)
    for key, value in expected.items():
        if recorded.get(key) != value:
            raise DatasetError(
                f"Tiled fault reference was recorded against a different dataset "
                f"({key}: {recorded.get(key)!r} != {value!r}); recapture it")
    return TiledFaultReference(
        dataset=recorded,
        bag_seconds=float(payload["bag_seconds"]),
        map_T_base=tuple(tuple(row) for row in
                         _require_matrix4(payload["map_T_base"], "map_T_base")),
        offset_seconds=float(payload.get("offset_seconds", 0.0)),
        warmup_seconds=float(payload.get("warmup_seconds", 10.0)),
    )
```

- [ ] **Step 4: Update the call sites and the tests that used the size fields**

In `test/integration/localization_regression.py`:

Remove `map_fingerprint,` from the `from stability_common import (...)` block (line 29).

Replace the fault-reference load (lines 541-543):

```python
                    load_tiled_fault_reference(
                        arguments.tiled_fault_reference, identity,
                        map_fingerprint(arguments.map, arguments.tiled_dir)),
```

with:

```python
                    load_tiled_fault_reference(
                        arguments.tiled_fault_reference, identity),
```

Replace the reference capture (lines 853-859):

```python
            save_tiled_fault_reference(
                arguments.record_reference,
                make_tiled_fault_reference(
                    identity, bag_seconds=bag_seconds,
                    map_T_base=captured[1],
                    map_fingerprint_value=map_fingerprint(
                        arguments.map, arguments.tiled_dir)))
```

with:

```python
            save_tiled_fault_reference(
                arguments.record_reference,
                make_tiled_fault_reference(
                    identity, bag_seconds=bag_seconds,
                    map_T_base=captured[1]))
```

In `test/test_localization_regression.py`, replace the `_identity` helper (lines 746-752) with:

```python
    def _identity(self, **overrides):
        base = {
            "duration_ns": 100, "message_count": 10, "topic_counts": {},
            "db3_bytes": 1000, "map_points": 50,
            "map_artifacts": {"GlobalMap.pcd": "a" * 64,
                              "GroundMap.pcd": "b" * 64,
                              "sc_db.bin": "c" * 64},
        }
        base.update(overrides)
        return SimpleNamespace(**base)
```

Replace the overrides in `test_reference_refuses_a_different_dataset` (lines 833-834):

```python
            for overrides in ({"map_bytes": 999}, {"message_count": 11},
                              {"db3_bytes": 1}):
```

with:

```python
            for overrides in (
                    {"map_artifacts": {"GlobalMap.pcd": "d" * 64,
                                       "GroundMap.pcd": "b" * 64,
                                       "sc_db.bin": "c" * 64}},
                    {"message_count": 11},
                    {"db3_bytes": 1}):
```

Insert the fail-closed test after `test_reference_refuses_a_different_dataset` (which ends at line 838):

```python
    def test_reference_rejects_a_legacy_size_only_identity(self):
        # schema_version stays 1, so there is no migration branch: an old
        # reference has no "artifacts" key and must fail closed instead of
        # matching a map that happens to have the same byte counts.
        with TemporaryDirectory() as directory:
            path = Path(directory) / "reference.json"
            save_tiled_fault_reference(
                path,
                make_tiled_fault_reference(
                    self._identity(), bag_seconds=1.0,
                    map_T_base=[[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]))
            payload = json.loads(path.read_text(encoding="utf-8"))
            dataset = payload["dataset"]
            self.assertIn("artifacts", dataset)
            del dataset["artifacts"]
            dataset["map_bytes"] = 500
            dataset["ground_map_bytes"] = 10
            dataset["sc_db_bytes"] = 12
            path.write_text(json.dumps(payload), encoding="utf-8")

            with self.assertRaisesRegex(DatasetError, "recapture"):
                load_tiled_fault_reference(path, self._identity())
```

In `test/test_localization_stability_baseline.py`, replace `map_bytes=1,` (line 355) with:

```python
            map_artifacts=MappingProxyType(
                {"GlobalMap.pcd": "a" * 64, "GroundMap.pcd": "b" * 64,
                 "sc_db.bin": "c" * 64}),
```

- [ ] **Step 5: Run the affected tests to verify they pass**

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai/.worktrees/p22-postmerge-fixes
PYTHONPATH=test/integration python3 test/test_localization_stability_common.py
PYTHONPATH=test/integration python3 test/test_localization_regression.py
PYTHONPATH=test/integration python3 test/test_localization_stability_baseline.py
python3 test/test_tiled_map_integration.py
```

Expected: `OK` for all four; `grep -rn "map_fingerprint\|\.map_bytes" test/` reports no remaining identity uses (the `global_map_bytes` fields in the tiled manifest fixture are unrelated manifest metadata).

- [ ] **Step 6: Commit**

```bash
git add test/integration/stability_common.py test/integration/localization_regression.py \
        test/test_localization_stability_common.py test/test_localization_regression.py \
        test/test_localization_stability_baseline.py
git commit -m "fix(integration): identify map artifacts by sha256"
```

---

## Task 5: Reference poses and timestamps are validated

**Files:**
- Modify: `test/integration/stability_common.py` (`_require_matrix4`, new helpers, `make_tiled_fault_reference`, `load_tiled_fault_reference`)
- Test: `test/test_localization_regression.py`

- [ ] **Step 1: Write the failing tests**

In `test/test_localization_regression.py`, insert after the imports (line 23):

```python
_TILED_FAULT_IDENTITY_POSE = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]
```

Inside `TiledFaultReferenceTest`, insert these helpers after `_identity` (which ends at line 756):

```python
    def _valid_reference(self, path, **overrides):
        reference = make_tiled_fault_reference(
            self._identity(),
            bag_seconds=overrides.pop("bag_seconds", 1.0),
            map_T_base=overrides.pop("map_T_base", _TILED_FAULT_IDENTITY_POSE),
            **overrides)
        save_tiled_fault_reference(path, reference)
        return path

    def _rewrite_reference(self, path, **overrides):
        payload = json.loads(path.read_text(encoding="utf-8"))
        payload.update(overrides)
        path.write_text(json.dumps(payload), encoding="utf-8")
```

Insert the validation tests after `test_reference_rejects_an_unknown_schema` (which ends at line 845):

```python
    def test_make_reference_rejects_a_non_rigid_pose(self):
        with self.assertRaises(DatasetError):
            make_tiled_fault_reference(
                self._identity(), bag_seconds=1.0,
                map_T_base=[[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 2.0]])

    def test_reference_rejects_a_non_finite_pose(self):
        with TemporaryDirectory() as directory:
            path = self._valid_reference(Path(directory) / "reference.json")
            for row, column in ((0, 3), (2, 1)):
                with self.subTest(row=row, column=column):
                    pose = [list(values)
                            for values in _TILED_FAULT_IDENTITY_POSE]
                    pose[row][column] = math.nan
                    self._rewrite_reference(path, map_T_base=pose)
                    with self.assertRaises(DatasetError):
                        load_tiled_fault_reference(path, self._identity())

    def test_reference_rejects_a_non_rigid_pose(self):
        with TemporaryDirectory() as directory:
            path = self._valid_reference(Path(directory) / "reference.json")
            invalid = {
                "scaled": [[2.0, 0.0, 0.0, 0.0], [0.0, 2.0, 0.0, 0.0],
                           [0.0, 0.0, 2.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
                "reflected": [[-1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                              [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
                "bottom row": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                               [0.0, 0.0, 1.0, 0.0], [0.5, 0.0, 0.0, 1.0]],
            }
            for name, pose in invalid.items():
                with self.subTest(name=name):
                    self._rewrite_reference(path, map_T_base=pose)
                    with self.assertRaises(DatasetError):
                        load_tiled_fault_reference(path, self._identity())

    def test_reference_rejects_invalid_time_fields(self):
        with TemporaryDirectory() as directory:
            path = self._valid_reference(Path(directory) / "reference.json")
            invalid = {
                "bag_seconds NaN": {"bag_seconds": math.nan},
                "bag_seconds negative": {"bag_seconds": -1.0},
                "offset_seconds NaN": {"offset_seconds": math.nan},
                "offset beyond bag": {"bag_seconds": 5.0,
                                      "offset_seconds": 6.0},
                "warmup NaN": {"warmup_seconds": math.nan},
                "warmup zero": {"warmup_seconds": 0.0},
            }
            for name, overrides in invalid.items():
                with self.subTest(name=name):
                    self._rewrite_reference(path, **overrides)
                    with self.assertRaises(DatasetError):
                        load_tiled_fault_reference(path, self._identity())
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai/.worktrees/p22-postmerge-fixes
PYTHONPATH=test/integration python3 test/test_localization_regression.py \
  TiledFaultReferenceTest
```

Expected: the four new tests FAIL — non-finite, non-rigid, and invalid-time payloads load without raising; `test_make_reference_rejects_a_non_rigid_pose` records the bad matrix.

- [ ] **Step 3: Implement the shared validators**

In `test/integration/stability_common.py`, insert after `_require_nonnegative_int` (which ends at line 652):

```python
def _require_finite_number(value, name, *, minimum=None,
                           exclusive_minimum=False):
    """Return a finite float, rejecting bools and non-numeric values."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise DatasetError(f"{name} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise DatasetError(f"{name} must be a finite number")
    if minimum is not None:
        if exclusive_minimum and number <= minimum:
            raise DatasetError(f"{name} must be greater than {minimum}")
        if not exclusive_minimum and number < minimum:
            raise DatasetError(f"{name} must be at least {minimum}")
    return number
```

Replace `_require_matrix4` (lines 716-724) with a finite-only version plus the rigid variant:

```python
_RIGID_TOLERANCE = 1e-6


def _require_matrix4(value, name):
    if (not isinstance(value, (list, tuple)) or len(value) != _TILE_MATRIX_ROWS):
        raise DatasetError(f"{name} must be a 4x4 matrix")
    rows = []
    for row in value:
        if not isinstance(row, (list, tuple)) or len(row) != _TILE_MATRIX_ROWS:
            raise DatasetError(f"{name} must be a 4x4 matrix")
        entries = []
        for entry in row:
            if isinstance(entry, bool) or not isinstance(entry, (int, float)):
                raise DatasetError(f"{name} must contain only numbers")
            number = float(entry)
            if not math.isfinite(number):
                raise DatasetError(f"{name} must contain only finite entries")
            entries.append(number)
        rows.append(entries)
    return rows


def _require_rigid_matrix4(value, name):
    rows = _require_matrix4(value, name)
    if any(abs(rows[3][column] - expected) > _RIGID_TOLERANCE
           for column, expected in enumerate((0.0, 0.0, 0.0, 1.0))):
        raise DatasetError(
            f"{name} must be a rigid transform with a [0, 0, 0, 1] bottom row")
    rotation = [[rows[row][column] for column in range(3)] for row in range(3)]
    for first in range(3):
        for second in range(3):
            product = sum(rotation[k][first] * rotation[k][second]
                          for k in range(3))
            expected = 1.0 if first == second else 0.0
            if abs(product - expected) > _RIGID_TOLERANCE:
                raise DatasetError(f"{name} rotation must be orthonormal")
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] -
                          rotation[1][2] * rotation[2][1]) -
        rotation[0][1] * (rotation[1][0] * rotation[2][2] -
                          rotation[1][2] * rotation[2][0]) +
        rotation[0][2] * (rotation[1][0] * rotation[2][1] -
                          rotation[1][1] * rotation[2][0]))
    if abs(determinant - 1.0) > _RIGID_TOLERANCE:
        raise DatasetError(f"{name} rotation must be right-handed (det +1)")
    return rows
```

- [ ] **Step 4: Use the validators in record and load**

In `make_tiled_fault_reference`, replace the body written in Task 4:

```python
def make_tiled_fault_reference(identity, bag_seconds, map_T_base,
                               warmup_seconds=10.0):
    map_T_base = _require_matrix4(map_T_base, "map_T_base")
    if not math.isfinite(bag_seconds) or bag_seconds < 0.0:
        raise DatasetError(
            "reference bag time must be finite and non-negative")
    if warmup_seconds <= 0.0:
        raise DatasetError("reference warmup must be positive")
    offset = max(0.0, bag_seconds - warmup_seconds)
    return TiledFaultReference(
        dataset=_dataset_identity_payload(identity),
        bag_seconds=float(bag_seconds),
        map_T_base=tuple(tuple(row) for row in map_T_base),
        offset_seconds=offset,
        warmup_seconds=float(warmup_seconds),
    )
```

with:

```python
def make_tiled_fault_reference(identity, bag_seconds, map_T_base,
                               warmup_seconds=10.0):
    map_T_base = _require_rigid_matrix4(map_T_base, "map_T_base")
    bag_seconds = _require_finite_number(
        bag_seconds, "reference bag time", minimum=0.0)
    warmup_seconds = _require_finite_number(
        warmup_seconds, "reference warmup", minimum=0.0,
        exclusive_minimum=True)
    return TiledFaultReference(
        dataset=_dataset_identity_payload(identity),
        bag_seconds=bag_seconds,
        map_T_base=tuple(tuple(row) for row in map_T_base),
        offset_seconds=max(0.0, bag_seconds - warmup_seconds),
        warmup_seconds=warmup_seconds,
    )
```

In `load_tiled_fault_reference`, replace the tail written in Task 4:

```python
    return TiledFaultReference(
        dataset=recorded,
        bag_seconds=float(payload["bag_seconds"]),
        map_T_base=tuple(tuple(row) for row in
                         _require_matrix4(payload["map_T_base"], "map_T_base")),
        offset_seconds=float(payload.get("offset_seconds", 0.0)),
        warmup_seconds=float(payload.get("warmup_seconds", 10.0)),
    )
```

with:

```python
    bag_seconds = _require_finite_number(
        payload.get("bag_seconds"), "reference bag_seconds", minimum=0.0)
    offset_seconds = _require_finite_number(
        payload.get("offset_seconds", 0.0), "reference offset_seconds",
        minimum=0.0)
    warmup_seconds = _require_finite_number(
        payload.get("warmup_seconds", 10.0), "reference warmup_seconds",
        minimum=0.0, exclusive_minimum=True)
    if offset_seconds > bag_seconds:
        raise DatasetError(
            "reference offset_seconds must not exceed bag_seconds")
    return TiledFaultReference(
        dataset=recorded,
        bag_seconds=bag_seconds,
        map_T_base=tuple(tuple(row) for row in
                         _require_rigid_matrix4(payload.get("map_T_base"),
                                                "map_T_base")),
        offset_seconds=offset_seconds,
        warmup_seconds=warmup_seconds,
    )
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
PYTHONPATH=test/integration python3 test/test_localization_regression.py
PYTHONPATH=test/integration python3 test/test_localization_stability_common.py
python3 test/test_tiled_map_integration.py
```

Expected: `OK` for all three; `test_reference_round_trips_and_records_the_offset` and `test_reference_refuses_a_different_dataset` still pass unchanged.

- [ ] **Step 6: Commit**

```bash
git add test/integration/stability_common.py test/test_localization_regression.py
git commit -m "fix(integration): validate reference poses and timestamps"
```

---

## Task 6: Wait for a matched `/initialpose` subscriber

**Files:**
- Modify: `test/integration/localization_regression.py:613-689` (`TiledFaultRun`) and `:816-827` (`fault_tick`)
- Test: `test/test_localization_regression.py` (`TiledFaultRunTest`)

- [ ] **Step 1: Write the failing tests**

In `test/test_localization_regression.py`, insert after `test_publishes_at_the_reference_time_once_the_imu_is_ready` (which ends at line 931):

```python
    def test_waits_for_a_matched_subscriber_before_publishing(self):
        run, _fixture, published, publish = self._run()
        run.observe_diagnostics(self._status({
            "map_tile_load_state": "ready", "map_tile_load_failures": "0"}))
        run.tick(0.0, publish)
        run.observe_diagnostics(self._status({"has_imu_init": "true"}))
        attempts = []

        def no_subscriber_yet():
            attempts.append(1)
            return False

        self.assertEqual(run.tick(10.0, no_subscriber_yet),
                         "waiting-for-subscriber")
        self.assertEqual(attempts, [1])
        self.assertFalse(run.published)
        self.assertIsNone(run.published_at)
        # The executor keeps ticking, so a match found later still publishes
        # the pose inside the tolerance window.
        self.assertEqual(run.tick(10.2, publish), "published")
        self.assertTrue(run.published)
        self.assertEqual(run.published_at, 10.2)
        self.assertEqual(len(published), 1)

    def test_reports_the_missing_subscriber_once_the_window_expires(self):
        run, _fixture, published, publish = self._run()
        run.observe_diagnostics(self._status({
            "map_tile_load_state": "ready", "map_tile_load_failures": "0"}))
        run.tick(0.0, publish)
        run.observe_diagnostics(self._status({"has_imu_init": "true"}))
        run.tick(10.0, lambda: False)

        with self.assertRaisesRegex(
                localization_regression.ReplayError, "subscriber"):
            run.tick(11.5, lambda: False)
        self.assertFalse(run.published)
        self.assertEqual(published, [])
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai/.worktrees/p22-postmerge-fixes
PYTHONPATH=test/integration python3 test/test_localization_regression.py \
  TiledFaultRunTest.test_waits_for_a_matched_subscriber_before_publishing \
  TiledFaultRunTest.test_reports_the_missing_subscriber_once_the_window_expires
```

Expected: FAIL — the first test gets `"published"` instead of `"waiting-for-subscriber"`; the second does not raise.

- [ ] **Step 3: Implement the subscriber gate**

In `test/integration/localization_regression.py`, add a fourth bullet to the `TiledFaultRun` docstring (after the IMU bullet):

```python
    * the pose is published only once a matched `/initialpose` subscriber
      exists, because a publish with no subscription is dropped silently.
```

Add the flag in `__init__` (lines 634-637):

```python
        self.injected = False
        self.published = False
        self.published_at = None
        self.missing_subscriber = False
        self.values = {}
```

Replace `tick` (lines 666-689) with:

```python
    def tick(self, clock_seconds, publish_pose):
        if not self.injected:
            if not self.prepared():
                return "waiting-for-prepare"
            self.fixture.inject()
            self.injected = True
            return "injected"
        if self.published or clock_seconds is None:
            return "waiting-for-reference"
        if clock_seconds < self.reference_offset:
            return "warming-up"
        if not self.imu_initialized():
            raise ReplayError(
                "IMU was not initialized by the reference pose time; the replay "
                "cannot seed a pose the node would reject")
        if clock_seconds > self.reference_offset + self.tolerance_seconds:
            if self.missing_subscriber:
                raise ReplayError(
                    "the reference pose was never published: no /initialpose "
                    "subscriber matched the publisher within the reference "
                    "window")
            raise ReplayError(
                f"reference pose time {self.reference_offset:.2f}s after the "
                f"replay clock starts was missed (now {clock_seconds:.2f}s); "
                "refusing to seed a stale pose")
        # Only an explicit True from the callback counts as published: a
        # publish() with no matched subscription is dropped by the middleware,
        # so retry from later ticks instead of blocking the executor with a
        # sleep or wait_for_all_acked().
        if publish_pose() is not True:
            self.missing_subscriber = True
            return "waiting-for-subscriber"
        self.published = True
        self.published_at = clock_seconds
        return "published"
```

Replace the `fault_tick` construction in `_run_replay` (lines 816-827) with:

```python
        fault_tick = None
        if fault is not None:
            clock_origin = [None]

            def publish_reference_pose():
                # get_subscription_count() is a local, non-blocking query:
                # without a matched subscriber the middleware drops the
                # message, so defer to the next tick.
                if initial_pose_publisher.get_subscription_count() <= 0:
                    return False
                initial_pose_publisher.publish(
                    _initial_pose_message(node, fault))
                return True

            def fault_tick(clock_seconds):
                if clock_seconds is None:
                    return
                if clock_origin[0] is None:
                    clock_origin[0] = clock_seconds
                fault.tick(clock_seconds - clock_origin[0],
                           publish_reference_pose)
```

- [ ] **Step 4: Make the existing test helper return a bool**

Existing `TiledFaultRunTest` publishes through `lambda: published.append(1)`, which returns `None`; with the gate that now means "not published". Replace `_run` (lines 888-893) with:

```python
    def _run(self, **kwargs):
        fixture = self._Fixture(**kwargs.pop("fixture", {}))
        run = localization_regression.TiledFaultRun(
            fixture, self._reference(**kwargs.pop("reference", {})))
        published = []

        def publish():
            published.append(1)
            return True

        return run, fixture, published, publish
```

- [ ] **Step 5: Run the file to verify everything passes**

```bash
PYTHONPATH=test/integration python3 test/test_localization_regression.py
```

Expected: `OK`, including `test_publishes_at_the_reference_time_once_the_imu_is_ready`, `test_refuses_a_reference_time_that_was_missed`, and the two new tests.

- [ ] **Step 6: Commit**

```bash
git add test/integration/localization_regression.py test/test_localization_regression.py
git commit -m "fix(integration): publish initialpose only to a matched subscriber"
```

---

## Task 7: Re-acceptance — unit tests, serial build, 44-test gate, tiled-faults

**Files:** none modified; this task produces the evidence recorded in Task 8.

- [ ] **Step 1: Run every touched unit test file**

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai/.worktrees/p22-postmerge-fixes
PYTHONPATH=test/integration python3 test/test_localization_regression.py
PYTHONPATH=test/integration python3 test/test_localization_stability_common.py
PYTHONPATH=test/integration python3 test/test_localization_stability_baseline.py
PYTHONPATH=test/integration python3 test/test_localization_stability_rss.py
python3 test/test_tiled_map_integration.py
python3 test/test_launch_sim_clock.py
python3 test/test_localization_tf_continuity.py
```

Expected: `OK` for every file (exit code 0 each).

- [ ] **Step 2: Serial clean build from scratch**

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai/.worktrees/p22-postmerge-fixes
rm -rf /tmp/p22_hardening
source /opt/ros/humble/setup.bash
time colcon --log-base /tmp/p22_hardening/log build --symlink-install \
  --packages-select lio_sam_hesai \
  --build-base /tmp/p22_hardening/build --install-base /tmp/p22_hardening/install \
  --cmake-args -DBUILD_TESTING=ON --executor sequential --parallel-workers 1
```

Expected: exit code 0, `Summary: 1 package finished`, about 28 minutes. Record the wall time for HANDOFF.

- [ ] **Step 3: Run the full CTest suite and confirm the failures are only the known lint baseline**

```bash
ctest --test-dir /tmp/p22_hardening/build/lio_sam_hesai --output-on-failure -j1
```

Expected: `96% tests passed, 4 tests failed out of 50`; the failures are exactly `flake8`, `lint_cmake`, `uncrustify`, `xmllint`.

- [ ] **Step 4: Run the 44-test convention gate**

```bash
ctest --test-dir /tmp/p22_hardening/build/lio_sam_hesai --output-on-failure -j1 \
  -E '^(copyright|cpplint|flake8|lint_cmake|uncrustify|xmllint)$'
```

Expected: `100% tests passed, 0 tests failed out of 44`. (50 registered tests minus the 6 excluded lint jobs.)

- [ ] **Step 5: Run the tiled fault replays against the clean install**

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai/.worktrees/p22-postmerge-fixes
LIO_SAM_WORKSPACE=/tmp/p22_hardening LIO_SAM_KEEP_RESULTS=1 \
  scripts/run_localization_stability.sh tiled-faults \
  > /tmp/p22_hardening/tiled-faults.log 2>&1
echo "exit=$?"
tail -40 /tmp/p22_hardening/tiled-faults.log
```

Expected: `exit=0`. The log contains `capturing a no-fault reference pose`, one `reference tile index:` line, and a `results_dir=` line. The reference capture now records the tiled manifest digest, and the three fault runs still validate it.

- [ ] **Step 6: Inspect the per-case evidence**

```bash
results_dir=$(sed -n 's/^results_dir=//p' /tmp/p22_hardening/tiled-faults.log | tail -1)
python3 - "$results_dir" <<'PY'
import json, pathlib, sys
root = pathlib.Path(sys.argv[1])
for case in ("missing-center", "missing-neighbor"):
    metrics = json.loads((root / case / "observed.json").read_text())
    print(case, {key: metrics.get(key) for key in
                 ("first_ok_seconds", "valid_odom_ratio", "error_samples",
                  "map_tile_load_failures")})
PY
grep -R "corrupt_manifest_node_died" "$results_dir"/*/observed.json
```

Expected: `missing-center` has `first_ok_seconds: None` and `valid_odom_ratio: None`; `missing-neighbor` has a finite `first_ok_seconds`, `valid_odom_ratio: 1.0`, `error_samples: 0`, and `map_tile_load_failures >= 1`; the grep finds `"corrupt_manifest_node_died": true` in the corrupt-manifest run. Save these numbers for Task 8.

- [ ] **Step 7: No commit**

This task only produces evidence; nothing changes in the tree.

---

## Task 8: Clear the new E303 and record the re-acceptance

**Files:**
- Modify: `test/test_tiled_map_integration.py:339-340` and `:392-395`
- Modify: `docs/HANDOFF.md:250` and append §9.9

- [ ] **Step 1: Remove the extra blank lines**

In `test/test_tiled_map_integration.py`, delete one of the two blank lines before `def test_fault_fixture_refuses_paths_that_climb_out` (line 340) and two of the three blank lines before `def test_injection_refuses_a_directory_that_is_not_a_fixture` (line 395), so each method is preceded by exactly one blank line inside the class.

- [ ] **Step 2: Verify E303 is gone and the file still passes**

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai/.worktrees/p22-postmerge-fixes
python3 -m flake8 --select=E303 test/
python3 test/test_tiled_map_integration.py
```

Expected: flake8 prints nothing and exits 0; the test file prints `OK`. Confirm the rest of the lint baseline is unchanged with:

```bash
python3 -m flake8 test/test_tiled_map_integration.py | sed 's/.*: //' | sort | uniq -c
```

Expected: only `E402` (2) and `E501` (14) remain — no `E303`.

- [ ] **Step 3: Re-run the convention gate on the final tree**

```bash
ctest --test-dir /tmp/p22_hardening/build/lio_sam_hesai --output-on-failure -j1 \
  -E '^(copyright|cpplint|flake8|lint_cmake|uncrustify|xmllint)$'
```

Expected: `100% tests passed, 0 tests failed out of 44`. CTest runs the Python tests from the source tree, so this re-verifies the whitespace-only edit without a rebuild; no C++ changed after Task 7's build.

- [ ] **Step 4: Update the merged status in HANDOFF**

In `docs/HANDOFF.md` line 250, replace:

```markdown
按 `docs/P2.2_fix.md` 执行。本轮修复的是第 8 节记录的四类问题：launch 参数方向、相对地图路径、分块覆盖语义与故障回放、以及独立参数复核。**分支仍未合并到 `main`。**
```

with:

```markdown
按 `docs/P2.2_fix.md` 执行。本轮修复的是第 8 节记录的四类问题：launch 参数方向、相对地图路径、分块覆盖语义与故障回放、以及独立参数复核。**本轮已并入本地 `main`（`740dab9`）；合并后加固与重新验收见 9.9。**
```

- [ ] **Step 5: Append §9.9**

Append to the end of `docs/HANDOFF.md`, filling every `<...>` from Task 7's actual output (no `<...>` may remain):

```markdown
### 9.9 合并后加固轮（2026-09-15）

P2.2 修复轮与复核轮已并入本地 `main`（`740dab9`）；本轮从该提交拉出
`fix/p22-postmerge-hardening`，按
`docs/superpowers/specs/2026-09-15-p22-postmerge-hardening-design.md` 修复复核遗留的
六类问题后重新验收。计划见
`docs/superpowers/plans/2026-09-15-p22-postmerge-hardening.md`。

**修复内容**

1. `src/common/slam_config.cpp`：删除 `map_save.tiled_map.tile_size_m` 的重复
   `RequireNonNegative`（死代码）并修正 `system.mapping_process_interval` 的缩进；
   `test_slam_config.cpp` 新增 `MappingRejectsNonPositiveOrNonFiniteTileSize`，
   锁定"有限且 > 0"契约（与 `tile_map_generator.cpp` 运行期检查一致）。
2. `prepare_tiled_fault_case`：`center_index=None` 对任何 fault case（含
   `corrupt-manifest`）都抛 `DatasetError`，不再泄漏 `TypeError`，且目标目录在
   校验通过前不会被创建。
3. `MetricCollector` 的 tracking 闸门不再粘性：每个可解码的目标诊断重写闸门，
   WARN/ERROR 或 `is_tracking != true` 都会复位，参考位姿不会记录在失去跟踪之后。
4. 地图身份从字节数升级为具名 artifact 的 SHA-256：`DatasetIdentity.map_artifacts`
   覆盖 `GlobalMap.pcd`、`GroundMap.pcd`、`sc_db.bin`，tiled 模式另含
   `tiled/manifest.yaml`；引用绑定成为身份的一部分，不再依赖调用方传入指纹。
   `schema_version` 保持 1：旧引用缺少 `artifacts` 键，比较时 fail-closed 并报
   `recapture it`，必须重录（`tiled-faults` 每次运行都会重新捕获）。
5. 参考位姿与时间字段校验收紧：`bag_seconds`/`offset_seconds` 有限且 >= 0，
   `warmup_seconds` 有限且 > 0，且 `offset_seconds <= bag_seconds`；`map_T_base`
   必须有限、底行 `[0,0,0,1]`、旋转正交、`det = +1`（容差 1e-6），record 与 load
   两侧使用同一套校验。
6. `/initialpose` 发布前等待 matched subscriber：发布回调返回 bool，
   `TiledFaultRun.tick` 只在回调显式返回 True 时置 `published`；无订阅者时返回
   `waiting-for-subscriber` 并由后续 tick 重试，窗口内始终无订阅者则抛
   `ReplayError`，不阻塞 executor。
7. `test/test_tiled_map_integration.py` 中 `06a24272` 引入的 2 处 flake8 E303 清零。

**重新验收（干净树）**

- 单测：<列出 Task 7 Step 1 的 7 个文件，全部 OK>。
- 串行干净构建：`colcon --log-base /tmp/p22_hardening/log build --symlink-install
  --packages-select lio_sam_hesai --build-base /tmp/p22_hardening/build
  --install-base /tmp/p22_hardening/install --cmake-args -DBUILD_TESTING=ON
  --executor sequential --parallel-workers 1`，退出码 0，耗时 <实测>。
- 约定 CTest：`ctest -E '^(copyright|cpplint|flake8|lint_cmake|uncrustify|xmllint)$'`
  → 44/44 通过；完整 CTest 46/50，失败项仍为既有的 `flake8`、`lint_cmake`、
  `uncrustify`、`xmllint`。
- `tiled-faults`：`LIO_SAM_WORKSPACE=/tmp/p22_hardening
  scripts/run_localization_stability.sh tiled-faults` 退出码 0。
  `corrupt-manifest` 的 `observed.json` 为 `corrupt_manifest_node_died: true`；
  `missing-center` 为 `first_ok_seconds: null`、`valid_odom_ratio: null`；
  `missing-neighbor` 为 `first_ok_seconds: <实测>`、`valid_odom_ratio: 1.0`、
  `error_samples: 0`、`map_tile_load_failures: <实测>`。
- 地图指纹：本轮起 reference 记录具名 artifact 摘要；`tiled-faults` 无需迁移脚本，
  旧 reference 文件必须重录。
```

- [ ] **Step 6: Commit**

```bash
git add test/test_tiled_map_integration.py
git commit -m "chore(lint): drop the new tiled fixture E303"
git add docs/HANDOFF.md
git commit -m "docs(p2): record the post-merge hardening round"
```

---

## Self-review

**Spec coverage:** Task 1 ↔ item 1; Task 2 ↔ item 2; Task 3 ↔ item 3; Tasks 4+5 ↔ item 4
(identity and validation are separated so each has a red/green cycle and its own commit);
Task 6 ↔ item 5; Task 7 ↔ item 6; Task 8 ↔ item 7.

**Placeholder scan:** the only `<...>` markers are measured build/replay values in the
HANDOFF template, and Step 5 states they must be filled from Task 7's output. No design or
code step is deferred.

**Type consistency:** `DatasetIdentity.map_artifacts` is the only identity field touched in
Tasks 4-5; `map_artifacts()` (module function), `_artifact_sha256()`,
`_require_finite_number()`, `_require_rigid_matrix4()` keep the same names and signatures
across tasks; `TiledFaultRun.missing_subscriber`, `tick(clock_seconds, publish_pose)`, and
`publish_reference_pose()` match between Task 6's tests and implementation.
