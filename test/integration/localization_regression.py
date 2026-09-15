#!/usr/bin/env python3
"""Collect localization replay metrics and run a bounded ROS bag regression."""

from __future__ import annotations

import argparse
import copy
import json
import re
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from collections.abc import Mapping

from stability_baseline import (
    BASELINE,
    BaselineError,
    compare_quick_metrics,
    load_baseline,
    write_candidate,
)
from stability_common import (
    DatasetError,
    load_tiled_fault_fixture,
    load_tiled_fault_reference,
    make_tiled_fault_reference,
    save_tiled_fault_reference,
    validate_dataset,
    write_runtime_parameters,
)


_METRIC_FIELDS = (
    "first_odom_seconds",
    "valid_odom_ratio",
    "max_odom_gap_seconds",
    "first_ok_seconds",
    "error_samples",
    "max_non_ok_gap_seconds",
    "min_health_score",
    "max_reg_fitness",
    "max_covariance_scale",
    "min_lidar_rate_hz",
    "min_imu_rate_hz",
    "timestamp_monotonic",
)
_TARGET_DIAGNOSTIC = "lio_sam_hesai: localization"
_NUMERIC_DIAGNOSTIC_FIELDS = (
    "health_score",
    "reg_fitness",
    "covariance_scale",
    "lidar_rate_hz",
    "imu_rate_hz",
)
_TILED_NUMERIC_DIAGNOSTIC_FIELDS = (
    "map_manifest_version",
    "map_active_tile_count",
    "map_cached_tile_count",
    "map_cache_mib",
    "map_cache_hits",
    "map_cache_misses",
    "map_tile_load_ms",
    "map_tile_load_failures",
)
_TILED_TEXT_DIAGNOSTIC_FIELDS = (
    "map_mode",
    "map_tile_load_state",
    "map_tile_load_error",
)
_REPLAY_TIMEOUT_MIN_SECONDS = 30.0
_REPLAY_TIMEOUT_MAX_SECONDS = 900.0
_REPLAY_TIMEOUT_ALLOWANCE_SECONDS = 30.0
_FINAL_CALLBACK_DRAIN_MAX_CALLBACKS = 32
_FAULT_DURATION_SECONDS = 30.0


def _observer_subscription_qos(sensor_data_qos, reliable_qos):
    """Select the compatible QoS profile for each replay observer topic."""
    return {
        "/clock": sensor_data_qos,
        "/lio_sam/localization/odometry": reliable_qos,
        "/diagnostics": reliable_qos,
    }


class MetricCollector:
    """Pure, callback-friendly localization metric accumulator."""

    def __init__(self, capture_pose=False):
        self._capture_pose = capture_pose
        self._captured_pose = None
        self._tile_events = []
        self._last_tile_event = None
        self._tracking_confirmed = False
        self._clock_start = None
        self._odom_samples = 0
        self._valid_odom_samples = 0
        self._first_valid_odom_seconds = None
        self._last_valid_odom = None
        self._max_odom_gap = None
        self._first_ok_seconds = None
        self._error_samples = 0
        self._diagnostic_events = []
        self._diagnostic_values = {key: [] for key in _NUMERIC_DIAGNOSTIC_FIELDS}
        self._invalid_diagnostic_values = set()
        self._timestamp_monotonic = None
        self._map_diagnostics = {}
        self._max_target_diagnostic_level = None

    def clock(self, seconds):
        timestamp = _finite_number(seconds)
        if timestamp is not None and self._clock_start is None:
            self._clock_start = timestamp

    def odom(self, seconds, x, y, z, qx, qy, qz, qw):
        self._odom_samples += 1
        timestamp = _finite_number(seconds)
        values = tuple(_finite_number(value) for value in (x, y, z, qx, qy, qz, qw))
        if timestamp is None or any(value is None for value in values):
            return

        quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
        if abs(quaternion_norm - 1.0) > 1e-3:
            return

        self._valid_odom_samples += 1
        if self._first_valid_odom_seconds is None:
            self._first_valid_odom_seconds = timestamp
        if self._capture_pose and self._captured_pose is None:
            # The initialization result is published as odometry too, with
            # is_tracking false, so the first valid sample is the initialization
            # pose and not a tracking pose. Wait until a diagnostic confirms
            # tracking.
            if self._tracking_confirmed:
                self._captured_pose = (timestamp, self._pose_matrix(values))
        if self._last_valid_odom is None:
            self._max_odom_gap = 0.0
        else:
            self._max_odom_gap = max(
                self._max_odom_gap,
                max(0.0, timestamp - self._last_valid_odom),
            )
        self._last_valid_odom = timestamp

    @staticmethod
    def _pose_matrix(values):
        x, y, z, qx, qy, qz, qw = values
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
        return [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
             2 * (qx * qz + qy * qw), x],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),
             2 * (qy * qz - qx * qw), y],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
             1 - 2 * (qx * qx + qy * qy), z],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def captured_pose(self):
        """
        (bag_seconds, map_T_base) of the first valid tracking sample, or None.

        Tracking is confirmed by the most recent decodable target diagnostic,
        so an initialization sample published with is_tracking=false is never
        mistaken for a stable pose. The timestamp is the replay clock, which is
        the bag's own time base -- an absolute value near 1.8e9, not an offset
        into the bag.
        """
        return self._captured_pose

    def clock_start(self):
        return self._clock_start

    def diagnostic(self, seconds, level, name, values):
        if name != _TARGET_DIAGNOSTIC:
            return
        if isinstance(values, Mapping):
            self._record_tile_event(values)
        timestamp = _finite_number(seconds)
        numeric_level = _diagnostic_level(level)
        if numeric_level is None or timestamp is None:
            return
        self._diagnostic_events.append((timestamp, numeric_level))
        self._max_target_diagnostic_level = _max_or_initial(
            self._max_target_diagnostic_level, numeric_level
        )

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

        if not isinstance(values, Mapping):
            values = {}
        for key in _TILED_TEXT_DIAGNOSTIC_FIELDS:
            if key in values:
                self._map_diagnostics[key] = str(values[key])
        for key in _TILED_NUMERIC_DIAGNOSTIC_FIELDS:
            if key in values:
                self._map_diagnostics[key] = _finite_number(values[key])
        for key in _NUMERIC_DIAGNOSTIC_FIELDS:
            if key not in values:
                continue
            value = _finite_number(values[key])
            if value is None:
                self._invalid_diagnostic_values.add(key)
            else:
                self._diagnostic_values[key].append(value)

        if "timestamp_monotonic" in values:
            timestamp_monotonic = _boolean_value(values["timestamp_monotonic"])
            if timestamp_monotonic is None:
                self._timestamp_monotonic = None
            elif self._timestamp_monotonic is None:
                self._timestamp_monotonic = timestamp_monotonic
            else:
                self._timestamp_monotonic = (
                    self._timestamp_monotonic and timestamp_monotonic
                )

    def _record_tile_event(self, values):
        """
        Track the tile-loading sequence, not just the final error string.

        Startup diagnostics are published before /clock moves, so this runs
        ahead of the clock filter. A run that only ever shows a startup state
        and a final summary would otherwise look like an injected fault.
        """
        snapshot = (
            values.get("map_tile_load_state"),
            str(values.get("map_tile_load_failures")),
            str(values.get("map_tile_load_error", "")),
        )
        if snapshot != self._last_tile_event:
            self._last_tile_event = snapshot
            self._tile_events.append(snapshot)

    def finish(self, seconds):
        finish_time = _finite_number(seconds)
        max_non_ok_gap = _max_non_ok_gap_seconds(
            self._diagnostic_events, self._clock_start, finish_time
        )

        values = self._diagnostic_values
        invalid = self._invalid_diagnostic_values
        metrics = {
            "first_odom_seconds": _offset_from_clock(
                self._first_valid_odom_seconds, self._clock_start
            ),
            "valid_odom_ratio": (
                self._valid_odom_samples / self._odom_samples
                if self._odom_samples
                else None
            ),
            "max_odom_gap_seconds": self._max_odom_gap,
            "first_ok_seconds": _offset_from_clock(
                self._first_ok_seconds, self._clock_start
            ),
            "error_samples": self._error_samples,
            "max_non_ok_gap_seconds": max_non_ok_gap,
            "min_health_score": _minimum(values["health_score"], "health_score" in invalid),
            "max_reg_fitness": _maximum(values["reg_fitness"], "reg_fitness" in invalid),
            "max_covariance_scale": _maximum(
                values["covariance_scale"], "covariance_scale" in invalid
            ),
            "min_lidar_rate_hz": _minimum(
                values["lidar_rate_hz"], "lidar_rate_hz" in invalid
            ),
            "min_imu_rate_hz": _minimum(values["imu_rate_hz"], "imu_rate_hz" in invalid),
            "timestamp_monotonic": self._timestamp_monotonic,
        }
        metrics.update(self._map_diagnostics)
        # The tile-loading sequence, kept alongside the final summary so a
        # fault is judged on what happened during the run.
        metrics["map_tile_events"] = [list(entry) for entry in self._tile_events]
        if self._map_diagnostics:
            metrics["max_diagnostic_level"] = self._max_target_diagnostic_level
        return metrics


def tiled_metric_violations(metrics, ok_deadline_seconds):
    """Return bounded-map invariant failures in addition to legacy thresholds."""
    violations = []
    if metrics.get("map_mode") != "tiled":
        violations.append("map_mode must be tiled")
    for key, limit in (("map_cached_tile_count", 25), ("map_cache_mib", 1024)):
        value = _finite_number(metrics.get(key))
        if value is None or value > limit:
            violations.append(f"{key} must be <= {limit}")
    failures = _finite_number(metrics.get("map_tile_load_failures"))
    if failures is None or failures != 0:
        violations.append("map_tile_load_failures must be 0")
    first_ok = _finite_number(metrics.get("first_ok_seconds"))
    deadline = _finite_number(ok_deadline_seconds)
    if first_ok is None or deadline is None or first_ok > deadline:
        violations.append("automatic relocalization did not reach OK by deadline")
    return violations


def tiled_fault_violations(metrics, fault_case, expected_missing_index=None):
    """
    Judge a tiled fault replay from its event sequence, not one final string.

    A run that never actually requested the faulty tile -- wrong seeded pose,
    injection that never took effect -- must not pass because some unrelated
    diagnostic happened to mention a tile index.
    """
    if fault_case == "corrupt-manifest":
        # The manifest is already broken, so the only question is whether the
        # node refused to come up. A launch process that keeps running says
        # nothing, which is why the node's own death is reported separately.
        violations = []
        if metrics.get("corrupt_manifest_node_died") is not True:
            violations.append(
                "corrupt-manifest must make the localization node exit "
                "non-zero, not merely kill it during cleanup")
        if metrics.get("corrupt_manifest_map_preparation_failed") is not True:
            violations.append(
                "corrupt-manifest must fail map preparation")
        if _finite_number(metrics.get("first_ok_seconds")) is not None:
            violations.append(
                "corrupt-manifest must not produce localization output")
        return violations
    if fault_case not in ("missing-center", "missing-neighbor"):
        return [f"unknown tiled fault case: {fault_case}"]

    violations = []
    events = [list(entry) for entry in (metrics.get("map_tile_events") or [])]
    if not events:
        violations.append(f"{fault_case} observed no tile diagnostics at all")
    if not any(str(entry[1]) not in ("0", "0.0") for entry in events):
        violations.append(
            f"{fault_case} never recorded a tile load failure, so the injected "
            "payload was never missed")
    if expected_missing_index and not any(
            expected_missing_index in str(entry[2]) for entry in events):
        violations.append(
            f"{fault_case} never named {expected_missing_index} while the map "
            "was being requested")

    first_ok = _finite_number(metrics.get("first_ok_seconds"))
    valid_ratio = _finite_number(metrics.get("valid_odom_ratio"))
    if fault_case == "missing-center":
        # The centre tile is gone, so nothing can be registered against it.
        if first_ok is not None or (valid_ratio is not None and valid_ratio > 0.0):
            violations.append(
                "missing-center must not produce localization output")
    else:
        # The neighbour is gone but the centre survives, so the run must still
        # localize and degrade. A startup WARN, or an initialization that never
        # completes, is not a pass.
        if first_ok is None:
            violations.append("missing-neighbor must still localize")
        if valid_ratio is None or valid_ratio <= 0.0:
            violations.append(
                "missing-neighbor must still publish valid odometry")
        # The centre must have been usable at the same time: a degraded
        # snapshot is a *ready* state that names the missing neighbour. A bare
        # load failure only says the payload was missed.
        degraded_snapshot = any(
            str(entry[0]) == "ready" and "missing tiles:" in str(entry[2]) and
            expected_missing_index and expected_missing_index in str(entry[2])
            for entry in events)
        if not degraded_snapshot:
            violations.append(
                "missing-neighbor must keep the centre usable and report a "
                f"degraded snapshot naming {expected_missing_index}")
    return violations


def _finite_number(value):
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def _diagnostic_level(value):
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, (bytes, bytearray)) and len(value) == 1:
        return value[0]
    return None


def _boolean_value(value):
    if value is True or (isinstance(value, str) and value.strip().lower() == "true"):
        return True
    if value is False or (isinstance(value, str) and value.strip().lower() == "false"):
        return False
    return None


def _max_or_initial(previous, value):
    return value if previous is None else max(previous, value)


def _max_non_ok_gap_seconds(events, clock_start, finish_time):
    if clock_start is None or not events:
        return None
    non_ok_start = clock_start
    maximum_gap = None
    for timestamp, level in sorted(events):
        if timestamp < clock_start:
            continue
        if level == 0:
            if non_ok_start is not None:
                maximum_gap = _max_or_initial(
                    maximum_gap, max(0.0, timestamp - non_ok_start)
                )
                non_ok_start = None
        elif non_ok_start is None:
            non_ok_start = timestamp
    if non_ok_start is not None and finish_time is not None:
        maximum_gap = _max_or_initial(
            maximum_gap, max(0.0, finish_time - non_ok_start)
        )
    return maximum_gap


def _offset_from_clock(timestamp, clock_start):
    if timestamp is None or clock_start is None:
        return None
    return timestamp - clock_start


def _minimum(values, invalid):
    if invalid:
        return math.nan
    return min(values) if values else None


def _maximum(values, invalid):
    if invalid:
        return math.nan
    return max(values) if values else None


class ReplayError(RuntimeError):
    pass


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="Replay a localization bag and compare its quick metrics."
    )
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--map", required=True, type=Path)
    parser.add_argument("--params-template", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--tiled-dir", type=Path)
    parser.add_argument(
        "--tiled-fault-case",
        choices=("corrupt-manifest", "missing-center", "missing-neighbor"),
    )
    parser.add_argument("--expected-missing-index")
    parser.add_argument(
        "--record-reference", type=Path,
        help="Record the first stable tracking pose for a later fault replay.")
    parser.add_argument(
        "--tiled-fault-reference", type=Path,
        help="Reference pose file captured from a no-fault replay.")

    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--baseline", type=Path)
    mode.add_argument("--record-baseline", action="store_true")
    return parser


def main(argv=None):
    parser = build_argument_parser()
    arguments = parser.parse_args(argv)
    output_dir = arguments.output_dir
    launch_log = None
    player_log = None
    identity = None
    metrics = None
    error_message = None
    exit_code = 0

    try:
        output_dir.mkdir(parents=True, exist_ok=True)
        launch_log = (output_dir / "launch.log").open("w", encoding="utf-8")
        player_log = (output_dir / "bag-play.log").open("w", encoding="utf-8")

        if (not arguments.record_baseline and arguments.baseline is None and
                arguments.tiled_fault_case is None):
            raise ReplayError("--baseline is required unless --record-baseline is used")
        if arguments.tiled_fault_case and arguments.record_baseline:
            raise ReplayError(
                "a tiled fault case must not record a baseline; the fault is "
                "deliberate and the thresholds are not measurements")

        identity = validate_dataset(
            arguments.bag, arguments.map, tiled_dir=arguments.tiled_dir
        )
        runtime_parameters = write_runtime_parameters(
            arguments.params_template,
            arguments.map,
            output_dir,
            tiled_dir=arguments.tiled_dir,
            # A fault replay seeds the pose itself; auto relocalization would
            # pick a different tile and prove nothing.
            auto_relocalization=(False if arguments.tiled_fault_case else None),
        )
        baseline = (copy.deepcopy(BASELINE) if arguments.record_baseline else
                    load_baseline(arguments.baseline) if arguments.baseline else
                    copy.deepcopy(BASELINE))
        if arguments.tiled_fault_case:
            baseline["quick"]["duration_seconds"] = _FAULT_DURATION_SECONDS
        fault = None
        if arguments.tiled_fault_case == "corrupt-manifest":
            # The manifest is already broken, so there is nothing to inject and
            # nothing to replay: the only question is whether the node refuses
            # to come up.
            metrics = _run_launch_only(
                arguments, runtime_parameters, baseline["quick"], launch_log)
        else:
            if arguments.tiled_fault_case is not None:
                if arguments.tiled_fault_reference is None:
                    raise ReplayError(
                        "--tiled-fault-case requires --tiled-fault-reference")
                # The fixture is loaded, never built from --tiled-dir: a
                # mistyped directory must not be able to move real map files.
                fault = TiledFaultRun(
                    load_tiled_fault_fixture(arguments.tiled_dir),
                    load_tiled_fault_reference(
                        arguments.tiled_fault_reference, identity),
                )
            metrics = _run_replay(
                arguments,
                runtime_parameters,
                baseline["quick"],
                launch_log,
                player_log,
                fault=fault,
                identity=identity,
            )

        if arguments.record_baseline:
            candidate_path = output_dir / "candidate-baseline.yaml"
            write_candidate(candidate_path, identity, metrics)
            print(f"Candidate baseline written to {candidate_path}")
        elif arguments.tiled_fault_case:
            violations = tiled_fault_violations(
                metrics,
                arguments.tiled_fault_case,
                arguments.expected_missing_index,
            )
            if violations:
                exit_code = 1
                for violation in violations:
                    print(f"fault violation: {violation}", file=sys.stderr)
        else:
            violations = compare_quick_metrics(baseline, metrics)
            if arguments.tiled_dir is not None:
                violations.extend(
                    tiled_metric_violations(
                        metrics, baseline["quick"]["ok_deadline_seconds"]
                    )
                )
            if violations:
                exit_code = 1
                for violation in violations:
                    print(f"metric violation: {violation}", file=sys.stderr)
    except KeyboardInterrupt:
        exit_code = 130
        error_message = "interrupted"
        print("localization replay interrupted", file=sys.stderr)
    except (BaselineError, DatasetError, ImportError, OSError, ReplayError) as error:
        exit_code = 2
        error_message = str(error)
        print(f"localization replay error: {error}", file=sys.stderr)
    finally:
        try:
            if output_dir.exists():
                _write_observed(output_dir / "observed.json", identity, metrics, error_message)
        except OSError as error:
            print(f"localization replay error: unable to write observed.json: {error}", file=sys.stderr)
            exit_code = 2
        if launch_log is not None:
            launch_log.close()
        if player_log is not None:
            player_log.close()
    return exit_code


# How late a reference pose may still be published. Past this the vehicle has
# moved far enough that the seeded pose no longer describes where it is, and
# the replay would be asserting against the wrong tile.
_FAULT_POSE_TOLERANCE_SECONDS = 1.0

# Seconds of replay kept after the reference pose, to observe what the tile
# request actually does.
_FAULT_OBSERVATION_SECONDS = 15.0


class TiledFaultRun:
    """
    Drive one tiled fault replay: inject the payload, then seed the pose.

    The order matters and is the whole point of the exercise:

    * the fault is injected only once the node reports a prepared map with no
      load failures, so it lands as a *runtime* load failure and not as a
      manifest validation failure;
    * the reference pose is published only once the IMU is initialized, so the
      node accepts it;
    * the pose is published only once a matched `/initialpose` subscriber
      exists, because a publish with no subscription is dropped silently;
    * both gates are read from diagnostics as they arrive, not through the
      replay clock, because startup diagnostics are published before /clock
      starts moving.
    """

    def __init__(self, fixture, reference,
                 tolerance_seconds=_FAULT_POSE_TOLERANCE_SECONDS):
        self.fixture = fixture
        self.reference = reference
        self.tolerance_seconds = tolerance_seconds
        self.injected = False
        self.published = False
        self.published_at = None
        self.missing_subscriber = False
        self.values = {}

    def observe_diagnostics(self, message):
        # Only the localization node's status: another node reporting a
        # same-named key would open the prepare or IMU gate by accident.
        for status in message.status:
            if status.name != _TARGET_DIAGNOSTIC:
                continue
            self.values.update({kv.key: kv.value for kv in status.values})

    @property
    def reference_offset(self):
        """
        Seconds after the replay clock starts when the reference pose is due.

        The stored bag time is relative to the bag start and the player is
        started at `offset_seconds`, so what the tick sees (`clock - clock_start`)
        lines up with `bag_seconds - offset_seconds`.
        """
        return max(0.0, self.reference.bag_seconds -
                   self.reference.offset_seconds)

    def prepared(self):
        return (self.values.get("map_tile_load_state") == "ready" and
                str(self.values.get("map_tile_load_failures", "1")) in ("0", "0.0"))

    def imu_initialized(self):
        return str(self.values.get("has_imu_init", "")).lower() == "true"

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


def _run_replay(arguments, runtime_parameters, quick, launch_log, player_log,
                fault=None, identity=None):
    """Run the ROS observer and bounded playback after pure CLI validation."""
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock

    domain_id = str(200 + (os.getpid() % 30))
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = domain_id
    os.environ["ROS_DOMAIN_ID"] = domain_id

    launch_command = [
        "ros2",
        "launch",
        "lio_sam_hesai",
        "localization_with_nonground.launch.py",
        "use_sim:=true",
        "use_rviz:=false",
        "publish_static_tf:=true",
        f"params_file:={runtime_parameters}",
    ]
    player_command = [
        "ros2",
        "bag",
        "play",
        str(arguments.bag),
        "--clock",
        "100",
        "--rate",
        str(quick["rate"]),
        "--start-offset",
        str(quick["start_offset_seconds"]),
        "--disable-keyboard-controls",
    ]
    replay_duration = float(quick["duration_seconds"])
    if fault is not None:
        # Start early enough to warm the IMU up, then keep playing past the
        # reference pose to watch the tile request fail.
        player_command[player_command.index("--start-offset") + 1] = str(
            fault.reference.offset_seconds)
        replay_duration = ((fault.reference.bag_seconds -
                            fault.reference.offset_seconds) +
                           _FAULT_OBSERVATION_SECONDS)

    launch_process = None
    player_process = None
    node = None
    rclpy_initialized = False
    latest_clock = [None]
    try:
        rclpy.init(args=None)
        rclpy_initialized = True
        node = rclpy.create_node(
            "localization_regression_observer",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        collector = MetricCollector(
            capture_pose=arguments.record_reference is not None)

        def clock_callback(message):
            seconds = _ros_time_seconds(message.clock)
            latest_clock[0] = seconds
            collector.clock(seconds)

        def odometry_callback(message):
            _record_odom_at_replay_clock(collector, latest_clock[0], message)

        initial_pose_publisher = None
        if fault is not None:
            initial_pose_publisher = node.create_publisher(
                PoseWithCovarianceStamped, "/initialpose",
                QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE),
            )

        def diagnostics_callback(message):
            _record_diagnostics_at_replay_clock(collector, latest_clock[0], message)
            if fault is not None:
                # Raw, not clock-filtered: the prepare/IMU gates are published
                # before /clock starts moving.
                fault.observe_diagnostics(message)

        subscription_qos = _observer_subscription_qos(
            qos_profile_sensor_data,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE),
        )
        node.create_subscription(
            Clock, "/clock", clock_callback, subscription_qos["/clock"]
        )
        node.create_subscription(
            Odometry,
            "/lio_sam/localization/odometry",
            odometry_callback,
            subscription_qos["/lio_sam/localization/odometry"],
        )
        node.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            diagnostics_callback,
            subscription_qos["/diagnostics"],
        )

        launch_process = subprocess.Popen(
            launch_command,
            stdout=launch_log,
            stderr=subprocess.STDOUT,
            env=environment,
            start_new_session=True,
        )
        _wait_for_startup(
            node, launch_process, float(quick["startup_grace_seconds"])
        )

        player_process = subprocess.Popen(
            player_command,
            stdout=player_log,
            stderr=subprocess.STDOUT,
            env=environment,
            start_new_session=True,
        )
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

        _spin_bag_window(
            node,
            launch_process,
            player_process,
            latest_clock,
            replay_duration,
            float(quick["rate"]),
            tick=fault_tick,
        )
        if arguments.record_reference is not None:
            captured = collector.captured_pose()
            if captured is None:
                raise ReplayError(
                    "no stable tracking pose was observed, so no reference "
                    "could be recorded")
            # The recorded time has to be relative to the BAG start. The
            # capture run may itself have started part-way in, so the elapsed
            # clock alone is short by that start offset.
            elapsed = _offset_from_clock(captured[0], collector.clock_start())
            if elapsed is None:
                raise ReplayError(
                    "the reference pose arrived before the replay clock started, "
                    "so it cannot be located in the bag")
            bag_seconds = (float(quick["start_offset_seconds"]) + elapsed)
            save_tiled_fault_reference(
                arguments.record_reference,
                make_tiled_fault_reference(
                    identity, bag_seconds=bag_seconds,
                    map_T_base=captured[1]))
        if fault is not None:
            if not fault.injected:
                raise ReplayError(
                    "the fault was never injected: the node never reported a "
                    "prepared map with zero load failures")
            if not fault.published:
                raise ReplayError(
                    "the reference pose was never published before the replay ended")
        return collector.finish(latest_clock[0])
    finally:
        cleanup_error = None
        for process in (player_process, launch_process):
            try:
                _stop_process_group(process)
            except ReplayError as error:
                if cleanup_error is None:
                    cleanup_error = error
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if rclpy_initialized:
                rclpy.shutdown()
        if cleanup_error is not None and sys.exc_info()[0] is None:
            raise cleanup_error


def _initial_pose_message(node, fault):
    """Return the reference pose as /initialpose, in the map frame."""
    from geometry_msgs.msg import PoseWithCovarianceStamped

    matrix = fault.reference.map_T_base
    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.pose.position.x = matrix[0][3]
    message.pose.pose.position.y = matrix[1][3]
    message.pose.pose.position.z = matrix[2][3]
    quaternion = _quaternion_from_rotation(
        [[matrix[row][column] for column in range(3)] for row in range(3)])
    message.pose.pose.orientation.x = quaternion[0]
    message.pose.pose.orientation.y = quaternion[1]
    message.pose.pose.orientation.z = quaternion[2]
    message.pose.pose.orientation.w = quaternion[3]
    return message


def _quaternion_from_rotation(rotation):
    """Convert a 3x3 rotation matrix to (x, y, z, w); Shepperd's method."""
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return ((rotation[2][1] - rotation[1][2]) / scale,
                (rotation[0][2] - rotation[2][0]) / scale,
                (rotation[1][0] - rotation[0][1]) / scale,
                0.25 * scale)
    if rotation[0][0] > rotation[1][1] and rotation[0][0] > rotation[2][2]:
        scale = math.sqrt(1.0 + rotation[0][0] - rotation[1][1] -
                          rotation[2][2]) * 2.0
        return (0.25 * scale,
                (rotation[0][1] + rotation[1][0]) / scale,
                (rotation[0][2] + rotation[2][0]) / scale,
                (rotation[2][1] - rotation[1][2]) / scale)
    if rotation[1][1] > rotation[2][2]:
        scale = math.sqrt(1.0 + rotation[1][1] - rotation[0][0] -
                          rotation[2][2]) * 2.0
        return ((rotation[0][1] + rotation[1][0]) / scale,
                0.25 * scale,
                (rotation[1][2] + rotation[2][1]) / scale,
                (rotation[0][2] - rotation[2][0]) / scale)
    scale = math.sqrt(1.0 + rotation[2][2] - rotation[0][0] -
                      rotation[1][1]) * 2.0
    return ((rotation[0][2] + rotation[2][0]) / scale,
            (rotation[1][2] + rotation[2][1]) / scale,
            0.25 * scale,
            (rotation[1][0] - rotation[0][1]) / scale)


_RELOCALIZATION_DEATH = re.compile(
    r"\[lio_sam_hesai_relocalization-\d+\]: process has died \[pid \d+, "
    r"exit code (-?\d+)")
_MAP_PREPARATION_FAILURE = "Failed to prepare localization map"


def _run_launch_only(arguments, runtime_parameters, quick, launch_log):
    """
    Launch without replaying, and report whether the node refused to start.

    A child dying does not necessarily take the launch process with it, so the
    launch log is inspected for the node's death rather than trusting the
    launch exit code.
    """
    import rclpy
    from rclpy.parameter import Parameter

    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(200 + (os.getpid() % 30))

    launch_command = [
        "ros2", "launch", "lio_sam_hesai",
        "localization_with_nonground.launch.py",
        "use_sim:=true", "use_rviz:=false", "publish_static_tf:=true",
        f"params_file:={runtime_parameters}",
    ]
    rclpy.init(args=None)
    launch_process = None
    try:
        node = rclpy.create_node(
            "localization_regression_observer",
            parameter_overrides=[Parameter("use_sim_time", value=True)])
        try:
            launch_process = subprocess.Popen(
                launch_command, stdout=launch_log, stderr=subprocess.STDOUT,
                env=environment, start_new_session=True)
            time.sleep(float(quick["startup_grace_seconds"]))
        finally:
            node.destroy_node()
    finally:
        if launch_process is not None:
            _stop_process_group(launch_process)
        rclpy.shutdown()

    launch_log.flush()
    text = Path(launch_log.name).read_text(encoding="utf-8", errors="replace")
    # "process has died" alone is not evidence: the normal cleanup also kills
    # the static TF publishers and the ground segmentation node. Match the
    # localization node specifically, with a non-zero exit code, and require
    # the reason it refused.
    died_non_zero = False
    death = _RELOCALIZATION_DEATH.search(text)
    if death is not None:
        died_non_zero = int(death.group(1)) != 0
    return {
        "corrupt_manifest_node_died": died_non_zero,
        "corrupt_manifest_map_preparation_failed":
            _MAP_PREPARATION_FAILURE in text,
    }


def _wait_for_startup(node, launch_process, grace_seconds):
    deadline = time.monotonic() + grace_seconds
    while time.monotonic() < deadline:
        if launch_process.poll() is not None:
            raise ReplayError(
                f"launch process exited during startup with code {launch_process.returncode}"
            )
        rclpy_spin_once(node, min(0.1, max(0.0, deadline - time.monotonic())))


def _spin_bag_window(
    node,
    launch_process,
    player_process,
    latest_clock,
    duration_seconds,
    rate,
    monotonic=time.monotonic,
    spin_once=None,
    tick=None,
):
    if spin_once is None:
        spin_once = rclpy_spin_once
    timeout_seconds = _replay_timeout_seconds(duration_seconds, rate)
    deadline = monotonic() + timeout_seconds
    first_clock = None

    def simulated_duration_reached():
        nonlocal first_clock
        current_clock = latest_clock[0]
        if current_clock is None:
            return False
        if first_clock is None:
            first_clock = current_clock
        return current_clock - first_clock >= duration_seconds

    def drain_final_callbacks():
        """
        Dispatch a bounded backlog from all replay metric subscriptions.

        The 32-callback cap covers the combined clock, odometry, and
        diagnostics subscription backlog with headroom without extending the
        replay or waiting for new work.
        """
        for _ in range(_FINAL_CALLBACK_DRAIN_MAX_CALLBACKS):
            spin_once(node, 0.0)

    while True:
        if launch_process.poll() is not None:
            raise ReplayError(
                f"launch process exited during playback with code {launch_process.returncode}"
            )
        player_exit_code = player_process.poll()
        if player_exit_code is not None:
            if player_exit_code != 0:
                raise ReplayError(
                    f"bag player exited during playback with code {player_exit_code}"
                )
            drain_final_callbacks()
            if simulated_duration_reached():
                return
            raise ReplayError(
                "bag player exited before replay duration after draining queued /clock"
            )

        if monotonic() >= deadline:
            waiting_for = "/clock" if first_clock is None else "/clock progress"
            raise ReplayError(
                f"replay timed out waiting for {waiting_for} after "
                f"{timeout_seconds:.1f} wall-clock seconds"
            )

        spin_once(node, min(0.1, max(0.0, deadline - monotonic())))
        if tick is not None:
            tick(latest_clock[0])
        if simulated_duration_reached():
            drain_final_callbacks()
            return


def _replay_timeout_seconds(duration_seconds, rate):
    """Bound replay collection to expected wall time plus 30s, capped at 15 minutes."""
    duration = _finite_number(duration_seconds)
    playback_rate = _finite_number(rate)
    if duration is None or duration <= 0.0:
        raise ReplayError("quick.duration_seconds must be a positive finite number")
    if playback_rate is None or playback_rate <= 0.0:
        raise ReplayError("quick.rate must be a positive finite number")
    expected_wall_seconds = duration / playback_rate
    return min(
        _REPLAY_TIMEOUT_MAX_SECONDS,
        max(
            _REPLAY_TIMEOUT_MIN_SECONDS,
            expected_wall_seconds + _REPLAY_TIMEOUT_ALLOWANCE_SECONDS,
        ),
    )


def rclpy_spin_once(node, timeout_seconds):
    import rclpy

    rclpy.spin_once(node, timeout_sec=timeout_seconds)


def _stop_process_group(process, timeout_seconds=10.0):
    if process is None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=timeout_seconds)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=timeout_seconds)
    except subprocess.TimeoutExpired:
        pass
    else:
        return
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=timeout_seconds)
    except subprocess.TimeoutExpired:
        raise ReplayError(f"process group {process.pid} could not be reaped after SIGKILL")


def _ros_time_seconds(time_message):
    seconds = _finite_number(getattr(time_message, "sec", None))
    nanoseconds = _finite_number(getattr(time_message, "nanosec", None))
    if seconds is None or nanoseconds is None:
        return None
    return seconds + nanoseconds * 1e-9


def _replay_observation_seconds(latest_clock_seconds):
    """Return replay-clock time, dropping observations received before the first /clock."""
    return _finite_number(latest_clock_seconds)


def _record_odom_at_replay_clock(collector, latest_clock_seconds, message):
    seconds = _replay_observation_seconds(latest_clock_seconds)
    if seconds is None:
        return False
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    collector.odom(
        seconds,
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    return True


def _record_diagnostics_at_replay_clock(collector, latest_clock_seconds, message):
    seconds = _replay_observation_seconds(latest_clock_seconds)
    if seconds is None:
        return False
    for status in message.status:
        collector.diagnostic(
            seconds,
            status.level,
            status.name,
            {value.key: value.value for value in status.values},
        )
    return True


def _write_observed(path, identity, metrics, error_message):
    observed = {
        "dataset": _identity_summary(identity),
        "metrics": _json_safe(metrics),
        "error": error_message,
    }
    destination = Path(path)
    temporary = destination.with_name(f".{destination.name}.{os.getpid()}.tmp")
    with temporary.open("w", encoding="utf-8") as output:
        json.dump(observed, output, allow_nan=False, indent=2, sort_keys=True)
        output.write("\n")
    os.replace(temporary, destination)


def _identity_summary(identity):
    if identity is None:
        return None
    fields = (
        "duration_ns",
        "message_count",
        "topic_counts",
        "db3_bytes",
        "map_points",
        "map_artifacts",
    )
    summary = {}
    for field in fields:
        value = identity.get(field) if isinstance(identity, Mapping) else getattr(identity, field, None)
        if value is not None:
            summary[field] = value
    return _json_safe(summary)


def _json_safe(value):
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


if __name__ == "__main__":
    sys.exit(main())
