#!/usr/bin/env python3
"""Collect localization replay metrics and run a bounded ROS bag regression."""

from __future__ import annotations

import argparse
import copy
import json
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
from stability_common import DatasetError, validate_dataset, write_runtime_parameters


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
_REPLAY_TIMEOUT_MIN_SECONDS = 30.0
_REPLAY_TIMEOUT_MAX_SECONDS = 900.0
_REPLAY_TIMEOUT_ALLOWANCE_SECONDS = 30.0
_FINAL_CALLBACK_DRAIN_MAX_CALLBACKS = 32


def _observer_subscription_qos(sensor_data_qos, reliable_qos):
    """Select the compatible QoS profile for each replay observer topic."""
    return {
        "/clock": sensor_data_qos,
        "/lio_sam/localization/odometry": reliable_qos,
        "/diagnostics": reliable_qos,
    }


class MetricCollector:
    """Pure, callback-friendly localization metric accumulator."""

    def __init__(self):
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
        if self._last_valid_odom is None:
            self._max_odom_gap = 0.0
        else:
            self._max_odom_gap = max(
                self._max_odom_gap,
                max(0.0, timestamp - self._last_valid_odom),
            )
        self._last_valid_odom = timestamp

    def diagnostic(self, seconds, level, name, values):
        if name != _TARGET_DIAGNOSTIC:
            return
        timestamp = _finite_number(seconds)
        numeric_level = _diagnostic_level(level)
        if numeric_level is None or timestamp is None:
            return
        self._diagnostic_events.append((timestamp, numeric_level))

        if numeric_level == 0:
            if self._first_ok_seconds is None:
                self._first_ok_seconds = timestamp
        else:
            if numeric_level == 2:
                self._error_samples += 1

        if not isinstance(values, Mapping):
            values = {}
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

    def finish(self, seconds):
        finish_time = _finite_number(seconds)
        max_non_ok_gap = _max_non_ok_gap_seconds(
            self._diagnostic_events, self._clock_start, finish_time
        )

        values = self._diagnostic_values
        invalid = self._invalid_diagnostic_values
        return {
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

        if not arguments.record_baseline and arguments.baseline is None:
            raise ReplayError("--baseline is required unless --record-baseline is used")

        identity = validate_dataset(arguments.bag, arguments.map)
        runtime_parameters = write_runtime_parameters(
            arguments.params_template, arguments.map, output_dir
        )
        baseline = copy.deepcopy(BASELINE) if arguments.record_baseline else load_baseline(arguments.baseline)
        metrics = _run_replay(
            arguments,
            runtime_parameters,
            baseline["quick"],
            launch_log,
            player_log,
        )

        if arguments.record_baseline:
            candidate_path = output_dir / "candidate-baseline.yaml"
            write_candidate(candidate_path, identity, metrics)
            print(f"Candidate baseline written to {candidate_path}")
        else:
            violations = compare_quick_metrics(baseline, metrics)
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


def _run_replay(arguments, runtime_parameters, quick, launch_log, player_log):
    """Run the ROS observer and bounded playback after pure CLI validation."""
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter
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
        collector = MetricCollector()

        def clock_callback(message):
            seconds = _ros_time_seconds(message.clock)
            latest_clock[0] = seconds
            collector.clock(seconds)

        def odometry_callback(message):
            _record_odom_at_replay_clock(collector, latest_clock[0], message)

        def diagnostics_callback(message):
            _record_diagnostics_at_replay_clock(collector, latest_clock[0], message)

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
        _spin_bag_window(
            node,
            launch_process,
            player_process,
            latest_clock,
            float(quick["duration_seconds"]),
            float(quick["rate"]),
        )
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
        """Dispatch a bounded backlog from all replay metric subscriptions.

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
        "map_bytes",
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
