#!/usr/bin/env python3
"""
Fault-injected rosbag acceptance harness for localization TF continuity.

The collector and fault schedule deliberately have no ROS imports so their
acceptance contract can run in ordinary Python CI.  ROS is imported only by
the bounded replay runner.
"""

from __future__ import annotations

import argparse
from collections.abc import Mapping
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import time

from stability_common import DatasetError, validate_dataset, write_runtime_parameters


_TARGET_DIAGNOSTIC = "lio_sam_hesai: localization"
_RAW_LIDAR_TOPIC = "/lidar_points"
_RAW_IMU_TOPIC = "/imu"
_RELAY_LIDAR_TOPIC = "/tf_continuity_test/lidar_points"
_RELAY_IMU_TOPIC = "/tf_continuity_test/imu"
_LOCALIZATION_ODOM_TOPIC = "/lio_sam/localization/odometry"
_LOCALIZATION_PATH_TOPIC = "/lio_sam/localization/path"
_MAX_ACTIVE_TF_SPACING_SECONDS = 0.10
_EXTRAPOLATION_DEADLINE_SECONDS = 0.5
# Forward-driving window that precedes the first injected outage.  The spatial
# contract below only applies here, and only when this window is itself
# measured to be forward-driving, so genuine reverse travel is never rejected.
_FORWARD_WINDOW_START_SECONDS = 10.0
_FORWARD_WINDOW_MIN_MEASURED_EDGES = 3
# A measured TF and its localization odometry are published from the same
# callback, so a result can only claim a TF that arrives within this window
# before it.  The window absorbs replay clock granularity and delivery jitter;
# callback timing alone cannot prove the publication source in every ordering.
_MEASURED_TF_ODOM_MATCH_SECONDS = 0.02
_SPATIAL_BACKWARD_MEDIAN_LIMIT_METERS = -0.01
_SPATIAL_BACKWARD_EDGE_LIMIT_METERS = 0.05
_STARTUP_TIMEOUT_SECONDS = 10.0
_PROCESS_STOP_TIMEOUT_SECONDS = 10.0
_PLAYBACK_TIMEOUT_ALLOWANCE_SECONDS = 45.0
_FINAL_CALLBACK_DRAIN_COUNT = 128
_MIN_SAFE_ROS_DOMAIN_ID = 1
_MAX_SAFE_ROS_DOMAIN_ID = 100


def _finite_seconds(value):
    if isinstance(value, bool):
        return None
    try:
        seconds = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return seconds if math.isfinite(seconds) else None


def _sensor_timestamp_microseconds(timestamp_seconds):
    timestamp = _finite_seconds(timestamp_seconds)
    if timestamp is None:
        return None
    return int(round(timestamp * 1.0e6))


def select_ros_domain_id(environment=None, pid=None):
    """Return an explicit safe ROS domain or a deterministic safe fallback."""
    environment = os.environ if environment is None else environment
    raw_domain_id = environment.get("ROS_DOMAIN_ID")
    if isinstance(raw_domain_id, str) and raw_domain_id.isdecimal():
        explicit_domain_id = int(raw_domain_id)
        if _MIN_SAFE_ROS_DOMAIN_ID <= explicit_domain_id <= _MAX_SAFE_ROS_DOMAIN_ID:
            return str(explicit_domain_id)

    process_id = os.getpid() if pid is None else int(pid)
    domain_count = _MAX_SAFE_ROS_DOMAIN_ID - _MIN_SAFE_ROS_DOMAIN_ID + 1
    return str(_MIN_SAFE_ROS_DOMAIN_ID + (process_id % domain_count))


@dataclass(frozen=True)
class RelayQos:
    depth: int
    reliability: str = "reliable"


def relay_qos_config():
    """Return relay QoS profiles that satisfy the localization consumers."""
    return RelayQos(depth=5), RelayQos(depth=2000)


def parse_diagnostic_level(value):
    """Return a JSON-safe diagnostic level or None for an unsupported value."""
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, (bytes, bytearray)) and len(value) == 1:
        return value[0]
    return None


@dataclass(frozen=True)
class FaultSchedule:
    """Replay-relative outage windows; the end of every window is exclusive."""

    lidar_outage: tuple[float, float] = (30.0, 40.0)
    all_outage: tuple[float, float] = (50.0, 60.0)

    def __post_init__(self):
        lidar_outage = self._validated_window("lidar_outage", self.lidar_outage)
        all_outage = self._validated_window("all_outage", self.all_outage)
        if max(lidar_outage[0], all_outage[0]) < min(lidar_outage[1], all_outage[1]):
            raise ValueError("lidar_outage and all_outage must not overlap")
        object.__setattr__(self, "lidar_outage", lidar_outage)
        object.__setattr__(self, "all_outage", all_outage)

    @staticmethod
    def _validated_window(name, window):
        if not isinstance(window, tuple) or len(window) != 2:
            raise ValueError(f"{name} must be a (start, end) tuple")
        start = _finite_seconds(window[0])
        end = _finite_seconds(window[1])
        if start is None or end is None or start < 0.0 or end <= start:
            raise ValueError(f"{name} must have finite non-negative start < end")
        return (start, end)

    @staticmethod
    def _in_window(seconds, window):
        return window[0] <= seconds < window[1]

    def forward_lidar(self, replay_seconds):
        seconds = _finite_seconds(replay_seconds)
        if seconds is None:
            return False
        return not (
            self._in_window(seconds, self.lidar_outage)
            or self._in_window(seconds, self.all_outage)
        )

    def forward_imu(self, replay_seconds):
        seconds = _finite_seconds(replay_seconds)
        return seconds is not None and not self._in_window(seconds, self.all_outage)

    def in_any_outage(self, replay_seconds):
        seconds = _finite_seconds(replay_seconds)
        return seconds is not None and (
            self._in_window(seconds, self.lidar_outage)
            or self._in_window(seconds, self.all_outage)
        )


class TfContinuityCollector:
    """Collect replay observations and return every TF-continuity violation."""

    def __init__(self, max_active_tf_spacing_seconds, schedule=None):
        max_spacing = _finite_seconds(max_active_tf_spacing_seconds)
        if max_spacing is None or max_spacing <= 0.0:
            raise ValueError("max_active_tf_spacing_seconds must be positive and finite")
        self.max_active_tf_spacing_seconds = max_spacing
        self.schedule = schedule if schedule is not None else FaultSchedule()
        if not isinstance(self.schedule, FaultSchedule):
            raise ValueError("schedule must be a FaultSchedule")
        self.tf_events = []
        self.diagnostic_events = []
        self.lidar_source_replay_events = []
        self.odom_events = []
        self.path_events = []
        self._lidar_replay_by_sensor_timestamp_us = {}
        self._errors = []
        self._last_tf_timestamp = None
        self._last_auto_trigger = None
        self._stream_recovery_auto_trigger_count = 0
        self._stream_recovery_auto_trigger_replay_seconds = []

    def tf(
        self,
        timestamp_seconds,
        mode=None,
        hold_reason=None,
        replay_seconds=None,
        translation=None,
        rotation=None,
    ):
        timestamp = _finite_seconds(timestamp_seconds)
        replay_time = _finite_seconds(
            timestamp_seconds if replay_seconds is None else replay_seconds
        )
        if timestamp is None or replay_time is None:
            self._errors.append("TF observation has a non-finite timestamp")
            return
        del hold_reason
        if self._last_tf_timestamp is not None:
            spacing = timestamp - self._last_tf_timestamp
            if spacing < 0.0:
                self._errors.append(
                    "TF timestamp regressed: "
                    f"{timestamp:.6f} < {self._last_tf_timestamp:.6f}"
                )
            elif spacing > self.max_active_tf_spacing_seconds:
                self._errors.append(
                    "TF spacing "
                    f"{spacing:.6f}s exceeds {self.max_active_tf_spacing_seconds:.6f}s"
                )
        self._last_tf_timestamp = timestamp

        self.tf_events.append(
            {
                "replay_seconds": replay_time,
                "stamp_seconds": timestamp,
                "mode": _normalized_mode(mode),
                "translation": _finite_vector(translation),
                "rotation": _finite_vector(rotation),
            }
        )

    def diagnostic(self, replay_seconds, values, health_state=None, level=None):
        replay_time = _finite_seconds(replay_seconds)
        if replay_time is None:
            self._errors.append("diagnostic observation has a non-finite replay timestamp")
            return
        if not isinstance(values, Mapping):
            self._errors.append("diagnostic values must be a mapping")
            return
        relevant = {
            key: str(values[key])
            for key in ("tf_mode", "tf_hold_reason", "auto_relocalization_trigger")
            if key in values
        }
        trigger = relevant.get("auto_relocalization_trigger", "none").strip().lower()
        if trigger == "stream_recovered" and self._last_auto_trigger != "stream_recovered":
            self._stream_recovery_auto_trigger_count += 1
            self._stream_recovery_auto_trigger_replay_seconds.append(replay_time)
        self._last_auto_trigger = trigger
        self.diagnostic_events.append(
            {
                "replay_seconds": replay_time,
                "health_state": None if health_state is None else str(health_state),
                "level": level,
                "values": relevant,
            }
        )

    def lidar_source(self, sensor_timestamp_seconds, replay_seconds):
        sensor_timestamp = _finite_seconds(sensor_timestamp_seconds)
        replay_time = _finite_seconds(replay_seconds)
        sensor_timestamp_us = _sensor_timestamp_microseconds(sensor_timestamp)
        if sensor_timestamp is None or sensor_timestamp_us is None:
            self._errors.append("raw lidar observation has a non-finite sensor timestamp")
            return
        if replay_time is None:
            self._errors.append("raw lidar observation has a non-finite replay timestamp")
            return
        self.lidar_source_replay_events.append(
            {
                "sensor_timestamp_seconds": sensor_timestamp,
                "replay_seconds": replay_time,
            }
        )
        self._lidar_replay_by_sensor_timestamp_us.setdefault(
            sensor_timestamp_us, replay_time
        )

    def odom(self, received_replay_seconds, source_timestamp_seconds=None):
        self._record_output_event(
            self.odom_events,
            "odometry",
            received_replay_seconds,
            source_timestamp_seconds,
        )

    def path(self, received_replay_seconds, source_timestamp_seconds=None):
        self._record_output_event(
            self.path_events,
            "path",
            received_replay_seconds,
            source_timestamp_seconds,
        )

    def _record_output_event(
        self,
        events,
        label,
        received_replay_seconds,
        source_timestamp_seconds,
    ):
        received_replay_time = _finite_seconds(received_replay_seconds)
        source_timestamp = _finite_seconds(source_timestamp_seconds)
        if received_replay_time is None:
            self._errors.append(
                f"{label} observation has a non-finite received replay timestamp"
            )
            return
        if source_timestamp is None:
            self._errors.append(
                f"{label} observation has a non-finite source sensor timestamp"
            )
            return
        source_replay_time = self._lookup_lidar_source_replay(source_timestamp)
        events.append(
            {
                "received_replay_seconds": received_replay_time,
                "source_sensor_timestamp_seconds": source_timestamp,
                "source_replay_seconds": source_replay_time,
            }
        )
        if source_replay_time is None:
            self._errors.append(
                f"{label} source timestamp has no raw lidar replay mapping at "
                f"{source_timestamp:.6f}s"
            )

    def _lookup_lidar_source_replay(self, source_timestamp_seconds):
        source_timestamp_us = _sensor_timestamp_microseconds(source_timestamp_seconds)
        if source_timestamp_us is None:
            return None
        return self._lidar_replay_by_sensor_timestamp_us.get(source_timestamp_us)

    def assert_contracts(self):
        """Return clear failures instead of raising, so the JSON artifact is complete."""
        errors = list(self._errors)
        lidar_start, lidar_end = self.schedule.lidar_outage
        all_start, all_end = self.schedule.all_outage

        self._require_tf_and_diagnostic(
            errors,
            "measured map->base_link TF before",
            lambda replay_time: replay_time < lidar_start,
            {"tf_mode": "measured"},
        )
        self._require_tf_and_diagnostic(
            errors,
            "extrapolated map->base_link TF within",
            lambda replay_time: lidar_start
            <= replay_time
            < lidar_start + _EXTRAPOLATION_DEADLINE_SECONDS,
            {"tf_mode": "extrapolated"},
        )
        lidar_held = self._matching_tf(
            lambda event: lidar_start <= event["replay_seconds"] < lidar_end
        )
        if len(lidar_held) < 2:
            errors.append(
                "missing sustained map->base_link TF during "
                f"{lidar_start:.1f}-{lidar_end:.1f}s (observed {len(lidar_held)}, need 2)"
            )
        lidar_held_diagnostics = self._matching_diagnostic(
            lambda replay_time: lidar_start <= replay_time < lidar_end,
            {"tf_mode": "held"},
        )
        if len(lidar_held_diagnostics) < 2:
            errors.append(
                "missing sustained tf_mode=held diagnostics during "
                f"{lidar_start:.1f}-{lidar_end:.1f}s "
                f"(observed {len(lidar_held_diagnostics)}, need 2)"
            )
        all_tf = self._matching_tf(
            lambda event: all_start <= event["replay_seconds"] < all_end
        )
        if len(all_tf) < 2:
            errors.append(
                "missing sustained map->base_link TF during "
                f"{all_start:.1f}-{all_end:.1f}s (observed {len(all_tf)}, need 2)"
            )
        all_held_diagnostics = self._matching_diagnostic(
            lambda replay_time: all_start <= replay_time < all_end,
            {"tf_mode": "held", "tf_hold_reason": "imu_unavailable"},
        )
        if len(all_held_diagnostics) < 2:
            errors.append(
                "missing sustained tf_mode=held, tf_hold_reason=imu_unavailable "
                f"diagnostics during {all_start:.1f}-{all_end:.1f}s "
                f"(observed {len(all_held_diagnostics)}, need 2)"
            )
        self._require_tf_and_diagnostic(
            errors,
            "measured map->base_link TF after",
            lambda replay_time: replay_time >= all_end,
            {"tf_mode": "measured"},
        )

        for label, events in (("odometry", self.odom_events), ("path", self.path_events)):
            for event in events:
                source_replay_time = event["source_replay_seconds"]
                if (
                    source_replay_time is not None
                    and self.schedule.in_any_outage(source_replay_time)
                ):
                    errors.append(
                        f"{label} source timestamp falls within injected outage at "
                        f"{source_replay_time:.6f}s"
                    )

        recovery_windows = self._stream_recovery_windows()
        if self._stream_recovery_auto_trigger_count != len(recovery_windows):
            errors.append(
                "auto_relocalization_trigger=stream_recovered occurred "
                f"{self._stream_recovery_auto_trigger_count} times "
                f"(expected {len(recovery_windows)})"
            )
        for start, end in recovery_windows:
            observed_count = sum(
                start <= replay_time < end
                for replay_time in self._stream_recovery_auto_trigger_replay_seconds
            )
            if observed_count != 1:
                window = (
                    f"{start:.1f}-{end:.1f}s"
                    if math.isfinite(end)
                    else f"{start:.1f}s onward"
                )
                errors.append(
                    "auto_relocalization_trigger=stream_recovered must occur "
                    f"once in recovery window {window} (observed {observed_count})"
                )
        errors.extend(self._spatial_contract_violations())
        return errors

    def report(self, violations):
        return {
            "fault_schedule": asdict(self.schedule),
            "max_active_tf_spacing_seconds": self.max_active_tf_spacing_seconds,
            "tf": self.tf_events,
            "diagnostics": self.diagnostic_events,
            "lidar_source_replay_events": self.lidar_source_replay_events,
            "odometry": self.odom_events,
            "path": self.path_events,
            "stream_recovery_auto_trigger_count": self._stream_recovery_auto_trigger_count,
            "stream_recovery_auto_trigger_replay_seconds": (
                self._stream_recovery_auto_trigger_replay_seconds
            ),
            "spatial_contract": self._spatial_contract_report(),
            "violations": list(violations),
        }

    def _stream_recovery_windows(self):
        outages = sorted((self.schedule.lidar_outage, self.schedule.all_outage))
        windows = [
            (outage[1], next_outage[0])
            for outage, next_outage in zip(outages, outages[1:])
        ]
        windows.append((outages[-1][1], math.inf))
        return windows

    def _has_tf(self, predicate):
        return any(predicate(event) for event in self.tf_events)

    def _matching_tf(self, predicate):
        return [event for event in self.tf_events if predicate(event)]

    def _require_tf_and_diagnostic(self, errors, label, time_predicate, values):
        if not self._has_tf(lambda event: time_predicate(event["replay_seconds"])):
            errors.append(f"missing {label} required replay window")
        if not self._matching_diagnostic(time_predicate, values):
            values_text = ", ".join(f"{key}={value}" for key, value in values.items())
            errors.append(f"missing diagnostic {values_text} for {label} required replay window")

    def _matching_diagnostic(self, time_predicate, expected_values):
        return [
            event
            for event in self.diagnostic_events
            if time_predicate(event["replay_seconds"])
            and all(event["values"].get(key) == value for key, value in expected_values.items())
        ]

    def _spatial_contract(self):
        """Return the forward-window longitudinal edges, or None when not evaluable.

        A TF event counts as a new measurement when it carries an explicit
        ``measured`` mode or, as in the replay, when it is the TF published with
        an adjacent localization odometry message.  The window is only enforced
        when its measured-only trajectory is itself forward-driving, so a
        genuine reverse maneuver is never rejected by a generic no-backward
        rule.
        """
        start = _FORWARD_WINDOW_START_SECONDS
        end = self.schedule.lidar_outage[0]
        if end <= start:
            return None
        measured_flags = self._measured_tf_flags()
        events = [
            (event, measured_flags[index])
            for index, event in enumerate(self.tf_events)
            if start <= event["replay_seconds"] < end
        ]
        if len(events) < 2:
            return None
        measured_edges = []
        measured_events = [
            event for event, measured in events if _is_measured_tf(event, measured)
        ]
        for previous, current in zip(measured_events, measured_events[1:]):
            delta_meters = _body_x_delta(previous, current)
            if delta_meters is not None:
                measured_edges.append(
                    {
                        "replay_seconds": current["replay_seconds"],
                        "delta_meters": delta_meters,
                    }
                )
        fallback_edges = []
        for (previous, previous_measured), (current, current_measured) in zip(
            events, events[1:]
        ):
            if not _is_measured_tf(current, current_measured):
                continue
            if _is_measured_tf(previous, previous_measured):
                continue
            delta_meters = _body_x_delta(previous, current)
            if delta_meters is not None:
                fallback_edges.append(
                    {
                        "replay_seconds": current["replay_seconds"],
                        "delta_meters": delta_meters,
                    }
                )
        if len(measured_edges) < _FORWARD_WINDOW_MIN_MEASURED_EDGES:
            return None
        if statistics.median(edge["delta_meters"] for edge in measured_edges) <= 0.0:
            return None
        return {
            "forward_window_seconds": [start, end],
            "measured_edges": measured_edges,
            "fallback_to_measured_edges": fallback_edges,
        }

    def _measured_tf_flags(self):
        """Mark the TF published with each adjacent localization odometry event.

        ``System::ProcessLocalizationResultCache`` publishes the measured TF
        immediately before its localization odometry, so a result can only
        claim a TF observed at or before its odometry arrival.  Candidates are
        claimed closest-first and one-to-one: a TF already claimed by another
        result, or one that only arrives after the result, is never promoted
        to a measurement.

        Callback timing alone cannot prove the publication source in every
        ordering, so this stays conservative instead of overclaiming.  The
        residual ambiguity is irreducible from the recorded events: when a
        result's own measured TF never arrives, a fallback TF observed within
        ``_MEASURED_TF_ODOM_MATCH_SECONDS`` before that result is
        indistinguishable from its measurement.  Such a fallback can still be
        claimed once, but it can never cover a later TF, can never serve two
        results, and an unmatched result stays unclaimed; under-reporting a
        measurement is preferable to masking a fallback snap-back in the
        spatial contract below.
        """
        flags = [False] * len(self.tf_events)
        candidates = []
        for odometry_index, odometry in enumerate(self.odom_events):
            received = odometry["received_replay_seconds"]
            for tf_index, event in enumerate(self.tf_events):
                gap = received - event["replay_seconds"]
                if 0.0 <= gap <= _MEASURED_TF_ODOM_MATCH_SECONDS:
                    candidates.append((gap, odometry_index, tf_index))
        claimed_tf = set()
        claimed_odometry = set()
        for _, odometry_index, tf_index in sorted(candidates):
            if tf_index in claimed_tf or odometry_index in claimed_odometry:
                continue
            claimed_tf.add(tf_index)
            claimed_odometry.add(odometry_index)
            flags[tf_index] = True
        return flags

    def _spatial_contract_violations(self):
        spatial = self._spatial_contract()
        if spatial is None:
            return []
        fallback_edges = spatial["fallback_to_measured_edges"]
        if not fallback_edges:
            return []
        deltas = [edge["delta_meters"] for edge in fallback_edges]
        median_delta = statistics.median(deltas)
        worst_edge = min(fallback_edges, key=lambda edge: edge["delta_meters"])
        backward_count = sum(delta < 0.0 for delta in deltas)
        window_start, window_end = spatial["forward_window_seconds"]
        window = f"{window_start:.1f}-{window_end:.1f}s"
        violations = []
        if median_delta < _SPATIAL_BACKWARD_MEDIAN_LIMIT_METERS:
            violations.append(
                "spatial systematic backward map->base_link snap-back in the "
                f"forward window {window}: {backward_count}/{len(fallback_edges)} "
                f"fallback->measured edges moved backward (median {median_delta:.5f}m)"
            )
        if worst_edge["delta_meters"] < -_SPATIAL_BACKWARD_EDGE_LIMIT_METERS:
            violations.append(
                "spatial backward map->base_link snap-back of "
                f"{-worst_edge['delta_meters']:.5f}m at "
                f"{worst_edge['replay_seconds']:.3f}s exceeds the "
                f"{_SPATIAL_BACKWARD_EDGE_LIMIT_METERS:.5f}m single-edge limit "
                f"in the forward window {window}"
            )
        return violations

    def _spatial_contract_report(self):
        start = _FORWARD_WINDOW_START_SECONDS
        end = self.schedule.lidar_outage[0]
        spatial = self._spatial_contract()
        if spatial is None:
            return {
                "forward_window_seconds": [start, end],
                "evaluated": False,
            }
        measured_deltas = [edge["delta_meters"] for edge in spatial["measured_edges"]]
        fallback_deltas = [
            edge["delta_meters"] for edge in spatial["fallback_to_measured_edges"]
        ]
        report = {
            "forward_window_seconds": spatial["forward_window_seconds"],
            "evaluated": True,
            "measured_edges": len(measured_deltas),
            "measured_median_delta_meters": statistics.median(measured_deltas),
            "fallback_to_measured_edges": len(fallback_deltas),
            "backward_fallback_edges": sum(delta < 0.0 for delta in fallback_deltas),
        }
        if fallback_deltas:
            report["fallback_median_delta_meters"] = statistics.median(fallback_deltas)
            report["fallback_worst_delta_meters"] = min(fallback_deltas)
        return report


def _finite_vector(values):
    if values is None:
        return None
    try:
        vector = tuple(_finite_seconds(value) for value in values)
    except TypeError:
        return None
    return list(vector) if all(value is not None for value in vector) else None


def _normalized_mode(value):
    if not isinstance(value, str):
        return None
    normalized = value.strip().lower()
    return normalized or None


def _is_measured_tf(event, matched_to_odometry):
    if event["mode"] is not None:
        return event["mode"] == "measured"
    return matched_to_odometry


def _body_x_delta(previous, current):
    """Project the TF translation delta onto the previous body X axis."""
    previous_translation = previous["translation"]
    current_translation = current["translation"]
    rotation = previous["rotation"]
    if (
        previous_translation is None
        or current_translation is None
        or rotation is None
        or len(previous_translation) < 2
        or len(current_translation) < 2
        or len(rotation) != 4
    ):
        return None
    x, y, z, w = rotation
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    delta_x = current_translation[0] - previous_translation[0]
    delta_y = current_translation[1] - previous_translation[1]
    return delta_x * math.cos(yaw) + delta_y * math.sin(yaw)


class ReplayError(RuntimeError):
    pass


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="Verify map->base_link TF continuity through deterministic sensor outages."
    )
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--map", required=True, type=Path)
    parser.add_argument("--params-template", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument(
        "--post-recovery-seconds",
        type=float,
        default=10.0,
        help="Replay time to observe after the all-sensor outage (default: 10).",
    )
    return parser


def main(argv=None):
    arguments = build_argument_parser().parse_args(argv)
    schedule = FaultSchedule()
    collector = TfContinuityCollector(_MAX_ACTIVE_TF_SPACING_SECONDS, schedule)
    output_dir = arguments.output_dir
    identity = None
    runtime_parameters = None
    error_message = None
    violations = []
    exit_code = 0

    try:
        post_recovery_seconds = _positive_seconds(
            arguments.post_recovery_seconds, "--post-recovery-seconds"
        )
        output_dir.mkdir(parents=True, exist_ok=True)
        identity = validate_dataset(arguments.bag, arguments.map)
        runtime_parameters = write_runtime_parameters(
            arguments.params_template,
            arguments.map,
            output_dir,
            topic_overrides={
                "lidar_topic": _RELAY_LIDAR_TOPIC,
                "imu_topic": _RELAY_IMU_TOPIC,
            },
        )
        _run_replay(arguments, runtime_parameters, collector, schedule, post_recovery_seconds)
        violations = collector.assert_contracts()
        if violations:
            exit_code = 1
            for violation in violations:
                print(f"TF continuity violation: {violation}", file=sys.stderr)
    except KeyboardInterrupt:
        exit_code = 130
        error_message = "interrupted"
    except (DatasetError, ReplayError, OSError, ValueError) as error:
        exit_code = 2
        error_message = str(error)
        print(f"TF continuity replay failed: {error_message}", file=sys.stderr)
    except Exception as error:
        exit_code = 2
        error_message = f"unexpected {type(error).__name__}: {error}"
        print(f"TF continuity replay failed: {error_message}", file=sys.stderr)
    finally:
        if error_message is not None:
            violations = list(violations) + [f"harness error: {error_message}"]
        try:
            _write_artifact(
                output_dir,
                collector,
                identity,
                runtime_parameters,
                violations,
                error_message,
            )
        except OSError as error:
            exit_code = 2
            print(f"unable to write TF continuity artifact: {error}", file=sys.stderr)
    return exit_code


def _positive_seconds(value, label):
    seconds = _finite_seconds(value)
    if seconds is None or seconds <= 0.0:
        raise ValueError(f"{label} must be positive and finite")
    return seconds


def _write_artifact(output_dir, collector, identity, runtime_parameters, violations, error_message):
    output_dir.mkdir(parents=True, exist_ok=True)
    artifact = collector.report(violations)
    artifact["dataset"] = None if identity is None else {
        "duration_ns": identity.duration_ns,
        "message_count": identity.message_count,
        "topic_counts": dict(identity.topic_counts),
        "map_points": identity.map_points,
    }
    artifact["runtime_parameters"] = (
        None if runtime_parameters is None else str(runtime_parameters)
    )
    artifact["error"] = error_message
    (output_dir / "tf-continuity.json").write_text(
        json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _run_replay(arguments, runtime_parameters, collector, schedule, post_recovery_seconds):
    """Run the ROS relay, launch, and player until post-outage recovery is observable."""
    import rclpy
    from rclpy.parameter import Parameter

    domain_id = select_ros_domain_id()
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = domain_id
    previous_domain_id = os.environ.get("ROS_DOMAIN_ID")
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
        "1.0",
        "--start-offset",
        "0",
        "--disable-keyboard-controls",
    ]

    launch_process = None
    player_process = None
    node = None
    rclpy_initialized = False
    launch_log = None
    player_log = None
    try:
        arguments.output_dir.mkdir(parents=True, exist_ok=True)
        launch_log = (arguments.output_dir / "launch.log").open("w", encoding="utf-8")
        player_log = (arguments.output_dir / "bag-play.log").open("w", encoding="utf-8")
        rclpy.init(args=None)
        rclpy_initialized = True
        node = rclpy.create_node(
            "localization_tf_continuity_harness",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        relay = _TfContinuityRelay(node, collector, schedule)
        launch_process = subprocess.Popen(
            launch_command,
            stdout=launch_log,
            stderr=subprocess.STDOUT,
            env=environment,
            start_new_session=True,
        )
        _wait_for_launch(node, launch_process)
        player_process = subprocess.Popen(
            player_command,
            stdout=player_log,
            stderr=subprocess.STDOUT,
            env=environment,
            start_new_session=True,
        )
        _spin_fault_window(
            node,
            launch_process,
            player_process,
            relay,
            schedule.all_outage[1] + post_recovery_seconds,
        )
    finally:
        cleanup_error = None
        for process in (player_process, launch_process):
            try:
                _stop_process_group(process)
            except ReplayError as error:
                if cleanup_error is None:
                    cleanup_error = error
        if node is not None:
            node.destroy_node()
        if rclpy_initialized:
            rclpy.shutdown()
        if launch_log is not None:
            launch_log.close()
        if player_log is not None:
            player_log.close()
        if previous_domain_id is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous_domain_id
        if cleanup_error is not None and sys.exc_info()[0] is None:
            raise cleanup_error


class _TfContinuityRelay:
    """One ROS node that gates raw inputs and records output at replay time."""

    def __init__(self, node, collector, schedule):
        from diagnostic_msgs.msg import DiagnosticArray
        from nav_msgs.msg import Odometry, Path
        from rclpy.qos import (
            QoSHistoryPolicy,
            QoSProfile,
            QoSReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, PointCloud2
        from tf2_msgs.msg import TFMessage

        self.node = node
        self.collector = collector
        self.schedule = schedule
        self._first_clock_seconds = None
        self._latest_clock_seconds = None
        sensor_qos = qos_profile_sensor_data
        reliable_qos = QoSProfile(depth=100, reliability=QoSReliabilityPolicy.RELIABLE)
        lidar_relay_config, imu_relay_config = relay_qos_config()
        lidar_relay_qos = QoSProfile(
            depth=lidar_relay_config.depth,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        imu_relay_qos = QoSProfile(
            depth=imu_relay_config.depth,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._lidar_publisher = node.create_publisher(
            PointCloud2, _RELAY_LIDAR_TOPIC, lidar_relay_qos
        )
        self._imu_publisher = node.create_publisher(Imu, _RELAY_IMU_TOPIC, imu_relay_qos)
        node.create_subscription(Clock, "/clock", self._clock_callback, sensor_qos)
        node.create_subscription(PointCloud2, _RAW_LIDAR_TOPIC, self._lidar_callback, sensor_qos)
        node.create_subscription(Imu, _RAW_IMU_TOPIC, self._imu_callback, sensor_qos)
        node.create_subscription(TFMessage, "/tf", self._tf_callback, reliable_qos)
        node.create_subscription(DiagnosticArray, "/diagnostics", self._diagnostics_callback, reliable_qos)
        node.create_subscription(Odometry, _LOCALIZATION_ODOM_TOPIC, self._odom_callback, reliable_qos)
        node.create_subscription(Path, _LOCALIZATION_PATH_TOPIC, self._path_callback, reliable_qos)

    @property
    def replay_seconds(self):
        if self._first_clock_seconds is None or self._latest_clock_seconds is None:
            return None
        return self._latest_clock_seconds - self._first_clock_seconds

    def _clock_callback(self, message):
        seconds = _ros_time_seconds(message.clock)
        if seconds is None:
            return
        if self._first_clock_seconds is None:
            self._first_clock_seconds = seconds
        self._latest_clock_seconds = seconds

    def _lidar_callback(self, message):
        replay_seconds = self._sensor_replay_seconds(message)
        if replay_seconds is not None:
            self.collector.lidar_source(
                _message_header_timestamp_seconds(message), replay_seconds
            )
        if replay_seconds is not None and self.schedule.forward_lidar(replay_seconds):
            self._lidar_publisher.publish(message)

    def _imu_callback(self, message):
        replay_seconds = self._sensor_replay_seconds(message)
        if replay_seconds is not None and self.schedule.forward_imu(replay_seconds):
            self._imu_publisher.publish(message)

    def _sensor_replay_seconds(self, message):
        if self._first_clock_seconds is None:
            return None
        return self._message_replay_seconds(message)

    def _message_replay_seconds(self, message):
        if self._first_clock_seconds is None:
            return None
        header_stamp = _message_header_timestamp_seconds(message)
        if header_stamp is not None and header_stamp >= self._first_clock_seconds - 1.0:
            return header_stamp - self._first_clock_seconds
        return self.replay_seconds

    def _tf_callback(self, message):
        replay_seconds = self.replay_seconds
        if replay_seconds is None:
            return
        for transform in message.transforms:
            if not _is_map_to_base_link(transform):
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self.collector.tf(
                _ros_time_seconds(transform.header.stamp),
                replay_seconds=replay_seconds,
                translation=(translation.x, translation.y, translation.z),
                rotation=(rotation.x, rotation.y, rotation.z, rotation.w),
            )

    def _diagnostics_callback(self, message):
        replay_seconds = self._message_replay_seconds(message)
        if replay_seconds is None:
            return
        for status in message.status:
            if status.name != _TARGET_DIAGNOSTIC:
                continue
            values = {value.key: value.value for value in status.values}
            self.collector.diagnostic(
                replay_seconds,
                values,
                health_state=status.message,
                level=parse_diagnostic_level(status.level),
            )

    def _odom_callback(self, message):
        received_replay_seconds = self.replay_seconds
        if received_replay_seconds is not None:
            self.collector.odom(
                received_replay_seconds,
                source_timestamp_seconds=_message_header_timestamp_seconds(message),
            )

    def _path_callback(self, message):
        received_replay_seconds = self.replay_seconds
        if received_replay_seconds is not None:
            self.collector.path(
                received_replay_seconds,
                source_timestamp_seconds=path_source_timestamp_seconds(message),
            )


def _is_map_to_base_link(transform):
    return (
        transform.header.frame_id.lstrip("/") == "map"
        and transform.child_frame_id.lstrip("/") == "base_link"
    )


def _ros_time_seconds(time_message):
    if time_message is None:
        return None
    seconds = _finite_seconds(getattr(time_message, "sec", None))
    nanoseconds = _finite_seconds(getattr(time_message, "nanosec", None))
    if seconds is None or nanoseconds is None:
        return None
    return seconds + nanoseconds * 1e-9


def _message_header_timestamp_seconds(message):
    header = getattr(message, "header", None)
    return _ros_time_seconds(getattr(header, "stamp", None))


def path_source_timestamp_seconds(message):
    """Return the latest Path pose source timestamp, not the publication header."""
    poses = getattr(message, "poses", None)
    if not poses:
        return None
    return _message_header_timestamp_seconds(poses[-1])


def _wait_for_launch(node, launch_process):
    deadline = time.monotonic() + _STARTUP_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        if launch_process.poll() is not None:
            raise ReplayError(
                f"launch process exited during startup with code {launch_process.returncode}"
            )
        _spin_once(node, min(0.1, max(0.0, deadline - time.monotonic())))


def _spin_fault_window(node, launch_process, player_process, relay, target_replay_seconds):
    timeout_seconds = target_replay_seconds + _PLAYBACK_TIMEOUT_ALLOWANCE_SECONDS
    deadline = time.monotonic() + timeout_seconds
    while True:
        if launch_process.poll() is not None:
            raise ReplayError(
                f"launch process exited during playback with code {launch_process.returncode}"
            )
        player_exit_code = player_process.poll()
        if player_exit_code is not None:
            if player_exit_code != 0:
                raise ReplayError(f"bag player exited during playback with code {player_exit_code}")
            raise ReplayError("bag player exited before the recovery observation window")
        if time.monotonic() >= deadline:
            raise ReplayError(
                "replay timed out waiting for /clock progress through "
                f"{target_replay_seconds:.1f}s"
            )
        _spin_once(node, min(0.1, max(0.0, deadline - time.monotonic())))
        replay_seconds = relay.replay_seconds
        if replay_seconds is not None and replay_seconds >= target_replay_seconds:
            for _ in range(_FINAL_CALLBACK_DRAIN_COUNT):
                _spin_once(node, 0.0)
            return


def _spin_once(node, timeout_seconds):
    import rclpy

    rclpy.spin_once(node, timeout_sec=timeout_seconds)


def _stop_process_group(process):
    if process is None:
        return
    for signal_number in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, signal_number)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=_PROCESS_STOP_TIMEOUT_SECONDS)
            return
        except subprocess.TimeoutExpired:
            continue
    raise ReplayError(f"process group {process.pid} could not be reaped after SIGKILL")


if __name__ == "__main__":
    sys.exit(main())
