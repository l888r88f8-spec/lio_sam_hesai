#!/usr/bin/env python3
"""Pure contracts for the localization TF-continuity replay harness."""

import sys
import unittest
from contextlib import redirect_stderr
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch


INTEGRATION_DIRECTORY = Path(__file__).parent / "integration"
sys.path.insert(0, str(INTEGRATION_DIRECTORY))

import localization_tf_continuity
from localization_tf_continuity import (
    FaultSchedule,
    TfContinuityCollector,
    parse_diagnostic_level,
    main,
    relay_qos_config,
    select_ros_domain_id,
)
from stability_common import DatasetError, write_runtime_parameters


class FaultScheduleTest(unittest.TestCase):
    def test_schedule_keeps_imu_during_lidar_only_outage(self):
        schedule = FaultSchedule(lidar_outage=(30.0, 40.0), all_outage=(50.0, 60.0))

        self.assertFalse(schedule.forward_lidar(35.0))
        self.assertTrue(schedule.forward_imu(35.0))
        self.assertFalse(schedule.forward_lidar(55.0))
        self.assertFalse(schedule.forward_imu(55.0))


class RosDomainSelectionTest(unittest.TestCase):
    def test_domain_selection_respects_explicit_valid_environment(self):
        self.assertEqual(select_ros_domain_id({"ROS_DOMAIN_ID": "25"}, pid=999), "25")

    def test_domain_selection_falls_back_for_invalid_or_out_of_range_environment(self):
        for value in ("0", "101", "-1", "not-a-domain"):
            with self.subTest(value=value):
                self.assertEqual(
                    select_ros_domain_id({"ROS_DOMAIN_ID": value}, pid=24), "25"
                )

    def test_domain_selection_fallback_stays_in_safe_range(self):
        for pid in (0, 1, 99, 100, 12345):
            with self.subTest(pid=pid):
                domain = int(select_ros_domain_id({}, pid=pid))
                self.assertGreaterEqual(domain, 1)
                self.assertLessEqual(domain, 100)


class RelayRuntimeContractTest(unittest.TestCase):
    def test_diagnostic_level_parser_accepts_int_and_singleton_bytes(self):
        self.assertEqual(parse_diagnostic_level(1), 1)
        self.assertEqual(parse_diagnostic_level(b"\x01"), 1)

    def test_diagnostic_level_parser_rejects_unsupported_values(self):
        for value in (True, "1", b"\x01\x02", None):
            with self.subTest(value=value):
                self.assertIsNone(parse_diagnostic_level(value))

    def test_relay_qos_config_matches_reliable_sensor_consumers(self):
        lidar_qos, imu_qos = relay_qos_config()

        self.assertEqual(lidar_qos.reliability, "reliable")
        self.assertEqual(lidar_qos.depth, 5)
        self.assertEqual(imu_qos.reliability, "reliable")
        self.assertEqual(imu_qos.depth, 2000)


class TfContinuityCollectorTest(unittest.TestCase):
    _TF_SAMPLES = (29.0, 30.1, 31.0, 39.0, 55.0, 55.1, 65.0)
    _DIAGNOSTICS = (
        (29.0, {"tf_mode": "measured"}),
        (30.1, {"tf_mode": "extrapolated"}),
        (31.0, {"tf_mode": "held"}),
        (39.0, {"tf_mode": "held"}),
        (55.0, {"tf_mode": "held", "tf_hold_reason": "imu_unavailable"}),
        (55.1, {"tf_mode": "held", "tf_hold_reason": "imu_unavailable"}),
        (65.0, {"tf_mode": "measured"}),
    )

    def _record_contract(
        self,
        collector,
        diagnostic_order=None,
        tf_mode=None,
        include_recovery_triggers=True,
    ):
        for timestamp in self._TF_SAMPLES:
            collector.tf(timestamp, mode=tf_mode)
        if diagnostic_order is None:
            diagnostic_order = range(len(self._DIAGNOSTICS))
        for index in diagnostic_order:
            timestamp, values = self._DIAGNOSTICS[index]
            collector.diagnostic(timestamp, values)
        if include_recovery_triggers:
            collector.diagnostic(40.1, {"auto_relocalization_trigger": "stream_recovered"})
            collector.diagnostic(50.1, {"auto_relocalization_trigger": "lost_entry"})
            collector.diagnostic(60.1, {"auto_relocalization_trigger": "stream_recovered"})

    def test_collector_requires_each_fallback_mode_and_recovery(self):
        collector = TfContinuityCollector(20.0)
        self._record_contract(collector)

        self.assertEqual(collector.assert_contracts(), [])

    def test_collector_validates_timestamped_diagnostics_not_tf_arrival_order(self):
        collector = TfContinuityCollector(20.0)
        self._record_contract(
            collector,
            diagnostic_order=(4, 0, 6, 1, 3, 2, 5),
            tf_mode="held",
        )

        self.assertEqual(collector.assert_contracts(), [])

    def test_collector_retains_tf_staleness_and_prediction_age(self):
        collector = TfContinuityCollector(20.0)
        collector.diagnostic(
            31.0,
            {
                "tf_mode": "held",
                "tf_pose_stale": "true",
                "tf_extrapolation_age_ms": "0.000000",
                "covariance_scale": "100.000000",
            },
            health_state="LOST",
            level=2,
        )

        self.assertEqual(
            collector.diagnostic_events[0]["values"],
            {
                "tf_mode": "held",
                "tf_pose_stale": "true",
                "tf_extrapolation_age_ms": "0.000000",
                "covariance_scale": "100.000000",
            },
        )

    def test_collector_reports_tf_timestamp_regression(self):
        collector = TfContinuityCollector(20.0)
        collector.tf(29.0)
        collector.tf(28.9)

        self.assertTrue(
            any("TF timestamp regressed" in error for error in collector.assert_contracts())
        )

    def test_collector_reports_too_large_tf_spacing(self):
        collector = TfContinuityCollector(0.10)
        collector.tf(29.0)
        collector.tf(29.11)

        self.assertTrue(
            any("TF spacing" in error for error in collector.assert_contracts())
        )

    def test_collector_reports_too_large_held_tf_spacing(self):
        collector = TfContinuityCollector(0.10)
        collector.tf(29.0)
        collector.tf(29.05)
        collector.tf(30.0)
        collector.tf(30.20)

        self.assertTrue(
            any("TF spacing" in error for error in collector.assert_contracts())
        )

    def test_collector_requires_sustained_held_tf_in_each_outage(self):
        collector = TfContinuityCollector(20.0)
        for timestamp in (29.0, 30.1, 31.0, 55.0, 65.0):
            collector.tf(timestamp)
        for index in (0, 1, 2, 4, 6):
            timestamp, values = self._DIAGNOSTICS[index]
            collector.diagnostic(timestamp, values)

        self.assertTrue(
            any("missing sustained map->base_link TF" in error for error in collector.assert_contracts())
        )

    def test_collector_requires_one_deduplicated_recovery_trigger_per_fault_edge(self):
        collector = TfContinuityCollector(20.0)
        self._record_contract(collector, include_recovery_triggers=False)

        collector.diagnostic(40.1, {"auto_relocalization_trigger": "stream_recovered"})
        collector.diagnostic(40.2, {"auto_relocalization_trigger": "stream_recovered"})
        collector.diagnostic(50.1, {"auto_relocalization_trigger": "lost_entry"})
        collector.diagnostic(60.1, {"auto_relocalization_trigger": "stream_recovered"})

        self.assertEqual(collector.assert_contracts(), [])

    def test_collector_rejects_recovery_trigger_outside_its_recovery_window(self):
        collector = TfContinuityCollector(20.0)
        self._record_contract(collector, include_recovery_triggers=False)
        collector.diagnostic(39.9, {"auto_relocalization_trigger": "stream_recovered"})
        collector.diagnostic(50.1, {"auto_relocalization_trigger": "lost_entry"})
        collector.diagnostic(60.1, {"auto_relocalization_trigger": "stream_recovered"})

        self.assertTrue(
            any("recovery window" in error for error in collector.assert_contracts())
        )


class SpatialForwardWindowContractTest(unittest.TestCase):
    """The known forward-driving 10-30s window must not snap backward.

    A new measured TF arrives adjacent to its localization odometry result; the
    fallback TFs in between have no adjacent odometry.  Every fallback->measured
    transition is projected onto the previous body X axis, so a genuine reverse
    maneuver (which only ever shows backward measured edges) must not be
    mistaken for the sawtooth caused by publishing a stale measurement.
    """

    _WINDOW_START_SECONDS = 10.0
    _WINDOW_END_SECONDS = 30.0
    _SAMPLE_INTERVAL_SECONDS = 0.2
    _SENSOR_CLOCK_OFFSET_SECONDS = 60.0
    _EXTRAPOLATION_OVERSHOOT_METERS = 0.11
    _MEASUREMENT_SNAP_BACK_METERS = 0.075

    def _spatial_violations(self, collector):
        return [error for error in collector.assert_contracts() if "spatial" in error]

    def _forward_window_evaluated(self, collector):
        return collector.report([])["spatial_contract"]["evaluated"]

    def _backward_fallback_steps(self, collector):
        """Backward fallback->measured steps a generic no-backward rule would flag."""
        odometry_times = [
            event["received_replay_seconds"] for event in collector.odom_events
        ]

        def measured(event):
            return any(
                abs(received - event["replay_seconds"]) <= 0.02
                for received in odometry_times
            )

        return [
            current
            for previous, current in zip(collector.tf_events, collector.tf_events[1:])
            if measured(current)
            and not measured(previous)
            and current["translation"][0] - previous["translation"][0] < -0.05
        ]

    def _record_measured_tf(self, collector, replay_seconds, x_meters):
        sensor_timestamp = replay_seconds + self._SENSOR_CLOCK_OFFSET_SECONDS
        collector.lidar_source(sensor_timestamp, replay_seconds)
        # System::ProcessLocalizationResultCache publishes the measured TF
        # immediately before its localization odometry, so record it first.
        collector.tf(
            replay_seconds + 0.001,
            translation=(x_meters, 0.0, 0.0),
            rotation=(0.0, 0.0, 0.0, 1.0),
        )
        collector.odom(replay_seconds + 0.002, source_timestamp_seconds=sensor_timestamp)

    def _record_fallback_tf(self, collector, replay_seconds, x_meters):
        collector.tf(
            replay_seconds,
            translation=(x_meters, 0.0, 0.0),
            rotation=(0.0, 0.0, 0.0, 1.0),
        )

    def test_measured_only_forward_drive_reports_no_spatial_violation(self):
        collector = TfContinuityCollector(20.0)
        for index in range(100):
            self._record_measured_tf(
                collector,
                self._WINDOW_START_SECONDS + index * self._SAMPLE_INTERVAL_SECONDS,
                index * 0.1,
            )

        self.assertEqual(self._spatial_violations(collector), [])
        self.assertTrue(self._forward_window_evaluated(collector))

    def test_forward_drive_with_small_prediction_error_reports_no_spatial_violation(self):
        collector = TfContinuityCollector(20.0)
        position = 0.0
        for index in range(100):
            replay_seconds = (
                self._WINDOW_START_SECONDS + index * self._SAMPLE_INTERVAL_SECONDS
            )
            self._record_measured_tf(collector, replay_seconds, position)
            self._record_fallback_tf(
                collector, replay_seconds + 0.1, position + 0.105
            )
            position += 0.1

        self.assertEqual(self._spatial_violations(collector), [])
        self.assertTrue(self._forward_window_evaluated(collector))

    def test_extrapolation_overshoot_before_a_new_measurement_is_a_spatial_violation(self):
        collector = TfContinuityCollector(20.0)
        position = 0.0
        index = 0
        while (
            self._WINDOW_START_SECONDS
            + (index + 1) * self._SAMPLE_INTERVAL_SECONDS
            <= self._WINDOW_END_SECONDS
        ):
            replay_seconds = (
                self._WINDOW_START_SECONDS + index * self._SAMPLE_INTERVAL_SECONDS
            )
            self._record_measured_tf(collector, replay_seconds, position)
            self._record_fallback_tf(
                collector,
                replay_seconds + self._SAMPLE_INTERVAL_SECONDS / 2.0,
                position + self._EXTRAPOLATION_OVERSHOOT_METERS,
            )
            position += (
                self._EXTRAPOLATION_OVERSHOOT_METERS
                - self._MEASUREMENT_SNAP_BACK_METERS
            )
            index += 1

        violations = self._spatial_violations(collector)

        self.assertTrue(
            violations, "expected the periodic +0.11m/-0.075m sawtooth to fail"
        )
        self.assertTrue(
            any("snap-back" in violation for violation in violations), violations
        )

    def test_single_large_backward_edge_is_a_spatial_violation(self):
        collector = TfContinuityCollector(20.0)
        position = 0.0
        for index in range(100):
            replay_seconds = (
                self._WINDOW_START_SECONDS + index * self._SAMPLE_INTERVAL_SECONDS
            )
            self._record_measured_tf(collector, replay_seconds, position)
            overshoot = 0.3 if index in (40, 60, 80) else 0.105
            self._record_fallback_tf(collector, replay_seconds + 0.1, position + overshoot)
            position += 0.1

        violations = self._spatial_violations(collector)

        self.assertTrue(violations)
        self.assertTrue(
            any(
                "single-edge limit" in violation and "0.20000m" in violation
                for violation in violations
            ),
            violations,
        )

    def test_reverse_measured_travel_is_not_rejected_by_a_no_backward_rule(self):
        collector = TfContinuityCollector(20.0)
        position = 20.0
        for index in range(100):
            replay_seconds = (
                self._WINDOW_START_SECONDS + index * self._SAMPLE_INTERVAL_SECONDS
            )
            self._record_measured_tf(collector, replay_seconds, position)
            if index < 99:
                self._record_fallback_tf(
                    collector,
                    replay_seconds + self._SAMPLE_INTERVAL_SECONDS / 2.0,
                    position,
                )
            position -= 0.1

        self.assertEqual(self._spatial_violations(collector), [])
        # This recording does contain backward fallback->measured steps; the
        # window is skipped because its measured travel itself is backward.
        self.assertTrue(self._backward_fallback_steps(collector))
        self.assertFalse(self._forward_window_evaluated(collector))


class MeasuredTfOdomMatchingContractTest(unittest.TestCase):
    """A localization odometry result owns exactly one adjacent measured TF.

    ``System::ProcessLocalizationResultCache`` calls ``PublishTF`` immediately
    before it publishes the localization odometry, so a result may only claim a
    TF observed at or before the odometry arrival, and no TF may be claimed by
    two results.  Callback timing cannot prove the publication source in every
    ordering, so the matcher must stay conservative instead of promoting a
    fallback TF that merely happens to be nearby.
    """

    _SENSOR_CLOCK_OFFSET_SECONDS = 60.0

    def _record_odometry(self, collector, received_replay_seconds):
        sensor_timestamp = received_replay_seconds + self._SENSOR_CLOCK_OFFSET_SECONDS
        collector.lidar_source(sensor_timestamp, received_replay_seconds)
        collector.odom(received_replay_seconds, source_timestamp_seconds=sensor_timestamp)

    def _record_tf(self, collector, replay_seconds):
        collector.tf(
            replay_seconds,
            translation=(0.0, 0.0, 0.0),
            rotation=(0.0, 0.0, 0.0, 1.0),
        )

    def test_one_tf_is_not_claimed_by_two_odometry_results(self):
        collector = TfContinuityCollector(20.0)
        # Ground truth: TF 0 and TF 1 each belong to one of the two results.
        self._record_tf(collector, 11.995)
        self._record_tf(collector, 12.000)
        self._record_odometry(collector, 12.0005)
        self._record_odometry(collector, 12.0010)

        # The old matcher lets both results claim TF 1, so TF 0 is silently
        # demoted to a fallback.
        self.assertEqual(collector._measured_tf_flags(), [True, True])

    def test_tf_after_the_odometry_is_not_claimed_as_its_measurement(self):
        collector = TfContinuityCollector(20.0)
        self._record_odometry(collector, 12.000)
        # Published after the result, so under the C++ publication order this
        # can only be a fallback even though it falls inside the match window.
        self._record_tf(collector, 12.010)

        self.assertEqual(collector._measured_tf_flags(), [False])


class OutputSourceTimeContractTest(unittest.TestCase):
    def test_output_source_time_uses_the_exact_raw_lidar_replay_mapping(self):
        collector = TfContinuityCollector(20.0)
        collector.lidar_source(66.45, 0.318)
        collector.lidar_source(98.367, 60.701)
        collector.odom(60.840, source_timestamp_seconds=98.367)
        collector.path(60.840, source_timestamp_seconds=98.367)

        errors = collector.assert_contracts()

        self.assertFalse(any(error.startswith("odometry source") for error in errors))
        self.assertFalse(any(error.startswith("path source") for error in errors))
        self.assertEqual(
            collector.odom_events,
            [
                {
                    "received_replay_seconds": 60.84,
                    "source_sensor_timestamp_seconds": 98.367,
                    "source_replay_seconds": 60.701,
                }
            ],
        )
        self.assertEqual(
            collector.path_events,
            [
                {
                    "received_replay_seconds": 60.84,
                    "source_sensor_timestamp_seconds": 98.367,
                    "source_replay_seconds": 60.701,
                }
            ],
        )
        self.assertEqual(
            collector.report([])["lidar_source_replay_events"],
            [
                {"sensor_timestamp_seconds": 66.45, "replay_seconds": 0.318},
                {"sensor_timestamp_seconds": 98.367, "replay_seconds": 60.701},
            ],
        )

    def test_output_with_source_time_in_outage_is_a_violation(self):
        collector = TfContinuityCollector(20.0)
        collector.lidar_source(98.367, 50.01)
        collector.odom(50.03, source_timestamp_seconds=98.367)
        collector.path(50.03, source_timestamp_seconds=98.367)

        errors = collector.assert_contracts()

        self.assertIn(
            "odometry source timestamp falls within injected outage at 50.010000s",
            errors,
        )
        self.assertIn(
            "path source timestamp falls within injected outage at 50.010000s",
            errors,
        )

    def test_output_with_unknown_source_timestamp_reports_a_mapping_error(self):
        collector = TfContinuityCollector(20.0)
        collector.odom(50.03, source_timestamp_seconds=98.367)
        collector.path(50.03, source_timestamp_seconds=98.367)

        errors = collector.assert_contracts()

        self.assertIn(
            "odometry source timestamp has no raw lidar replay mapping at 98.367000s",
            errors,
        )
        self.assertIn(
            "path source timestamp has no raw lidar replay mapping at 98.367000s",
            errors,
        )
        self.assertFalse(any("falls within injected outage" in error for error in errors))
        self.assertEqual(
            collector.odom_events,
            [
                {
                    "received_replay_seconds": 50.03,
                    "source_sensor_timestamp_seconds": 98.367,
                    "source_replay_seconds": None,
                }
            ],
        )

    def test_path_source_time_uses_the_last_pose_header(self):
        def stamp(seconds):
            whole_seconds = int(seconds)
            return SimpleNamespace(
                sec=whole_seconds,
                nanosec=int((seconds - whole_seconds) * 1.0e9),
            )

        path = SimpleNamespace(
            header=SimpleNamespace(stamp=stamp(50.03)),
            poses=[
                SimpleNamespace(header=SimpleNamespace(stamp=stamp(49.85))),
                SimpleNamespace(header=SimpleNamespace(stamp=stamp(49.95))),
            ],
        )

        self.assertEqual(
            localization_tf_continuity.path_source_timestamp_seconds(path), 49.95
        )


class ArtifactFailureTest(unittest.TestCase):
    def test_main_returns_nonzero_when_artifact_write_fails(self):
        identity = SimpleNamespace(
            duration_ns=1,
            message_count=1,
            topic_counts={},
            map_points=1,
        )
        with TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            arguments = [
                "--bag", str(root / "bag"),
                "--map", str(root / "map"),
                "--params-template", str(root / "params.yaml"),
                "--output-dir", str(root / "output"),
            ]
            with redirect_stderr(StringIO()), patch(
                "localization_tf_continuity.validate_dataset", return_value=identity
            ), patch(
                "localization_tf_continuity.write_runtime_parameters",
                return_value=root / "runtime.yaml",
            ), patch("localization_tf_continuity._run_replay"), patch(
                "localization_tf_continuity._write_artifact",
                side_effect=OSError("artifact disk full"),
            ):
                self.assertEqual(main(arguments), 2)


class RuntimeParameterTopicOverrideTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.template = self.root / "relocalization.yaml"
        self.map_dir = self.root / "map"
        self.map_dir.mkdir()

    def tearDown(self):
        self.temporary_directory.cleanup()

    def _write_template(self, sensor_topic):
        self.template.write_text(
            """/**:
  ros__parameters:
    sensor_topic:
"""
            + sensor_topic
            + """    localization:
      map_path: old-map
      ground_map_path: old-ground
      sc_db_path: old-db
""",
            encoding="utf-8",
        )

    def test_write_runtime_parameters_replaces_and_quotes_two_sensor_topics(self):
        self._write_template(
            """      lidar_topic: /lidar_points
      imu_topic: /imu
"""
        )

        runtime_path = write_runtime_parameters(
            self.template,
            self.map_dir,
            self.root / "runtime",
            topic_overrides={
                "lidar_topic": "/tf_continuity_test/lidar_points",
                "imu_topic": "/tf_continuity_test/imu",
            },
        )

        contents = runtime_path.read_text(encoding="utf-8")
        self.assertIn('lidar_topic: "/tf_continuity_test/lidar_points"', contents)
        self.assertIn('imu_topic: "/tf_continuity_test/imu"', contents)

    def test_write_runtime_parameters_rejects_duplicate_or_missing_sensor_topic_leaf(self):
        cases = {
            "duplicate": """      lidar_topic: /first
      lidar_topic: /second
      imu_topic: /imu
""",
            "missing": """      lidar_topic: /lidar_points
""",
        }
        overrides = {
            "lidar_topic": "/tf_continuity_test/lidar_points",
            "imu_topic": "/tf_continuity_test/imu",
        }

        for name, sensor_topic in cases.items():
            with self.subTest(name=name):
                self._write_template(sensor_topic)
                with self.assertRaisesRegex(DatasetError, "sensor_topic.*exactly once"):
                    write_runtime_parameters(
                        self.template,
                        self.map_dir,
                        self.root / "runtime",
                        topic_overrides=overrides,
                    )


if __name__ == "__main__":
    unittest.main()
