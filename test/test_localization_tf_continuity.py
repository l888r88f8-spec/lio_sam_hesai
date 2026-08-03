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

from localization_tf_continuity import FaultSchedule, TfContinuityCollector, main
from stability_common import DatasetError, write_runtime_parameters


class FaultScheduleTest(unittest.TestCase):
    def test_schedule_keeps_imu_during_lidar_only_outage(self):
        schedule = FaultSchedule(lidar_outage=(30.0, 40.0), all_outage=(50.0, 60.0))

        self.assertFalse(schedule.forward_lidar(35.0))
        self.assertTrue(schedule.forward_imu(35.0))
        self.assertFalse(schedule.forward_lidar(55.0))
        self.assertFalse(schedule.forward_imu(55.0))


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

    def _record_contract(self, collector, diagnostic_order=None, tf_mode=None):
        for timestamp in self._TF_SAMPLES:
            collector.tf(timestamp, mode=tf_mode)
        if diagnostic_order is None:
            diagnostic_order = range(len(self._DIAGNOSTICS))
        for index in diagnostic_order:
            timestamp, values = self._DIAGNOSTICS[index]
            collector.diagnostic(timestamp, values)

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

    def test_collector_limits_stream_recovery_auto_trigger_to_one(self):
        collector = TfContinuityCollector(20.0)
        self._record_contract(collector)
        collector.diagnostic(61.0, {"auto_relocalization_trigger": "stream_recovered"})
        collector.diagnostic(61.5, {"auto_relocalization_trigger": "none"})
        collector.diagnostic(62.0, {"auto_relocalization_trigger": "stream_recovered"})

        self.assertTrue(
            any("stream_recovered" in error for error in collector.assert_contracts())
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
