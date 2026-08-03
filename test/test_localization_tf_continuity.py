#!/usr/bin/env python3
"""Pure contracts for the localization TF-continuity replay harness."""

import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory


INTEGRATION_DIRECTORY = Path(__file__).parent / "integration"
sys.path.insert(0, str(INTEGRATION_DIRECTORY))

from localization_tf_continuity import FaultSchedule, TfContinuityCollector
from stability_common import DatasetError, write_runtime_parameters


class FaultScheduleTest(unittest.TestCase):
    def test_schedule_keeps_imu_during_lidar_only_outage(self):
        schedule = FaultSchedule(lidar_outage=(30.0, 40.0), all_outage=(50.0, 60.0))

        self.assertFalse(schedule.forward_lidar(35.0))
        self.assertTrue(schedule.forward_imu(35.0))
        self.assertFalse(schedule.forward_lidar(55.0))
        self.assertFalse(schedule.forward_imu(55.0))


class TfContinuityCollectorTest(unittest.TestCase):
    def test_collector_requires_each_fallback_mode_and_recovery(self):
        collector = TfContinuityCollector(20.0)
        collector.tf(29.0, "measured")
        collector.tf(30.1, "extrapolated")
        collector.tf(31.0, "held")
        collector.tf(55.0, "held", "imu_unavailable")
        collector.tf(65.0, "measured")

        self.assertEqual(collector.assert_contracts(), [])

    def test_collector_reports_tf_timestamp_regression(self):
        collector = TfContinuityCollector(20.0)
        collector.tf(29.0, "measured")
        collector.tf(28.9, "measured")

        self.assertTrue(
            any("TF timestamp regressed" in error for error in collector.assert_contracts())
        )

    def test_collector_reports_too_large_active_tf_spacing(self):
        collector = TfContinuityCollector(0.10)
        collector.tf(29.0, "measured")
        collector.tf(29.11, "measured")

        self.assertTrue(
            any("active-mode spacing" in error for error in collector.assert_contracts())
        )

    def test_collector_limits_stream_recovery_auto_trigger_to_one(self):
        collector = TfContinuityCollector(20.0)
        collector.tf(29.0, "measured")
        collector.tf(30.1, "extrapolated")
        collector.tf(31.0, "held")
        collector.tf(55.0, "held", "imu_unavailable")
        collector.tf(65.0, "measured")
        collector.diagnostic(61.0, {"auto_relocalization_trigger": "stream_recovered"})
        collector.diagnostic(61.5, {"auto_relocalization_trigger": "none"})
        collector.diagnostic(62.0, {"auto_relocalization_trigger": "stream_recovered"})

        self.assertTrue(
            any("stream_recovered" in error for error in collector.assert_contracts())
        )


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
