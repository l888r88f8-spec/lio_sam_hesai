#!/usr/bin/env python3
import copy
import math
from types import MappingProxyType
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import yaml

from stability_baseline import (
    BASELINE,
    BaselineError,
    compare_quick_metrics,
    load_baseline,
    write_candidate,
)
from stability_common import DatasetIdentity


class LocalizationStabilityBaselineTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.fixture_path = self.root / "baseline.yaml"
        self.baseline = copy.deepcopy(BASELINE)
        self._write_fixture(self.baseline)

    def tearDown(self):
        self.temporary_directory.cleanup()

    def _write_fixture(self, contents):
        self.fixture_path.write_text(
            yaml.safe_dump(contents, sort_keys=False), encoding="utf-8"
        )

    def _write_raw_fixture(self, contents):
        self.fixture_path.write_text(contents, encoding="utf-8")

    def test_load_baseline_returns_an_independent_validated_copy(self):
        loaded = load_baseline(self.fixture_path)
        loaded["quick"]["rate"] = 2.0

        reloaded = load_baseline(self.fixture_path)

        self.assertEqual(reloaded, BASELINE)
        self.assertIsNot(loaded, reloaded)
        self.assertIsNot(loaded["quick"], reloaded["quick"])

    def test_load_baseline_rejects_unsupported_schema_version(self):
        baseline = copy.deepcopy(self.baseline)
        baseline["schema_version"] = 2
        self._write_fixture(baseline)

        with self.assertRaisesRegex(BaselineError, "schema_version.*1"):
            load_baseline(self.fixture_path)

    def test_load_baseline_rejects_each_missing_required_key(self):
        required_paths = (
            "schema_version",
            "dataset",
            "dataset.duration_ns",
            "dataset.message_count",
            "dataset.topic_counts",
            "dataset.topic_counts./imu",
            "dataset.topic_counts./lidar_points",
            "dataset.map_points",
            "quick.start_offset_seconds",
            "quick.duration_seconds",
            "quick.rate",
            "quick.startup_grace_seconds",
            "quick.first_odom_max_seconds",
            "quick.min_valid_odom_ratio",
            "quick.max_odom_gap_seconds",
            "quick.ok_deadline_seconds",
            "quick.max_error_samples",
            "quick.max_non_ok_gap_seconds",
            "quick.health_score_min",
            "quick.reg_fitness_max",
            "quick.covariance_scale_max",
            "quick.lidar_rate_min_hz",
            "quick.imu_rate_min_hz",
            "memory",
            "memory.warmup_seconds",
            "memory.duration_seconds",
            "memory.sample_period_seconds",
            "memory.max_rss_mib",
            "memory.max_slope_mib_per_hour",
        )

        for required_path in required_paths:
            with self.subTest(required_path=required_path):
                baseline = copy.deepcopy(self.baseline)
                target = baseline
                keys = required_path.split(".")
                for key in keys[:-1]:
                    target = target[key]
                del target[keys[-1]]
                self._write_fixture(baseline)

                with self.assertRaisesRegex(BaselineError, required_path):
                    load_baseline(self.fixture_path)

    def test_load_baseline_rejects_boolean_numeric_bounds(self):
        baseline = copy.deepcopy(self.baseline)
        baseline["quick"]["first_odom_max_seconds"] = True
        self._write_fixture(baseline)

        with self.assertRaisesRegex(BaselineError, "quick.first_odom_max_seconds"):
            load_baseline(self.fixture_path)

    def test_load_baseline_rejects_duplicate_yaml_keys(self):
        fixture = yaml.safe_dump(self.baseline, sort_keys=False)
        duplicate_fixtures = {
            "schema version": fixture + "schema_version: 1\n",
            "quick threshold": fixture.replace(
                "  first_odom_max_seconds: 60.0\n",
                "  first_odom_max_seconds: 60.0\n"
                "  first_odom_max_seconds: 61.0\n",
            ),
        }

        for name, duplicate_fixture in duplicate_fixtures.items():
            with self.subTest(name=name):
                self._write_raw_fixture(duplicate_fixture)

                with self.assertRaisesRegex(BaselineError, "duplicate key"):
                    load_baseline(self.fixture_path)

    def test_compare_quick_metrics_accepts_exact_boundaries(self):
        observed = {
            "first_odom_seconds": 60.0,
            "valid_odom_ratio": 0.95,
            "max_odom_gap_seconds": 1.0,
            "first_ok_seconds": 90.0,
            "error_samples": 0,
            "max_non_ok_gap_seconds": 5.0,
            "min_health_score": 0.5,
            "max_reg_fitness": 1.0,
            "max_covariance_scale": 100.0,
            "min_lidar_rate_hz": 5.0,
            "min_imu_rate_hz": 50.0,
            "timestamp_monotonic": True,
        }

        self.assertEqual(compare_quick_metrics(self.baseline, observed), [])

    def test_compare_quick_metrics_reports_every_violation(self):
        observed = {
            "first_odom_seconds": 61.0,
            "valid_odom_ratio": 0.90,
            "max_odom_gap_seconds": 2.0,
            "first_ok_seconds": None,
            "error_samples": 1,
            "max_non_ok_gap_seconds": 6.0,
            "min_health_score": 0.2,
            "max_reg_fitness": 1.2,
            "max_covariance_scale": 101.0,
            "min_lidar_rate_hz": 4.0,
            "min_imu_rate_hz": 40.0,
            "timestamp_monotonic": False,
        }

        violations = compare_quick_metrics(self.baseline, observed)

        expected_fields = (
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
        self.assertEqual(len(violations), len(expected_fields))
        for field in expected_fields:
            self.assertTrue(
                any(field in violation for violation in violations),
                msg=f"No violation reported for {field}: {violations}",
            )

    def test_compare_quick_metrics_rejects_invalid_scalar_domains(self):
        observed = {
            "first_odom_seconds": -1.0,
            "valid_odom_ratio": 2.0,
            "max_odom_gap_seconds": -1.0,
            "first_ok_seconds": 90.0,
            "error_samples": -1,
            "max_non_ok_gap_seconds": 5.0,
            "min_health_score": 2.0,
            "max_reg_fitness": 1.0,
            "max_covariance_scale": 100.0,
            "min_lidar_rate_hz": 5.0,
            "min_imu_rate_hz": 50.0,
            "timestamp_monotonic": True,
        }

        violations = compare_quick_metrics(self.baseline, observed)

        expected_fields = (
            "first_odom_seconds",
            "valid_odom_ratio",
            "max_odom_gap_seconds",
            "error_samples",
            "min_health_score",
        )
        self.assertEqual(len(violations), len(expected_fields))
        for field in expected_fields:
            self.assertTrue(
                any(field in violation for violation in violations),
                msg=f"No domain violation reported for {field}: {violations}",
            )

    def test_compare_quick_metrics_treats_nonfinite_observations_as_violations(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                observed = {
                    "first_odom_seconds": value,
                    "valid_odom_ratio": value,
                    "max_odom_gap_seconds": value,
                    "first_ok_seconds": value,
                    "error_samples": value,
                    "max_non_ok_gap_seconds": value,
                    "min_health_score": value,
                    "max_reg_fitness": value,
                    "max_covariance_scale": value,
                    "min_lidar_rate_hz": value,
                    "min_imu_rate_hz": value,
                    "timestamp_monotonic": True,
                }

                violations = compare_quick_metrics(self.baseline, observed)

                self.assertEqual(len(violations), 11)
                self.assertTrue(
                    all("finite number" in violation for violation in violations)
                )

    def test_compare_quick_metrics_never_mutates_baseline(self):
        baseline_before = copy.deepcopy(self.baseline)

        compare_quick_metrics(
            self.baseline,
            {"timestamp_monotonic": True},
        )

        self.assertEqual(self.baseline, baseline_before)

    def test_load_baseline_rejects_huge_integer_bounds(self):
        fixture = yaml.safe_dump(self.baseline, sort_keys=False)
        self._write_raw_fixture(
            fixture.replace(
                "  first_odom_max_seconds: 60.0\n",
                "  first_odom_max_seconds: " + "1" + "0" * 10000 + "\n",
            )
        )

        with self.assertRaisesRegex(BaselineError, "first_odom_max_seconds"):
            load_baseline(self.fixture_path)

    def test_compare_quick_metrics_reports_huge_integer_observations(self):
        violations = compare_quick_metrics(
            self.baseline,
            {
                "first_odom_seconds": 10**10000,
                "error_samples": 10**10000,
                "timestamp_monotonic": True,
            },
        )

        self.assertTrue(any("first_odom_seconds" in violation for violation in violations))
        self.assertTrue(any("error_samples" in violation for violation in violations))

    def test_write_candidate_rejects_huge_integer_before_writing(self):
        candidate_path = self.root / "candidate.yaml"

        with self.assertRaisesRegex(BaselineError, "first_odom_seconds"):
            write_candidate(
                candidate_path,
                self.baseline["dataset"],
                {"first_odom_seconds": 10**10000},
            )

        self.assertFalse(candidate_path.exists())

    def test_write_candidate_uses_identity_and_conservative_observed_bounds(self):
        candidate_path = self.root / "candidate" / "baseline.yaml"
        identity = {
            "duration_ns": 756378139407,
            "message_count": 46875,
            "topic_counts": {"/imu": 42617, "/lidar_points": 4258},
            "map_points": 178455,
        }
        observed = {
            "first_odom_seconds": 8.0,
            "valid_odom_ratio": 0.99,
            "max_odom_gap_seconds": 0.4,
            "first_ok_seconds": 12.0,
            "error_samples": 3,
            "max_non_ok_gap_seconds": 0.8,
            "min_health_score": 0.8,
            "max_reg_fitness": 0.5,
            "max_covariance_scale": 40.0,
            "min_lidar_rate_hz": 10.0,
            "min_imu_rate_hz": 100.0,
        }

        self.assertFalse(candidate_path.exists())
        written_path = write_candidate(candidate_path, identity, observed)
        candidate = yaml.safe_load(candidate_path.read_text(encoding="utf-8"))

        self.assertEqual(written_path, candidate_path)
        self.assertEqual(candidate["schema_version"], 1)
        self.assertEqual(candidate["dataset"], identity)
        expected_quick = {
            "start_offset_seconds": 0.0,
            "duration_seconds": 180.0,
            "rate": 1.0,
            "startup_grace_seconds": 30.0,
            "first_odom_max_seconds": 10.0,
            "min_valid_odom_ratio": 0.891,
            "max_odom_gap_seconds": 0.5,
            "ok_deadline_seconds": 13.2,
            "max_error_samples": 3,
            "max_non_ok_gap_seconds": 0.8800000000000001,
            "health_score_min": 0.7200000000000001,
            "reg_fitness_max": 0.55,
            "covariance_scale_max": 44.0,
            "lidar_rate_min_hz": 9.0,
            "imu_rate_min_hz": 90.0,
        }
        self.assertEqual(
            {key: value for key, value in candidate["quick"].items() if key != "ok_deadline_seconds"},
            {key: value for key, value in expected_quick.items() if key != "ok_deadline_seconds"},
        )
        self.assertAlmostEqual(
            candidate["quick"]["ok_deadline_seconds"],
            expected_quick["ok_deadline_seconds"],
        )
        self.assertEqual(candidate["memory"], BASELINE["memory"])

    def test_write_candidate_accepts_read_only_dataset_identity(self):
        candidate_path = self.root / "candidate.yaml"
        identity = DatasetIdentity(
            duration_ns=756378139407,
            message_count=46875,
            topic_counts=MappingProxyType({"/imu": 42617, "/lidar_points": 4258}),
            db3_bytes=1580380160,
            map_points=178455,
            map_artifacts=MappingProxyType(
                {"GlobalMap.pcd": "a" * 64, "GroundMap.pcd": "b" * 64,
                 "sc_db.bin": "c" * 64}),
        )
        observed = {
            "first_odom_seconds": 8.0,
            "valid_odom_ratio": 0.99,
            "max_odom_gap_seconds": 0.4,
            "first_ok_seconds": 12.0,
            "error_samples": 3,
            "max_non_ok_gap_seconds": 0.8,
            "min_health_score": 0.8,
            "max_reg_fitness": 0.5,
            "max_covariance_scale": 40.0,
            "min_lidar_rate_hz": 10.0,
            "min_imu_rate_hz": 100.0,
        }

        write_candidate(candidate_path, identity, observed)
        candidate = yaml.safe_load(candidate_path.read_text(encoding="utf-8"))
        candidate["dataset"]["topic_counts"]["/imu"] = 1

        self.assertEqual(identity.topic_counts["/imu"], 42617)
        self.assertEqual(candidate["dataset"]["topic_counts"]["/lidar_points"], 4258)


if __name__ == "__main__":
    unittest.main()
