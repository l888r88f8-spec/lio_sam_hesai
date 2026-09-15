#!/usr/bin/env python3
import copy
import json
import math
import unittest
from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import Mock, call, patch

import localization_regression
from localization_regression import MetricCollector
from stability_baseline import BASELINE
from stability_common import (
    DatasetError,
    load_tiled_fault_reference,
    make_tiled_fault_reference,
    reference_base_to_lidar,
    save_tiled_fault_reference,
    tile_index_for_pose,
)


class MetricCollectorTest(unittest.TestCase):
    def test_diagnostic_level_decodes_single_byte_ros_levels(self):
        self.assertEqual(localization_regression._diagnostic_level(b"\x00"), 0)
        self.assertEqual(localization_regression._diagnostic_level(b"\x01"), 1)
        self.assertEqual(localization_regression._diagnostic_level(b"\x02"), 2)
        self.assertEqual(
            localization_regression._diagnostic_level(bytearray(b"\x02")), 2
        )

    def test_diagnostic_level_rejects_invalid_byte_representations(self):
        for value in (
            b"",
            b"\x00\x01",
            bytearray(),
            bytearray(b"\x00\x01"),
            memoryview(b"\x00"),
            "0",
            0.0,
            None,
            True,
        ):
            with self.subTest(value=value):
                self.assertIsNone(localization_regression._diagnostic_level(value))

    def test_collects_reference_clock_odom_and_ok_diagnostic(self):
        collector = MetricCollector()

        collector.clock(100.0)
        collector.odom(101.5, 0, 0, 0, 0, 0, 0, 1)
        collector.diagnostic(
            102.0,
            0,
            "lio_sam_hesai: localization",
            {
                "health_score": "0.8",
                "reg_fitness": "0.2",
                "covariance_scale": "1.0",
                "lidar_rate_hz": "10.0",
                "imu_rate_hz": "100.0",
                "timestamp_monotonic": "true",
            },
        )

        metrics = collector.finish(110.0)

        self.assertEqual(
            tuple(metrics),
            (
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
                "map_tile_events",
            ),
        )
        self.assertEqual(metrics["first_odom_seconds"], 1.5)
        self.assertEqual(metrics["first_ok_seconds"], 2.0)
        self.assertIs(metrics["timestamp_monotonic"], True)

    def test_byte_level_ok_diagnostic_populates_first_ok_and_scalars(self):
        collector = MetricCollector()

        collector.clock(100.0)
        collector.diagnostic(
            102.0,
            b"\x00",
            "lio_sam_hesai: localization",
            {
                "health_score": "0.8",
                "reg_fitness": "0.2",
                "covariance_scale": "1.0",
                "lidar_rate_hz": "10.0",
                "imu_rate_hz": "100.0",
            },
        )

        metrics = collector.finish(110.0)

        self.assertEqual(metrics["first_ok_seconds"], 2.0)
        self.assertEqual(metrics["min_health_score"], 0.8)
        self.assertEqual(metrics["max_reg_fitness"], 0.2)
        self.assertEqual(metrics["max_covariance_scale"], 1.0)
        self.assertEqual(metrics["min_lidar_rate_hz"], 10.0)
        self.assertEqual(metrics["min_imu_rate_hz"], 100.0)

    def test_invalid_quaternion_does_not_count_as_valid_odom(self):
        collector = MetricCollector()

        collector.clock(100.0)
        collector.odom(101.0, 0, 0, 0, 0, 0, 0, 1.01)

        metrics = collector.finish(102.0)

        self.assertEqual(metrics["valid_odom_ratio"], 0.0)
        self.assertIsNone(metrics["first_odom_seconds"])
        self.assertIsNone(metrics["max_odom_gap_seconds"])

    def test_nan_diagnostic_scalar_is_collected_as_invalid_without_raising(self):
        collector = MetricCollector()

        collector.clock(100.0)
        collector.diagnostic(
            101.0,
            0,
            "lio_sam_hesai: localization",
            {"health_score": "nan"},
        )

        metrics = collector.finish(102.0)

        self.assertTrue(math.isnan(metrics["min_health_score"]))

    def test_non_target_status_is_ignored(self):
        collector = MetricCollector()

        collector.clock(100.0)
        collector.diagnostic(
            101.0,
            2,
            "another node",
            {"health_score": "nan", "timestamp_monotonic": "false"},
        )

        metrics = collector.finish(102.0)

        self.assertEqual(metrics["error_samples"], 0)
        self.assertIsNone(metrics["min_health_score"])
        self.assertIsNone(metrics["timestamp_monotonic"])

    def test_no_ok_status_has_no_first_ok_time(self):
        collector = MetricCollector()

        collector.clock(100.0)
        collector.diagnostic(101.0, 1, "lio_sam_hesai: localization", {})

        metrics = collector.finish(104.0)

        self.assertIsNone(metrics["first_ok_seconds"])
        self.assertEqual(metrics["max_non_ok_gap_seconds"], 4.0)

    def test_no_diagnostic_samples_leave_non_ok_gap_unavailable(self):
        collector = MetricCollector()

        collector.clock(100.0)

        self.assertIsNone(collector.finish(104.0)["max_non_ok_gap_seconds"])

    def test_consecutive_ok_events_close_initial_non_ok_gap_once(self):
        self.assertEqual(
            localization_regression._max_non_ok_gap_seconds(
                [(101.0, 0), (102.0, 0), (103.0, 0)], 100.0, 104.0
            ),
            1.0,
        )

    def test_repeated_ok_events_do_not_start_a_new_non_ok_gap(self):
        self.assertEqual(
            localization_regression._max_non_ok_gap_seconds(
                [(101.0, 0), (102.0, 0)], 100.0, 104.0
            ),
            1.0,
        )

    def test_non_ok_gap_tracks_ok_warn_ok_transitions(self):
        self.assertEqual(
            localization_regression._max_non_ok_gap_seconds(
                [(101.0, 0), (102.0, 1), (104.0, 0), (105.0, 0)],
                100.0,
                106.0,
            ),
            2.0,
        )

    def test_error_level_two_is_counted(self):
        collector = MetricCollector()

        collector.clock(100.0)
        collector.diagnostic(101.0, 2, "lio_sam_hesai: localization", {})
        collector.diagnostic(102.0, 2, "lio_sam_hesai: localization", {})

        self.assertEqual(collector.finish(103.0)["error_samples"], 2)

    def test_first_odom_and_ok_offsets_are_order_independent_of_clock_callback(self):
        collector = MetricCollector()

        collector.odom(101.5, 0, 0, 0, 0, 0, 0, 1)
        collector.diagnostic(
            102.0,
            0,
            "lio_sam_hesai: localization",
            {"timestamp_monotonic": "true"},
        )
        collector.clock(100.0)

        metrics = collector.finish(110.0)

        self.assertEqual(metrics["first_odom_seconds"], 1.5)
        self.assertEqual(metrics["first_ok_seconds"], 2.0)
        self.assertEqual(metrics["max_non_ok_gap_seconds"], 2.0)

    def test_observer_uses_replay_clock_instead_of_epoch_message_headers(self):
        epoch_stamp = SimpleNamespace(sec=1781750040, nanosec=500000000)
        odometry = SimpleNamespace(
            header=SimpleNamespace(stamp=epoch_stamp),
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            ),
        )
        diagnostics = SimpleNamespace(
            header=SimpleNamespace(stamp=epoch_stamp),
            status=[
                SimpleNamespace(
                    level=0,
                    name="lio_sam_hesai: localization",
                    values=[SimpleNamespace(key="timestamp_monotonic", value="true")],
                )
            ],
        )
        collector = MetricCollector()

        collector.clock(64.0)
        localization_regression._record_odom_at_replay_clock(
            collector, 65.0, odometry
        )
        localization_regression._record_diagnostics_at_replay_clock(
            collector, 65.0, diagnostics
        )

        metrics = collector.finish(66.0)

        self.assertEqual(metrics["first_odom_seconds"], 1.0)
        self.assertEqual(metrics["first_ok_seconds"], 1.0)
        self.assertGreaterEqual(metrics["first_odom_seconds"], 0.0)

    def test_observer_drops_messages_until_replay_clock_is_available(self):
        odometry = SimpleNamespace(
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            )
        )
        collector = MetricCollector()

        localization_regression._record_odom_at_replay_clock(
            collector, None, odometry
        )

        self.assertIsNone(collector.finish(1.0)["valid_odom_ratio"])


class ObserverQosTest(unittest.TestCase):
    def test_clock_uses_sensor_qos_and_metrics_use_reliable_qos(self):
        sensor_qos = object()
        reliable_qos = object()

        qos_by_topic = localization_regression._observer_subscription_qos(
            sensor_qos, reliable_qos
        )

        self.assertEqual(
            tuple(qos_by_topic),
            (
                "/clock",
                "/lio_sam/localization/odometry",
                "/diagnostics",
            ),
        )
        self.assertIs(qos_by_topic["/clock"], sensor_qos)
        self.assertIs(
            qos_by_topic["/lio_sam/localization/odometry"], reliable_qos
        )
        self.assertIs(qos_by_topic["/diagnostics"], reliable_qos)


class LocalizationRegressionCliTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.output_dir = self.root / "results"
        self.identity = SimpleNamespace(
            duration_ns=756378139407,
            message_count=46875,
            topic_counts={"/imu": 42617, "/lidar_points": 4258},
            map_points=178455,
        )
        self.metrics = {
            "first_odom_seconds": 1.5,
            "valid_odom_ratio": 1.0,
            "max_odom_gap_seconds": 0.1,
            "first_ok_seconds": 2.0,
            "error_samples": 0,
            "max_non_ok_gap_seconds": 2.0,
            "min_health_score": 0.8,
            "max_reg_fitness": 0.2,
            "max_covariance_scale": 1.0,
            "min_lidar_rate_hz": 10.0,
            "min_imu_rate_hz": 100.0,
            "timestamp_monotonic": True,
        }

    def tearDown(self):
        self.temporary_directory.cleanup()

    def _arguments(self, *extra):
        return [
            "--bag",
            str(self.root / "bag"),
            "--map",
            str(self.root / "map"),
            "--params-template",
            str(self.root / "relocalization.yaml"),
            "--output-dir",
            str(self.output_dir),
            *extra,
        ]

    def test_normal_mode_writes_artifacts_and_returns_nonzero_for_comparator_violation(self):
        baseline = copy.deepcopy(BASELINE)
        runtime_parameters = self.output_dir / "relocalization.runtime.yaml"
        with (
            patch.object(localization_regression, "validate_dataset", return_value=self.identity),
            patch.object(
                localization_regression,
                "write_runtime_parameters",
                return_value=runtime_parameters,
            ),
            patch.object(localization_regression, "load_baseline", return_value=baseline),
            patch.object(
                localization_regression, "_run_replay", return_value=self.metrics
            ),
            patch.object(
                localization_regression,
                "compare_quick_metrics",
                return_value=["min_health_score too low"],
            ) as compare,
        ):
            with redirect_stderr(StringIO()):
                exit_code = localization_regression.main(
                    self._arguments("--baseline", str(self.root / "baseline.yaml"))
                )

        self.assertEqual(exit_code, 1)
        compare.assert_called_once_with(baseline, self.metrics)
        self.assertTrue((self.output_dir / "launch.log").is_file())
        self.assertTrue((self.output_dir / "bag-play.log").is_file())
        observed = json.loads((self.output_dir / "observed.json").read_text(encoding="utf-8"))
        self.assertEqual(observed["dataset"]["message_count"], 46875)
        self.assertEqual(observed["metrics"], self.metrics)

    def test_record_mode_writes_only_candidate_baseline_and_uses_default_quick_settings(self):
        runtime_parameters = self.output_dir / "relocalization.runtime.yaml"
        tracked_baseline = self.root / "hesai_sim.yaml"
        tracked_baseline.write_text("leave this untouched\n", encoding="utf-8")
        with (
            patch.object(localization_regression, "validate_dataset", return_value=self.identity),
            patch.object(
                localization_regression,
                "write_runtime_parameters",
                return_value=runtime_parameters,
            ),
            patch.object(
                localization_regression, "_run_replay", return_value=self.metrics
            ) as run_replay,
        ):
            with redirect_stdout(StringIO()):
                exit_code = localization_regression.main(
                    self._arguments("--record-baseline")
                )

        self.assertEqual(exit_code, 0)
        self.assertEqual(tracked_baseline.read_text(encoding="utf-8"), "leave this untouched\n")
        self.assertTrue((self.output_dir / "candidate-baseline.yaml").is_file())
        quick = run_replay.call_args.args[2]
        self.assertEqual(
            {key: quick[key] for key in ("start_offset_seconds", "duration_seconds", "rate", "startup_grace_seconds")},
            {"start_offset_seconds": 0.0, "duration_seconds": 180.0, "rate": 1.0, "startup_grace_seconds": 30.0},
        )

    def test_record_baseline_and_baseline_are_mutually_exclusive(self):
        parser = localization_regression.build_argument_parser()

        with redirect_stderr(StringIO()), self.assertRaises(SystemExit):
            parser.parse_args(self._arguments("--baseline", "baseline.yaml", "--record-baseline"))

    def test_cli_error_keeps_empty_artifacts_and_valid_null_json(self):
        with redirect_stderr(StringIO()):
            exit_code = localization_regression.main(self._arguments())

        self.assertEqual(exit_code, 2)
        self.assertTrue((self.output_dir / "launch.log").is_file())
        self.assertTrue((self.output_dir / "bag-play.log").is_file())
        observed = json.loads((self.output_dir / "observed.json").read_text(encoding="utf-8"))
        self.assertIsNone(observed["dataset"])
        self.assertIsNone(observed["metrics"])

    def test_observed_json_converts_nonfinite_metric_to_null(self):
        destination = self.root / "observed.json"

        localization_regression._write_observed(
            destination,
            self.identity,
            {"health_score": math.nan, "unavailable": None},
            None,
        )

        observed = json.loads(destination.read_text(encoding="utf-8"))
        self.assertIsNone(observed["metrics"]["health_score"])
        self.assertIsNone(observed["metrics"]["unavailable"])

    def test_ros_import_failure_returns_clear_cli_error_with_artifacts(self):
        runtime_parameters = self.output_dir / "relocalization.runtime.yaml"
        with (
            patch.object(localization_regression, "validate_dataset", return_value=self.identity),
            patch.object(
                localization_regression,
                "write_runtime_parameters",
                return_value=runtime_parameters,
            ),
            patch.object(
                localization_regression,
                "load_baseline",
                return_value=copy.deepcopy(BASELINE),
            ),
            patch.object(
                localization_regression,
                "_run_replay",
                side_effect=ImportError("no rclpy in this environment"),
            ),
            redirect_stderr(StringIO()),
        ):
            exit_code = localization_regression.main(
                self._arguments("--baseline", str(self.root / "baseline.yaml"))
            )

        self.assertEqual(exit_code, 2)
        observed = json.loads((self.output_dir / "observed.json").read_text(encoding="utf-8"))
        self.assertEqual(observed["error"], "no rclpy in this environment")

    def test_replay_timeout_returns_clear_cli_error_with_artifacts(self):
        runtime_parameters = self.output_dir / "relocalization.runtime.yaml"
        with (
            patch.object(localization_regression, "validate_dataset", return_value=self.identity),
            patch.object(
                localization_regression,
                "write_runtime_parameters",
                return_value=runtime_parameters,
            ),
            patch.object(
                localization_regression,
                "load_baseline",
                return_value=copy.deepcopy(BASELINE),
            ),
            patch.object(
                localization_regression,
                "_run_replay",
                side_effect=localization_regression.ReplayError(
                    "replay timed out waiting for /clock"
                ),
            ),
            redirect_stderr(StringIO()),
        ):
            exit_code = localization_regression.main(
                self._arguments("--baseline", str(self.root / "baseline.yaml"))
            )

        self.assertEqual(exit_code, 2)
        self.assertTrue((self.output_dir / "launch.log").is_file())
        self.assertTrue((self.output_dir / "bag-play.log").is_file())
        observed = json.loads((self.output_dir / "observed.json").read_text(encoding="utf-8"))
        self.assertEqual(observed["error"], "replay timed out waiting for /clock")

    def test_replay_clock_timeout_uses_injected_monotonic_without_sleeping(self):
        class FakeMonotonic:
            def __init__(self):
                self.seconds = 0.0

            def __call__(self):
                return self.seconds

            def advance(self, seconds):
                self.seconds += seconds

        monotonic = FakeMonotonic()
        launch_process = Mock()
        player_process = Mock()
        launch_process.poll.return_value = None
        player_process.poll.return_value = None

        def spin_once(_node, _timeout_seconds):
            monotonic.advance(31.0)

        with self.assertRaisesRegex(
            localization_regression.ReplayError, "timed out waiting for /clock"
        ):
            localization_regression._spin_bag_window(
                None,
                launch_process,
                player_process,
                [None],
                duration_seconds=1.0,
                rate=1.0,
                monotonic=monotonic,
                spin_once=spin_once,
            )

    def test_target_clock_drains_queued_diagnostic_before_metrics_finish(self):
        launch_process = Mock()
        player_process = Mock()
        latest_clock = [None]
        collector = MetricCollector()
        launch_process.poll.return_value = None
        player_process.poll.return_value = None

        def record_clock(seconds):
            latest_clock[0] = seconds
            collector.clock(seconds)

        callbacks = iter(
            (
                lambda: record_clock(100.0),
                lambda: record_clock(101.0),
                lambda: collector.diagnostic(
                    latest_clock[0], 0, "lio_sam_hesai: localization", {}
                ),
            )
        )

        def spin_once(_node, _timeout_seconds):
            try:
                next(callbacks)()
            except StopIteration:
                pass

        localization_regression._spin_bag_window(
            None,
            launch_process,
            player_process,
            latest_clock,
            duration_seconds=1.0,
            rate=1.0,
            monotonic=lambda: 0.0,
            spin_once=spin_once,
        )

        self.assertEqual(collector.finish(latest_clock[0])["first_ok_seconds"], 1.0)

    def test_cleanup_signals_process_group_when_leader_already_exited(self):
        process = Mock()
        process.pid = 1234
        process.poll.return_value = 0
        process.wait.return_value = 0

        with patch.object(localization_regression.os, "killpg") as killpg:
            localization_regression._stop_process_group(process)

        killpg.assert_called_once_with(1234, localization_regression.signal.SIGINT)
        process.wait.assert_called_once_with(timeout=10.0)

    def test_cleanup_escalates_to_sigkill_and_performs_final_wait(self):
        process = Mock()
        process.pid = 1234
        process.wait.side_effect = [
            localization_regression.subprocess.TimeoutExpired("ros2", 10.0),
            localization_regression.subprocess.TimeoutExpired("ros2", 10.0),
            0,
        ]

        with patch.object(localization_regression.os, "killpg") as killpg:
            localization_regression._stop_process_group(process)

        self.assertEqual(
            killpg.call_args_list,
            [
                call(1234, localization_regression.signal.SIGINT),
                call(1234, localization_regression.signal.SIGTERM),
                call(1234, localization_regression.signal.SIGKILL),
            ],
        )
        self.assertEqual(process.wait.call_count, 3)

    def test_cleanup_reports_unreaped_group_after_sigkill_final_wait(self):
        process = Mock()
        process.pid = 1234
        process.wait.side_effect = [
            localization_regression.subprocess.TimeoutExpired("ros2", 10.0),
            localization_regression.subprocess.TimeoutExpired("ros2", 10.0),
            localization_regression.subprocess.TimeoutExpired("ros2", 10.0),
        ]

        with patch.object(localization_regression.os, "killpg") as killpg, self.assertRaisesRegex(
            localization_regression.ReplayError, "process group 1234 could not be reaped"
        ):
            localization_regression._stop_process_group(process)

        self.assertEqual(
            killpg.call_args_list[-1],
            call(1234, localization_regression.signal.SIGKILL),
        )
        self.assertEqual(process.wait.call_count, 3)

    def test_successful_player_exit_accepts_a_drained_final_clock_callback(self):
        launch_process = Mock()
        player_process = Mock()
        latest_clock = [None]
        launch_process.poll.return_value = None
        player_process.poll.side_effect = [None, 0]
        clock_values = iter((100.0, 101.0))
        spin_calls = 0

        def spin_once(_node, _timeout_seconds):
            nonlocal spin_calls
            spin_calls += 1
            try:
                latest_clock[0] = next(clock_values)
            except StopIteration:
                pass

        localization_regression._spin_bag_window(
            None,
            launch_process,
            player_process,
            latest_clock,
            duration_seconds=1.0,
            rate=1.0,
            monotonic=lambda: 0.0,
            spin_once=spin_once,
        )

        self.assertEqual(latest_clock[0], 101.0)
        self.assertEqual(
            spin_calls,
            1 + localization_regression._FINAL_CALLBACK_DRAIN_MAX_CALLBACKS,
        )

    def test_successful_player_exit_after_short_drained_clock_is_premature(self):
        launch_process = Mock()
        player_process = Mock()
        latest_clock = [None]
        launch_process.poll.return_value = None
        player_process.poll.side_effect = [None, 0]
        clock_values = iter((100.0, 100.5))
        spin_calls = 0

        def spin_once(_node, _timeout_seconds):
            nonlocal spin_calls
            spin_calls += 1
            try:
                latest_clock[0] = next(clock_values)
            except StopIteration:
                pass

        with self.assertRaisesRegex(
            localization_regression.ReplayError,
            "before replay duration after draining queued /clock",
        ):
            localization_regression._spin_bag_window(
                None,
                launch_process,
                player_process,
                latest_clock,
                duration_seconds=1.0,
                rate=1.0,
                monotonic=lambda: 0.0,
                spin_once=spin_once,
            )

        self.assertEqual(
            spin_calls,
            1 + localization_regression._FINAL_CALLBACK_DRAIN_MAX_CALLBACKS,
        )

    def test_successful_player_exit_drains_multiple_queued_clock_callbacks(self):
        launch_process = Mock()
        player_process = Mock()
        latest_clock = [None]
        launch_process.poll.return_value = None
        player_process.poll.side_effect = [None, 0]
        clock_values = iter((100.0, 100.5, 101.0))
        spin_calls = 0

        def spin_once(_node, _timeout_seconds):
            nonlocal spin_calls
            spin_calls += 1
            try:
                latest_clock[0] = next(clock_values)
            except StopIteration:
                pass

        localization_regression._spin_bag_window(
            None,
            launch_process,
            player_process,
            latest_clock,
            duration_seconds=1.0,
            rate=1.0,
            monotonic=lambda: 0.0,
            spin_once=spin_once,
        )

        self.assertEqual(latest_clock[0], 101.0)
        self.assertEqual(
            spin_calls,
            1 + localization_regression._FINAL_CALLBACK_DRAIN_MAX_CALLBACKS,
        )


class TiledFaultReferenceTest(unittest.TestCase):
    def _identity(self, **overrides):
        base = {
            "duration_ns": 100, "message_count": 10, "topic_counts": {},
            "db3_bytes": 1000, "map_points": 50, "map_bytes": 500,
        }
        base.update(overrides)
        return SimpleNamespace(**base)

    def test_rotated_lever_arm_moves_the_lidar_into_another_tile(self):
        # Base at (-5, -5) yawed 90 degrees, lidar 9.9 m ahead in base axes.
        # Rotating the lever arm puts the lidar at (-5, 4.9) -> tile (-1, 0).
        # Adding it in map axes would put it at (4.9, -5) -> tile (0, -1).
        map_T_base = [
            [0.0, -1.0, 0.0, -5.0],
            [1.0, 0.0, 0.0, -5.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        base_T_lidar = reference_base_to_lidar((9.9, 0.0, 0.32))

        self.assertEqual(
            tile_index_for_pose(map_T_base, base_T_lidar, (0.0, 0.0), 10.0),
            (-1, 0),
        )
        # A coarser grid still separates the rotated result (-5, 4.9) from the
        # unrotated one (4.9, -5): (-1, 0) against (0, -1).
        self.assertEqual(
            tile_index_for_pose(map_T_base, base_T_lidar, (0.0, 0.0), 100.0),
            (-1, 0),
        )

    def test_negative_coordinates_floor_towards_negative_infinity(self):
        base_T_lidar = reference_base_to_lidar((0.0, 0.0, 0.0))
        for x, expected_column in ((-0.001, -1), (-10.0, -1), (-10.001, -2),
                                   (0.0, 0), (9.999, 0), (10.0, 1)):
            pose = [[1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
            with self.subTest(x=x):
                self.assertEqual(
                    tile_index_for_pose(pose, base_T_lidar, (0.0, 0.0), 10.0),
                    (expected_column, 0),
                )

    def test_origin_offsets_the_tile_grid(self):
        base_T_lidar = reference_base_to_lidar((0.0, 0.0, 0.0))
        pose = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
        self.assertEqual(
            tile_index_for_pose(pose, base_T_lidar, (0.0, 0.0), 10.0), (0, 0))
        self.assertEqual(
            tile_index_for_pose(pose, base_T_lidar, (-5.0, 5.0), 10.0), (0, -1))

    def test_rejects_a_malformed_pose(self):
        base_T_lidar = reference_base_to_lidar((0.0, 0.0, 0.0))
        with self.assertRaises(DatasetError):
            tile_index_for_pose([[1.0, 0.0, 0.0]], base_T_lidar, (0.0, 0.0), 10.0)
        with self.assertRaises(DatasetError):
            tile_index_for_pose([[1.0] * 4] * 4, base_T_lidar, (0.0, 0.0), 0.0)

    def test_reference_round_trips_and_records_the_offset(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "reference.json"
            identity = self._identity()
            reference = make_tiled_fault_reference(
                identity, bag_seconds=42.5,
                map_T_base=[[1.0, 0.0, 0.0, 1.0], [0.0, 1.0, 0.0, 2.0],
                            [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
                warmup_seconds=10.0)
            self.assertEqual(reference.offset_seconds, 32.5)
            save_tiled_fault_reference(path, reference)

            loaded = load_tiled_fault_reference(path, identity)
            self.assertEqual(loaded.bag_seconds, 42.5)
            self.assertEqual(loaded.offset_seconds, 32.5)
            self.assertEqual(loaded.map_T_base[0][3], 1.0)

    def test_reference_refuses_a_different_dataset(self):
        # A pose measured in one map has no meaning in another, and a reused
        # bag would silently point the fault at the wrong tile.
        with TemporaryDirectory() as directory:
            path = Path(directory) / "reference.json"
            save_tiled_fault_reference(
                path,
                make_tiled_fault_reference(
                    self._identity(), bag_seconds=1.0,
                    map_T_base=[[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]))
            for overrides in ({"map_bytes": 999}, {"message_count": 11},
                              {"db3_bytes": 1}):
                with self.subTest(**overrides):
                    with self.assertRaises(DatasetError):
                        load_tiled_fault_reference(
                            path, self._identity(**overrides))

    def test_reference_rejects_an_unknown_schema(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "reference.json"
            path.write_text(json.dumps({"schema_version": 99}), encoding="utf-8")
            with self.assertRaises(DatasetError):
                load_tiled_fault_reference(path, self._identity())


class TiledFaultRunTest(unittest.TestCase):
    class _Fixture:
        def __init__(self, fail=False):
            self.injections = 0
            self.fail = fail

        def inject(self):
            if self.fail:
                raise DatasetError("injection failed")
            self.injections += 1

    def _reference(self, bag_seconds=20.0):
        return SimpleNamespace(bag_seconds=bag_seconds, offset_seconds=10.0)

    def _status(self, values, name=None):
        return SimpleNamespace(status=[
            SimpleNamespace(
                name=(name if name is not None
                      else localization_regression._TARGET_DIAGNOSTIC),
                values=[SimpleNamespace(key=k, value=v)
                        for k, v in values.items()])])

    def test_other_nodes_cannot_open_the_gates(self):
        # A different localization-relevant node reporting the same key names
        # must not satisfy the prepare or IMU gate.
        run, fixture, published, publish = self._run()
        run.observe_diagnostics(self._status(
            {"map_tile_load_state": "ready", "map_tile_load_failures": "0"},
            name="lio_sam_hesai: runtime"))
        self.assertEqual(run.tick(0.0, publish), "waiting-for-prepare")
        self.assertEqual(fixture.injections, 0)

        run.observe_diagnostics(self._status(
            {"has_imu_init": "true"}, name="some_other_node"))
        run.observe_diagnostics(self._status(
            {"map_tile_load_state": "ready", "map_tile_load_failures": "0"}))
        run.tick(0.0, publish)
        with self.assertRaises(localization_regression.ReplayError):
            run.tick(10.0, publish)

    def _run(self, **kwargs):
        fixture = self._Fixture(**kwargs.pop("fixture", {}))
        run = localization_regression.TiledFaultRun(
            fixture, self._reference(**kwargs.pop("reference", {})))
        published = []
        return run, fixture, published, lambda: published.append(1)

    def test_does_not_inject_before_the_map_is_prepared(self):
        run, fixture, published, publish = self._run()
        run.observe_diagnostics(self._status({"map_tile_load_state": "loading"}))
        self.assertEqual(run.tick(0.0, publish), "waiting-for-prepare")
        self.assertEqual(fixture.injections, 0)
        self.assertEqual(published, [])

    def test_zero_load_failures_is_part_of_the_prepare_gate(self):
        run, fixture, _published, publish = self._run()
        run.observe_diagnostics(self._status({
            "map_tile_load_state": "ready", "map_tile_load_failures": "1"}))
        run.tick(0.0, publish)
        self.assertEqual(fixture.injections, 0)

    def test_injects_once_then_waits_for_the_reference_time(self):
        run, fixture, published, publish = self._run()
        run.observe_diagnostics(self._status({
            "map_tile_load_state": "ready", "map_tile_load_failures": "0"}))
        self.assertEqual(run.tick(0.0, publish), "injected")
        self.assertEqual(fixture.injections, 1)
        self.assertEqual(run.tick(5.0, publish), "warming-up")
        self.assertEqual(fixture.injections, 1)
        self.assertEqual(published, [])

    def test_publishes_at_the_reference_time_once_the_imu_is_ready(self):
        # Ticks carry elapsed replay time; the pose is due at
        # bag_seconds - offset_seconds, which is the warm-up window.
        run, fixture, published, publish = self._run()
        self.assertEqual(run.reference_offset, 10.0)
        run.observe_diagnostics(self._status({
            "map_tile_load_state": "ready", "map_tile_load_failures": "0"}))
        run.tick(0.0, publish)
        run.observe_diagnostics(self._status({"has_imu_init": "true"}))
        self.assertEqual(run.tick(10.0, publish), "published")
        self.assertEqual(len(published), 1)
        self.assertEqual(run.tick(11.0, publish), "waiting-for-reference")
        self.assertEqual(len(published), 1)

    def test_refuses_to_publish_without_imu_initialization(self):
        run, _fixture, published, publish = self._run()
        run.observe_diagnostics(self._status({
            "map_tile_load_state": "ready", "map_tile_load_failures": "0"}))
        run.tick(0.0, publish)
        with self.assertRaises(localization_regression.ReplayError):
            run.tick(10.0, publish)
        self.assertEqual(published, [])

    def test_refuses_a_reference_time_that_was_missed(self):
        # Continuing with a stale pose would assert against the wrong tile.
        run, _fixture, published, publish = self._run()
        run.observe_diagnostics(self._status({
            "map_tile_load_state": "ready", "map_tile_load_failures": "0"}))
        run.tick(0.0, publish)
        run.observe_diagnostics(self._status({"has_imu_init": "true"}))
        with self.assertRaises(localization_regression.ReplayError):
            run.tick(30.0, publish)
        self.assertEqual(published, [])

    def test_injection_failure_propagates(self):
        run, fixture, _published, publish = self._run(fixture={"fail": True})
        run.observe_diagnostics(self._status({
            "map_tile_load_state": "ready", "map_tile_load_failures": "0"}))
        with self.assertRaises(DatasetError):
            run.tick(0.0, publish)
        self.assertFalse(run.injected)


if __name__ == "__main__":
    unittest.main()
