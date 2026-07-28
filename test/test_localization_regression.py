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


if __name__ == "__main__":
    unittest.main()
