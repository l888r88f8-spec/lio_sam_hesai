import io
import json
import math
import tempfile
import unittest
from pathlib import Path

from scripts import measure_localization_jitter as jitter
from scripts.measure_localization_jitter import summarize_positions


class PositionJitterSummaryTest(unittest.TestCase):
    def test_constant_pose_has_zero_jitter_and_drift(self):
        samples = [(float(t), 1.2, -0.5, 0.3) for t in range(5)]

        summary = summarize_positions(samples)

        self.assertEqual(summary["sample_count"], 5)
        self.assertEqual(summary["duration_s"], 4.0)
        for axis in ("x", "y", "z"):
            self.assertAlmostEqual(summary["axis"][axis]["peak_to_peak_mm"], 0.0)
            self.assertAlmostEqual(summary["axis"][axis]["std_mm"], 0.0)
        self.assertAlmostEqual(summary["xy_rms_mm"], 0.0)
        self.assertAlmostEqual(summary["xy_detrended_rms_mm"], 0.0)
        self.assertAlmostEqual(summary["xy_drift_mm_s"], 0.0)

    def test_linear_drift_is_not_counted_as_detrended_jitter(self):
        samples = [(float(t), 0.01 * t, 0.02 * t, 0.0) for t in range(5)]

        summary = summarize_positions(samples)

        self.assertAlmostEqual(summary["axis"]["x"]["peak_to_peak_mm"], 40.0)
        self.assertAlmostEqual(summary["axis"]["y"]["peak_to_peak_mm"], 80.0)
        self.assertAlmostEqual(summary["xy_drift_mm_s"], math.hypot(10.0, 20.0))
        self.assertAlmostEqual(summary["xy_detrended_rms_mm"], 0.0, places=9)

    def test_back_and_forth_motion_remains_after_detrending(self):
        positions_mm = (1.0, 0.0, -1.0, 0.0, 1.0)
        samples = [(float(t), x_mm / 1000.0, 0.0, 0.0)
                   for t, x_mm in enumerate(positions_mm)]

        summary = summarize_positions(samples)

        self.assertAlmostEqual(summary["axis"]["x"]["peak_to_peak_mm"], 2.0)
        self.assertAlmostEqual(summary["xy_drift_mm_s"], 0.0)
        self.assertGreater(summary["xy_detrended_rms_mm"], 0.0)

    def test_too_few_or_nonmonotonic_samples_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "at least 3"):
            summarize_positions([(0.0, 0.0, 0.0, 0.0)])
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            summarize_positions([(0.0, 0.0, 0.0, 0.0),
                                 (1.0, 0.0, 0.0, 0.0),
                                 (1.0, 0.0, 0.0, 0.0)])


class _Stub:
    def __init__(self, **fields):
        for name, value in fields.items():
            setattr(self, name, value)


def _header(sec=0, nanosec=0, frame_id="map"):
    return _Stub(stamp=_Stub(sec=sec, nanosec=nanosec), frame_id=frame_id)


def _odometry(x=1.0, y=2.0, z=3.0, sec=0, nanosec=0):
    return _Stub(
        header=_header(sec, nanosec),
        child_frame_id="base_link",
        pose=_Stub(pose=_Stub(
            position=_Stub(x=x, y=y, z=z),
            orientation=_Stub(x=0.0, y=0.0, z=0.0, w=1.0),
        )),
    )


def _imu(sec=0, nanosec=0):
    return _Stub(
        header=_header(sec, nanosec, "imu_link"),
        angular_velocity=_Stub(x=0.01, y=-0.02, z=0.03),
        linear_acceleration=_Stub(x=0.1, y=0.2, z=9.81),
    )


def _key_value(key, value):
    return _Stub(key=key, value=value)


def _status(name=None, level=0, message="ok", values=()):
    if name is None:
        name = jitter.LOCALIZATION_DIAGNOSTIC_NAME
    return _Stub(name=name, level=level, message=message, values=list(values))


def _diagnostic_array(*statuses, sec=0, nanosec=0):
    return _Stub(header=_header(sec, nanosec), status=list(statuses))


class ValueParsingTest(unittest.TestCase):
    def test_numeric_and_boolean_text_parsing(self):
        self.assertEqual(jitter.parse_number("1.25"), 1.25)
        self.assertIsNone(jitter.parse_number("abc"))
        self.assertIsNone(jitter.parse_number(None))
        self.assertIs(jitter.parse_bool("true"), True)
        self.assertIs(jitter.parse_bool("FALSE"), False)
        self.assertIsNone(jitter.parse_bool("maybe"))
        self.assertIsNone(jitter.finite_or_none(float("nan")))
        self.assertIsNone(jitter.finite_or_none(None))
        self.assertEqual(jitter.finite_or_none(3), 3.0)


class OdometryEventTest(unittest.TestCase):
    def test_event_preserves_full_pose_and_relative_receive_time(self):
        message = _odometry(x=1.5, y=-2.25, z=0.75, sec=12, nanosec=500000000)

        event = jitter.build_odometry_event(message, 100.25, 100.0)

        self.assertEqual(event["type"], "odometry")
        self.assertAlmostEqual(event["t_s"], 0.25)
        self.assertAlmostEqual(event["stamp_s"], 12.5)
        self.assertEqual(event["frame_id"], "map")
        self.assertEqual(event["child_frame_id"], "base_link")
        self.assertEqual(event["position"], {"x": 1.5, "y": -2.25, "z": 0.75})
        self.assertEqual(
            event["orientation"], {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})

    def test_missing_header_stamp_is_recorded_as_null(self):
        message = _odometry()
        message.header = _Stub(frame_id="map")

        event = jitter.build_odometry_event(message, 1.0, 1.0)

        self.assertIsNone(event["stamp_s"])


class ImuEventTest(unittest.TestCase):
    def test_event_preserves_angular_velocity_and_linear_acceleration(self):
        message = _imu(sec=3, nanosec=250000000)

        event = jitter.build_imu_event(message, 2.0, 1.5)

        self.assertEqual(event["type"], "imu")
        self.assertAlmostEqual(event["t_s"], 0.5)
        self.assertAlmostEqual(event["stamp_s"], 3.25)
        self.assertEqual(event["frame_id"], "imu_link")
        self.assertEqual(
            event["angular_velocity"], {"x": 0.01, "y": -0.02, "z": 0.03})
        self.assertEqual(
            event["linear_acceleration"], {"x": 0.1, "y": 0.2, "z": 9.81})


class LocalizationDiagnosticsEventTest(unittest.TestCase):
    def test_ros_byte_level_is_json_serializable_and_named_warn(self):
        array = _diagnostic_array(_status(level=b"\x01"))

        event = jitter.build_localization_diagnostics_event(array, 5.0, 5.0)

        self.assertEqual(event["level"], 1)
        self.assertEqual(event["level_name"], "WARN")
        self.assertEqual(json.loads(jitter.encode_event(event))["level"], 1)

    def test_selects_localization_status_and_extracts_requested_values(self):
        runtime = _status(name="lio_sam_hesai: runtime", message="runtime")
        localization = _status(
            level=1, message="fitness degraded",
            values=[
                _key_value("reg_fitness", "0.3125"),
                _key_value("is_tracking", "true"),
                _key_value("tf_mode", "extrapolated"),
                _key_value("health_score", "0.875"),
                _key_value("last_result_age_us", "4200"),
                _key_value("s_stream", "1.0"),
            ])
        array = _diagnostic_array(runtime, localization, sec=7,
                                  nanosec=250000000)

        self.assertIs(jitter.select_localization_status(array), localization)
        event = jitter.build_localization_diagnostics_event(array, 11.0, 10.0)

        self.assertEqual(event["type"], "diagnostics")
        self.assertAlmostEqual(event["t_s"], 1.0)
        self.assertAlmostEqual(event["stamp_s"], 7.25)
        self.assertEqual(event["level"], 1)
        self.assertEqual(event["level_name"], "WARN")
        self.assertEqual(event["message"], "fitness degraded")
        self.assertAlmostEqual(event["values"]["reg_fitness"], 0.3125)
        self.assertIs(event["values"]["is_tracking"], True)
        self.assertEqual(event["values"]["tf_mode"], "extrapolated")
        self.assertAlmostEqual(event["values"]["health_score"], 0.875)
        self.assertAlmostEqual(event["values"]["last_result_age_us"], 4200.0)

    def test_missing_values_are_null_and_absent_status_is_none(self):
        array = _diagnostic_array(_status(values=[_key_value("is_tracking", "maybe")]))

        event = jitter.build_localization_diagnostics_event(array, 5.0, 5.0)

        self.assertEqual(event["level_name"], "OK")
        self.assertEqual(
            event["values"],
            {"reg_fitness": None, "is_tracking": None, "tf_mode": None,
             "health_score": None, "last_result_age_us": None})

        other = _diagnostic_array(_status(name="lio_sam_hesai: runtime"))
        self.assertIsNone(
            jitter.build_localization_diagnostics_event(other, 5.0, 5.0))
        self.assertIsNone(jitter.select_localization_status(other))


class AdjacentXyJumpTest(unittest.TestCase):
    def test_reports_largest_adjacent_jumps_in_descending_order(self):
        samples = [
            (0.0, 0.000, 0.000, 0.0),
            (1.0, 0.001, 0.000, 0.0),
            (2.0, 0.001, 0.003, 0.0),
            (3.0, 0.005, 0.003, 0.0),
            (4.0, 0.005, 0.013, 0.0),
            (5.0, 0.006, 0.013, 0.0),
        ]

        jumps = jitter.top_xy_jumps(samples, count=3)

        self.assertEqual(len(jumps), 3)
        self.assertAlmostEqual(jumps[0]["t_s"], 4.0)
        self.assertAlmostEqual(jumps[0]["dx_mm"], 0.0)
        self.assertAlmostEqual(jumps[0]["dy_mm"], 10.0)
        self.assertAlmostEqual(jumps[0]["norm_mm"], 10.0)
        self.assertAlmostEqual(jumps[1]["t_s"], 3.0)
        self.assertAlmostEqual(jumps[1]["norm_mm"], 4.0)
        self.assertAlmostEqual(jumps[2]["t_s"], 2.0)
        self.assertAlmostEqual(jumps[2]["norm_mm"], 3.0)

    def test_fewer_than_two_samples_yields_no_jumps(self):
        self.assertEqual(jitter.top_xy_jumps([]), [])
        self.assertEqual(jitter.top_xy_jumps([(0.0, 0.0, 0.0, 0.0)]), [])


class StreamSummaryTest(unittest.TestCase):
    def test_diagnostics_summary_counts_levels_fitness_tracking_and_modes(self):
        events = [
            {"level_name": "OK",
             "values": {"reg_fitness": 0.1, "is_tracking": True,
                        "tf_mode": "measured"}},
            {"level_name": "WARN",
             "values": {"reg_fitness": 0.3, "is_tracking": False,
                        "tf_mode": "extrapolated"}},
            {"level_name": "WARN",
             "values": {"reg_fitness": None, "is_tracking": None,
                        "tf_mode": None}},
        ]

        summary = jitter.summarize_diagnostics(events)

        self.assertEqual(summary["sample_count"], 3)
        self.assertEqual(summary["level_counts"], {"OK": 1, "WARN": 2})
        self.assertEqual(summary["fitness_count"], 2)
        self.assertAlmostEqual(summary["fitness_min"], 0.1)
        self.assertAlmostEqual(summary["fitness_mean"], 0.2)
        self.assertAlmostEqual(summary["fitness_max"], 0.3)
        self.assertEqual(summary["is_tracking_counts"],
                         {"true": 1, "false": 1, "unknown": 1})
        self.assertEqual(summary["tf_mode_counts"],
                         {"measured": 1, "extrapolated": 1, "unknown": 1})

    def test_empty_summaries_are_zeroed(self):
        diagnostics = jitter.summarize_diagnostics([])
        self.assertEqual(diagnostics["sample_count"], 0)
        self.assertEqual(diagnostics["level_counts"], {})
        self.assertIsNone(diagnostics["fitness_min"])
        self.assertIsNone(diagnostics["fitness_mean"])

        imu = jitter.summarize_imu([])
        self.assertEqual(imu["sample_count"], 0)
        self.assertEqual(imu["duration_s"], 0.0)
        self.assertIsNone(imu["rate_hz"])

    def test_imu_summary_reports_count_duration_and_rate(self):
        events = [{"t_s": value} for value in (0.0, 0.01, 0.02, 0.03)]

        summary = jitter.summarize_imu(events)

        self.assertEqual(summary["sample_count"], 4)
        self.assertAlmostEqual(summary["duration_s"], 0.03)
        self.assertAlmostEqual(summary["rate_hz"], 100.0)


class OutputPathTest(unittest.TestCase):
    def test_default_output_path_is_timestamped_jsonl(self):
        path = jitter.default_output_path(epoch_seconds=0.0)

        self.assertRegex(path.name, r"^localization_jitter_\d{8}_\d{6}\.jsonl$")

    def test_resolve_output_path_keeps_free_path(self):
        with tempfile.TemporaryDirectory() as directory:
            candidate = Path(directory) / "run.jsonl"

            self.assertEqual(jitter.resolve_output_path(candidate), candidate)

    def test_resolve_output_path_avoids_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            existing = Path(directory) / "run.jsonl"
            existing.write_text("", encoding="utf-8")

            resolved = jitter.resolve_output_path(existing)

            self.assertNotEqual(resolved, existing)
            self.assertFalse(resolved.exists())

            resolved.write_text("", encoding="utf-8")
            self.assertEqual(jitter.resolve_output_path(existing).name,
                             "run-2.jsonl")


class JsonlEncodingTest(unittest.TestCase):
    def test_events_are_written_as_one_json_object_per_line(self):
        stream = io.StringIO()

        jitter.write_event(stream, {"type": "imu", "t_s": 0.5})
        jitter.write_event(stream, {"type": "odometry", "t_s": 0.6})

        lines = stream.getvalue().splitlines()
        self.assertEqual(len(lines), 2)
        self.assertEqual(json.loads(lines[0])["type"], "imu")
        self.assertEqual(json.loads(lines[1])["type"], "odometry")

    def test_non_finite_values_are_not_encoded_as_json(self):
        with self.assertRaises(ValueError):
            jitter.encode_event({"type": "imu", "value": float("nan")})


class OutputStreamTest(unittest.TestCase):
    def test_creates_stream_at_unused_path_without_touching_existing_file(self):
        with tempfile.TemporaryDirectory() as directory:
            existing = Path(directory) / "run.jsonl"
            existing.write_text("old\n", encoding="utf-8")

            path, stream = jitter.create_output_stream(existing)
            try:
                self.assertNotEqual(path, existing)
                self.assertEqual(existing.read_text(encoding="utf-8"), "old\n")
                stream.write("new\n")
            finally:
                stream.close()

            self.assertFalse(existing.read_text(encoding="utf-8") == "new\n")
            self.assertEqual(path.read_text(encoding="utf-8"), "new\n")

    def test_unwritable_output_path_raises_oserror(self):
        with tempfile.TemporaryDirectory() as directory:
            blocker = Path(directory) / "blocker"
            blocker.write_text("", encoding="utf-8")

            with self.assertRaises(OSError):
                jitter.create_output_stream(blocker / "run.jsonl")


class MetaEventTest(unittest.TestCase):
    def test_meta_event_records_topics_window_and_path(self):
        event = jitter.build_meta_event(
            topics={"odometry": "/odom", "diagnostics": "/diag", "imu": "/imu"},
            duration_s=30.0,
            output_path="/tmp/run.jsonl",
            recorded_at="2026-09-17T15:00:00+08:00",
        )

        self.assertEqual(event["type"], "meta")
        self.assertEqual(event["topics"]["imu"], "/imu")
        self.assertAlmostEqual(event["duration_s"], 30.0)
        self.assertEqual(event["output_path"], "/tmp/run.jsonl")
        self.assertEqual(event["recorded_at"], "2026-09-17T15:00:00+08:00")
        self.assertEqual(event["time_origin"], jitter.TIME_ORIGIN_DESCRIPTION)


class ConsoleReportTest(unittest.TestCase):
    def test_jump_lines_report_time_and_components(self):
        lines = jitter.format_xy_jump_lines([
            {"t_s": 1.5, "dx_mm": 3.0, "dy_mm": -4.0, "norm_mm": 5.0}])

        self.assertTrue(any(
            "t=1.500 s" in line and "dx=3.00 mm" in line
            and "dy=-4.00 mm" in line and "|dxy|=5.00 mm" in line
            for line in lines))

    def test_jump_lines_state_when_no_jump_is_available(self):
        lines = jitter.format_xy_jump_lines([])

        self.assertTrue(lines)
        self.assertIn("跳变", "\n".join(lines))

    def test_diagnostics_lines_report_fitness_and_counts(self):
        summary = jitter.summarize_diagnostics([
            {"level_name": "WARN",
             "values": {"reg_fitness": 0.2, "is_tracking": True,
                        "tf_mode": "held"}},
        ])

        text = "\n".join(jitter.format_diagnostics_lines(summary))

        self.assertIn("reg_fitness", text)
        self.assertIn("WARN", text)
        self.assertIn("held", text)

    def test_imu_lines_report_sample_count(self):
        text = "\n".join(jitter.format_imu_lines(
            jitter.summarize_imu([{"t_s": 0.0}, {"t_s": 0.1}])))

        self.assertIn("IMU 样本: 2", text)
        self.assertIn("10.0 Hz", text)


class CommandLineTest(unittest.TestCase):
    def test_parser_preserves_original_options_and_adds_stream_topics(self):
        parser = jitter.build_parser()

        defaults = parser.parse_args([])

        self.assertEqual(defaults.topic, jitter.DEFAULT_TOPIC)
        self.assertEqual(defaults.duration, 30.0)
        self.assertEqual(defaults.diagnostics_topic,
                         jitter.DEFAULT_DIAGNOSTICS_TOPIC)
        self.assertEqual(defaults.imu_topic, jitter.DEFAULT_IMU_TOPIC)
        self.assertIsNone(defaults.output)

        overridden = parser.parse_args([
            "--topic", "/odom",
            "--duration", "5",
            "--diagnostics-topic", "/diag",
            "--imu-topic", "/imu0",
            "--output", "/tmp/run.jsonl",
        ])
        self.assertEqual(overridden.topic, "/odom")
        self.assertEqual(overridden.duration, 5.0)
        self.assertEqual(overridden.diagnostics_topic, "/diag")
        self.assertEqual(overridden.imu_topic, "/imu0")
        self.assertEqual(overridden.output, "/tmp/run.jsonl")

    def test_empty_odometry_topic_is_rejected(self):
        with self.assertRaises(SystemExit):
            jitter.main(["--topic", "   "])

    def test_non_positive_duration_is_rejected(self):
        with self.assertRaises(SystemExit):
            jitter.main(["--duration", "0"])


if __name__ == "__main__":
    unittest.main()
