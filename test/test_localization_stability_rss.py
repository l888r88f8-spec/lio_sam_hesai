#!/usr/bin/env python3
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from localization_rss import (
    RssError,
    _child_process_ids,
    _find_relocalization_pid,
    linear_slope_mib_per_hour,
    parse_vmrss_kib,
    summarize_samples,
)


class LocalizationStabilityRssTest(unittest.TestCase):
    def test_parse_vmrss_kib_returns_status_value(self):
        self.assertEqual(
            parse_vmrss_kib("Name:\tnode\nVmRSS:\t 12345 kB\n"), 12345
        )

    def test_parse_vmrss_kib_rejects_missing_value(self):
        with self.assertRaisesRegex(RuntimeError, "VmRSS"):
            parse_vmrss_kib("Name:\tnode\n")

    def test_linear_slope_mib_per_hour_uses_least_squares(self):
        self.assertAlmostEqual(
            linear_slope_mib_per_hour([(0.0, 100.0), (3600.0, 112.0)]),
            12.0,
        )

    def test_linear_slope_mib_per_hour_is_zero_for_flat_samples(self):
        self.assertAlmostEqual(
            linear_slope_mib_per_hour([(0.0, 100.0), (60.0, 100.0)]),
            0.0,
        )

    def test_summarize_samples_discards_samples_before_warmup(self):
        with self.assertRaisesRegex(RuntimeError, "at least two"):
            summarize_samples([(0.0, 100.0), (299.0, 101.0)], 300.0)

    def test_summarize_samples_rejects_one_post_warmup_sample(self):
        with self.assertRaisesRegex(RuntimeError, "at least two"):
            summarize_samples([(0.0, 100.0), (300.0, 101.0)], 300.0)

    def test_summarize_samples_reports_peak_and_slope_after_warmup(self):
        summary = summarize_samples(
            [(0.0, 80.0), (300.0, 100.0), (3900.0, 112.0)], 300.0
        )

        self.assertEqual(summary["peak_rss_mib"], 112.0)
        self.assertAlmostEqual(summary["slope_mib_per_hour"], 12.0)

    def test_find_relocalization_pid_falls_back_to_status_ppid(self):
        with TemporaryDirectory() as directory:
            proc_root = Path(directory)
            self._write_status(proc_root, 100, "PPid:\t1\n")
            self._write_status(proc_root, 200, "PPid:\t100\n")
            (proc_root / "200" / "cmdline").write_bytes(
                b"/opt/ros/lib/lio_sam_hesai/lio_sam_hesai_relocalization\0"
            )

            self.assertEqual(_find_relocalization_pid(100, proc_root), 200)

    def test_status_ppid_fallback_rejects_missing_ppid(self):
        with TemporaryDirectory() as directory:
            proc_root = Path(directory)
            self._write_status(proc_root, 100, "PPid:\t1\n")
            self._write_status(proc_root, 200, "Name:\ttarget\n")

            with self.assertRaisesRegex(RssError, "PPid"):
                _child_process_ids(100, proc_root)

    def test_status_ppid_fallback_rejects_malformed_ppid(self):
        with TemporaryDirectory() as directory:
            proc_root = Path(directory)
            self._write_status(proc_root, 100, "PPid:\t1\n")
            self._write_status(proc_root, 200, "PPid:\tnot-a-pid\n")

            with self.assertRaisesRegex(RssError, "PPid"):
                _child_process_ids(100, proc_root)

    @staticmethod
    def _write_status(proc_root, process_id, contents):
        status_path = proc_root / str(process_id) / "status"
        status_path.parent.mkdir(parents=True)
        status_path.write_text(contents, encoding="ascii")


if __name__ == "__main__":
    unittest.main()
