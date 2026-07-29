#!/usr/bin/env python3
import unittest

from localization_rss import (
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


if __name__ == "__main__":
    unittest.main()
