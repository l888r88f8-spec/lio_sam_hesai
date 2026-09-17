import math
import unittest

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


if __name__ == "__main__":
    unittest.main()
