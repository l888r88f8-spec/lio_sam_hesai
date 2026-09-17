#!/usr/bin/env python3
"""Measure position jitter from a stationary localization odometry stream."""

import argparse
import math
import sys
import time


DEFAULT_TOPIC = "/lio_sam/localization/odometry"
WAIT_FOR_FIRST_SAMPLE_SECONDS = 10.0


def summarize_positions(samples):
    """Summarize (monotonic_seconds, x_m, y_m, z_m) position samples."""
    if len(samples) < 3:
        raise ValueError("at least 3 valid samples are required")
    if not all(math.isfinite(value) for sample in samples for value in sample):
        raise ValueError("all sample values must be finite")
    if any(right[0] <= left[0] for left, right in zip(samples, samples[1:])):
        raise ValueError("sample times must be strictly increasing")

    times = [sample[0] - samples[0][0] for sample in samples]
    coordinates = {
        axis: [sample[index] for sample in samples]
        for index, axis in enumerate(("x", "y", "z"), start=1)
    }
    count = len(samples)
    axis_summary = {}
    means = {}
    for axis, values in coordinates.items():
        mean = math.fsum(values) / count
        means[axis] = mean
        axis_summary[axis] = {
            "peak_to_peak_mm": (max(values) - min(values)) * 1000.0,
            "std_mm": math.sqrt(
                math.fsum((value - mean) ** 2 for value in values) / count
            ) * 1000.0,
        }

    mean_time = math.fsum(times) / count
    centered_times = [value - mean_time for value in times]
    time_variance_sum = math.fsum(value * value for value in centered_times)
    slopes = {
        axis: math.fsum(
            centered_time * (value - means[axis])
            for centered_time, value in zip(centered_times, coordinates[axis])
        ) / time_variance_sum
        for axis in ("x", "y")
    }
    xy_rms = math.sqrt(math.fsum(
        (x - means["x"]) ** 2 + (y - means["y"]) ** 2
        for x, y in zip(coordinates["x"], coordinates["y"])
    ) / count)
    xy_detrended_rms = math.sqrt(math.fsum(
        (x - means["x"] - slopes["x"] * centered_time) ** 2
        + (y - means["y"] - slopes["y"] * centered_time) ** 2
        for centered_time, x, y in zip(
            centered_times, coordinates["x"], coordinates["y"]
        )
    ) / count)

    return {
        "sample_count": count,
        "duration_s": times[-1],
        "axis": axis_summary,
        "xy_rms_mm": xy_rms * 1000.0,
        "xy_detrended_rms_mm": xy_detrended_rms * 1000.0,
        "xy_drift_mm_s": math.hypot(slopes["x"], slopes["y"]) * 1000.0,
    }


def positive_finite_seconds(value):
    try:
        seconds = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("duration must be a number") from error
    if not math.isfinite(seconds) or seconds <= 0.0:
        raise argparse.ArgumentTypeError("duration must be positive and finite")
    return seconds


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="Odometry topic")
    parser.add_argument(
        "--duration", type=positive_finite_seconds, default=30.0,
        help="stationary sampling window in seconds (default: 30)",
    )
    args = parser.parse_args(argv)
    if not args.topic.strip():
        parser.error("topic must not be empty")

    try:
        import rclpy
        from nav_msgs.msg import Odometry
    except ImportError as error:
        print(f"ROS 2 Python dependencies unavailable: {error}", file=sys.stderr)
        return 2

    samples = []
    invalid_count = 0

    def on_odometry(message):
        nonlocal invalid_count
        position = message.pose.pose.position
        values = (position.x, position.y, position.z)
        if not all(math.isfinite(value) for value in values):
            invalid_count += 1
            return
        samples.append((time.monotonic(), *values))

    rclpy.init()
    node = rclpy.create_node("measure_localization_jitter")
    node.create_subscription(Odometry, args.topic, on_odometry, 10)
    wait_started = time.monotonic()
    print(f"等待 {args.topic}；收到首条有效数据后采集 {args.duration:g} 秒。")
    try:
        while rclpy.ok():
            now = time.monotonic()
            if samples and now - samples[0][0] >= args.duration:
                break
            if not samples and now - wait_started >= WAIT_FOR_FIRST_SAMPLE_SECONDS:
                print("错误：10 秒内未收到有效 odometry 数据。", file=sys.stderr)
                return 2
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("采集被中断。", file=sys.stderr)
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if len(samples) < 3 or samples[-1][0] - samples[0][0] < args.duration * 0.8:
        print("错误：有效样本不足，或未覆盖采样窗口的 80%。", file=sys.stderr)
        return 2

    summary = summarize_positions(samples)
    print(f"有效样本: {summary['sample_count']}，无效样本: {invalid_count}，"
          f"实际覆盖: {summary['duration_s']:.2f} s")
    for axis in ("x", "y", "z"):
        metrics = summary["axis"][axis]
        print(f"{axis}: 峰峰值 {metrics['peak_to_peak_mm']:.2f} mm，"
              f"标准差 {metrics['std_mm']:.2f} mm")
    print(f"xy 总波动 RMS: {summary['xy_rms_mm']:.2f} mm")
    print(f"xy 去漂移抖动 RMS: {summary['xy_detrended_rms_mm']:.2f} mm")
    print(f"xy 水平漂移速度: {summary['xy_drift_mm_s']:.2f} mm/s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
