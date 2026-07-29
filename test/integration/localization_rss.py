#!/usr/bin/env python3
"""Measure relocalization RSS during a bounded looped rosbag replay."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from stability_baseline import BaselineError, load_baseline
from stability_common import (
    DatasetError,
    terminate_process_group,
    validate_dataset,
    write_runtime_parameters,
)


_MIB_IN_KIB = 1024.0
_PROCESS_STARTUP_TIMEOUT_SECONDS = 30.0
_PROCESS_STOP_TIMEOUT_SECONDS = 10.0


class RssError(RuntimeError):
    pass


def parse_vmrss_kib(status_text):
    """Return the VmRSS field from one Linux /proc/<pid>/status file."""
    for line in status_text.splitlines():
        fields = line.split()
        if len(fields) == 3 and fields[0] == "VmRSS:" and fields[2] == "kB":
            try:
                value = int(fields[1])
            except ValueError as error:
                raise RuntimeError("VmRSS is not an integer number of kB") from error
            if value < 0:
                raise RuntimeError("VmRSS must not be negative")
            return value
    raise RuntimeError("VmRSS field is missing from process status")


def linear_slope_mib_per_hour(samples):
    """Estimate RSS growth with a least-squares fit over monotonic seconds."""
    if len(samples) < 2:
        raise RuntimeError("at least two RSS samples are required")
    points = []
    for elapsed_seconds, rss_mib in samples:
        elapsed = _finite_number(elapsed_seconds)
        rss = _finite_number(rss_mib)
        if elapsed is None or rss is None:
            raise RuntimeError("RSS samples must contain finite numbers")
        points.append((elapsed, rss))

    count = float(len(points))
    mean_elapsed = sum(point[0] for point in points) / count
    mean_rss = sum(point[1] for point in points) / count
    denominator = sum((elapsed - mean_elapsed) ** 2 for elapsed, _ in points)
    if denominator == 0.0:
        raise RuntimeError("RSS samples must have distinct elapsed times")
    numerator = sum(
        (elapsed - mean_elapsed) * (rss - mean_rss) for elapsed, rss in points
    )
    return (numerator / denominator) * 3600.0


def summarize_samples(samples, warmup_seconds):
    """Summarize only samples collected at or after the warmup boundary."""
    warmup = _finite_number(warmup_seconds)
    if warmup is None or warmup < 0.0:
        raise RuntimeError("warmup_seconds must be a non-negative finite number")
    post_warmup = [sample for sample in samples if sample[0] >= warmup]
    if len(post_warmup) < 2:
        raise RuntimeError("at least two RSS samples are required after warmup")
    return {
        "peak_rss_mib": max(rss_mib for _, rss_mib in post_warmup),
        "slope_mib_per_hour": linear_slope_mib_per_hour(post_warmup),
    }


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="Sample relocalization RSS during a looped rosbag replay."
    )
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--map", required=True, type=Path)
    parser.add_argument("--params-template", required=True, type=Path)
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    return parser


def main(argv=None):
    arguments = build_argument_parser().parse_args(argv)
    output_dir = arguments.output_dir
    samples = []
    memory = None
    summary = None
    exit_reason = "completed"
    exit_code = 0

    try:
        output_dir.mkdir(parents=True, exist_ok=True)
        identity = validate_dataset(arguments.bag, arguments.map)
        del identity
        baseline = load_baseline(arguments.baseline)
        memory = baseline["memory"]
        runtime_parameters = write_runtime_parameters(
            arguments.params_template, arguments.map, output_dir
        )
        samples = _run_rss_collection(arguments, runtime_parameters, memory, output_dir)
        summary = summarize_samples(samples, memory["warmup_seconds"])
        violations = _memory_violations(summary, memory)
        if violations:
            exit_code = 1
            exit_reason = "limit_exceeded"
            for observed_key, observed, allowed in violations:
                print(
                    f"memory limit violation: {observed_key} observed={observed:.6f} "
                    f"allowed={allowed:.6f}",
                    file=sys.stderr,
                )
    except KeyboardInterrupt:
        exit_code = 130
        exit_reason = "interrupted"
        print("localization RSS collection interrupted", file=sys.stderr)
    except (BaselineError, DatasetError, OSError, RssError, RuntimeError) as error:
        exit_code = 2
        exit_reason = f"error: {error}"
        print(f"localization RSS collection error: {error}", file=sys.stderr)
    finally:
        summary = _summary_document(summary, samples, memory, exit_reason)
        try:
            _write_results(output_dir, samples, summary)
        except OSError as error:
            print(
                f"localization RSS collection error: unable to write results: {error}",
                file=sys.stderr,
            )
            exit_code = 2
    return exit_code


def _run_rss_collection(arguments, runtime_parameters, memory, output_dir):
    domain_id = str(200 + (os.getpid() % 30))
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = domain_id
    launch_command = [
        "ros2",
        "launch",
        "lio_sam_hesai",
        "localization_with_nonground.launch.py",
        "use_sim:=true",
        "use_rviz:=false",
        "publish_static_tf:=true",
        f"params_file:={runtime_parameters}",
    ]
    player_command = [
        "ros2",
        "bag",
        "play",
        str(arguments.bag),
        "--loop",
        "--clock",
        "100",
        "--disable-keyboard-controls",
    ]
    launch_process = None
    player_process = None
    launch_log = None
    player_log = None
    try:
        launch_log = (output_dir / "launch.log").open("w", encoding="utf-8")
        player_log = (output_dir / "bag-play.log").open("w", encoding="utf-8")
        launch_process = subprocess.Popen(
            launch_command,
            stdout=launch_log,
            stderr=subprocess.STDOUT,
            env=environment,
            start_new_session=True,
        )
        player_process = subprocess.Popen(
            player_command,
            stdout=player_log,
            stderr=subprocess.STDOUT,
            env=environment,
            start_new_session=True,
        )
        relocalization_pid = _wait_for_relocalization_pid(launch_process)
        return _collect_rss_samples(
            relocalization_pid,
            launch_process,
            player_process,
            float(memory["duration_seconds"]),
            float(memory["sample_period_seconds"]),
        )
    finally:
        cleanup_error = None
        for process in (player_process, launch_process):
            try:
                _stop_process_group(process)
            except RssError as error:
                if cleanup_error is None:
                    cleanup_error = error
        if launch_log is not None:
            launch_log.close()
        if player_log is not None:
            player_log.close()
        if cleanup_error is not None and sys.exc_info()[0] is None:
            raise cleanup_error


def _wait_for_relocalization_pid(launch_process):
    deadline = time.monotonic() + _PROCESS_STARTUP_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        if launch_process.poll() is not None:
            raise RssError(
                "launch process exited before relocalization started with code "
                f"{launch_process.returncode}"
            )
        relocalization_pid = _find_relocalization_pid(launch_process.pid)
        if relocalization_pid is not None:
            return relocalization_pid
        time.sleep(0.1)
    raise RssError(
        "timed out waiting for lio_sam_hesai_relocalization below the launch process"
    )


def _find_relocalization_pid(launch_pid):
    for process_id in _descendant_process_ids(launch_pid):
        if _is_relocalization_process(process_id):
            return process_id
    return None


def _descendant_process_ids(root_pid):
    pending = [int(root_pid)]
    seen = set()
    descendants = []
    while pending:
        process_id = pending.pop()
        if process_id in seen:
            continue
        seen.add(process_id)
        for child_pid in _child_process_ids(process_id):
            if child_pid not in seen:
                descendants.append(child_pid)
                pending.append(child_pid)
    return descendants


def _child_process_ids(process_id):
    children_path = Path(
        f"/proc/{process_id}/task/{process_id}/children"
    )
    try:
        children_text = children_path.read_text(encoding="ascii")
    except FileNotFoundError:
        return ()
    except OSError as error:
        raise RssError(f"unable to read {children_path}: {error}") from error
    try:
        return tuple(int(value) for value in children_text.split())
    except ValueError as error:
        raise RssError(f"invalid child PID data in {children_path}") from error


def _is_relocalization_process(process_id):
    cmdline_path = Path(f"/proc/{process_id}/cmdline")
    try:
        command_line = cmdline_path.read_bytes()
    except FileNotFoundError:
        return False
    except OSError as error:
        raise RssError(f"unable to read {cmdline_path}: {error}") from error
    return b"lio_sam_hesai_relocalization" in command_line


def _collect_rss_samples(
    relocalization_pid,
    launch_process,
    player_process,
    duration_seconds,
    sample_period_seconds,
):
    start = time.monotonic()
    deadline = start + duration_seconds
    next_sample = start
    samples = []
    while True:
        now = time.monotonic()
        if now >= deadline:
            return samples
        _require_running(launch_process, "launch")
        _require_running(player_process, "bag player")
        rss_kib = _read_process_rss_kib(relocalization_pid)
        samples.append((now - start, rss_kib / _MIB_IN_KIB))
        next_sample += sample_period_seconds
        time.sleep(max(0.0, min(next_sample, deadline) - time.monotonic()))


def _read_process_rss_kib(process_id):
    status_path = Path(f"/proc/{process_id}/status")
    try:
        status_text = status_path.read_text(encoding="utf-8")
    except FileNotFoundError as error:
        raise RssError(
            f"relocalization process {process_id} exited during RSS collection"
        ) from error
    except OSError as error:
        raise RssError(f"unable to read {status_path}: {error}") from error
    return parse_vmrss_kib(status_text)


def _require_running(process, label):
    if process.poll() is not None:
        raise RssError(f"{label} process exited during RSS collection with code {process.returncode}")


def _stop_process_group(process):
    if process is None:
        return
    terminate_process_group(process.pid)
    try:
        process.wait(timeout=_PROCESS_STOP_TIMEOUT_SECONDS)
        return
    except subprocess.TimeoutExpired:
        pass
    for signal_number, signal_name in ((signal.SIGTERM, "SIGTERM"), (signal.SIGKILL, "SIGKILL")):
        try:
            os.killpg(process.pid, signal_number)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=_PROCESS_STOP_TIMEOUT_SECONDS)
            return
        except subprocess.TimeoutExpired:
            continue
    raise RssError(f"process group {process.pid} could not be reaped after SIGKILL")


def _memory_violations(summary, memory):
    limits = (
        ("peak_rss_mib", memory["max_rss_mib"]),
        ("slope_mib_per_hour", memory["max_slope_mib_per_hour"]),
    )
    return [
        (name, summary[name], limit)
        for name, limit in limits
        if summary[name] > limit
    ]


def _summary_document(summary, samples, memory, exit_reason):
    memory = memory or {}
    summary = summary or {}
    return {
        "peak_rss_mib": summary.get("peak_rss_mib"),
        "slope_mib_per_hour": summary.get("slope_mib_per_hour"),
        "sample_count": len(samples),
        "warmup_seconds": memory.get("warmup_seconds"),
        "duration_seconds": memory.get("duration_seconds"),
        "exit_reason": exit_reason,
    }


def _write_results(output_dir, samples, summary):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "rss.csv").open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(("elapsed_seconds", "rss_mib"))
        writer.writerows(samples)
    with (output_dir / "rss-summary.json").open("w", encoding="utf-8") as summary_file:
        json.dump(summary, summary_file, allow_nan=False, indent=2, sort_keys=True)
        summary_file.write("\n")


def _finite_number(value):
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


if __name__ == "__main__":
    sys.exit(main())
