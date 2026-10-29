#!/usr/bin/env python3
"""Run the actual native DUFOMap core on a deterministic occluded moving scene."""
import argparse
import json
import platform
import sys
from pathlib import Path

import numpy as np
import dufomap as backend


def scene():
    # Dense lidar returns are needed for DUFOMap's unknown-space erosion (d_p).
    wall = np.array([[10.05, y / 20 + .05, z / 20 + .05]
                     for y in range(-60, 61) for z in range(-40, 41)], dtype=np.float32)
    pose = np.eye(4, dtype=np.float64)
    pose[:3, 3] = .05
    frames = []
    moving = []
    for y in (.35, 1.35, -1.35, 1.35):
        point = np.array([5.05, y, .35], dtype=np.float32)
        # Hide wall returns behind the foreground point's angular footprint.
        ratio = (point[0] - .05) / (wall[:, 0] - .05)
        intersection = .05 + (wall - .05) * ratio[:, None]
        hidden = ((np.abs(intersection[:, 1] - y) < .15) &
                  (np.abs(intersection[:, 2] - .35) < .15))
        world = np.concatenate((wall[~hidden], point[None, :]))
        local = (world - pose[:3, 3]).astype(np.float32)
        frames.append(local)
        moving.append(point)
    return wall, pose, frames, np.array(moving, dtype=np.float32)


def run(output_dir=None):
    wall, pose, frames, moving = scene()
    identity = np.eye(4)
    result = {"python": sys.executable, "machine": platform.machine(),
              "backend": backend.__backend__, "ufomap_revision": backend.__ufomap_revision__,
              "numpy": np.__version__, "parameters": {"resolution": .1, "d_s": .2},
              "backend_kind": backend.__backend_kind__, "frames": len(frames),
              "points_per_frame": [len(frame) for frame in frames]}
    all_reports = []
    for threads, dp in ((2, 1), (0, 1), (0, 2)):
        model = backend.dufomap(.1, .2, dp, num_threads=threads)
        model.run(frames[0], pose, cloud_transform=True)
        model.oncePropagateCluster(if_propagate=True, if_cluster=False)
        before = model.segment(moving[:1], identity, cloud_transform=False)
        for frame in frames[1:]:
            model.run(frame, pose, cloud_transform=True)
        model.oncePropagateCluster(if_propagate=True, if_cluster=False)
        labels = model.segment(moving, identity, cloud_transform=False)
        wall_labels = model.segment(wall, identity, cloud_transform=False)
        occluded = np.array([[12.05, .35, .35], [12.05, -.35, -.35]], dtype=np.float32)
        occluded_labels = model.segment(occluded, identity, cloud_transform=False)
        # Also catch the upstream serial TODO path returning all-static results.
        free_probe = np.array([[5.05, .35, .35], [5.05, .55, .35]], dtype=np.float32)
        free_labels = model.segment(free_probe, identity, cloud_transform=False)
        if before.tolist() != [0] or labels[0] != 1 or not np.all(free_labels == 1):
            raise AssertionError("moving point must become seen-free after later rays: {} -> {}".format(before, labels))
        if not np.all(wall_labels == 0) or not np.all(occluded_labels == 0):
            raise AssertionError("static wall and occluded unknown space must remain static")
        report = {"num_threads_requested": threads, "d_p": dp, "moving_before": before.tolist(),
                  "moving_after": labels.tolist(), "free_space_labels": free_labels.tolist(),
                  "static_wall_points": int(np.count_nonzero(wall_labels == 0)),
                  "dynamic_wall_points": int(np.count_nonzero(wall_labels == 1)),
                  "occluded_labels": occluded_labels.tolist()}
        all_reports.append(report)
    try:
        backend.dufomap(.1, .2, 1, num_threads=1)
    except ValueError:
        result["serial_rejected"] = True
    else:
        raise AssertionError("num_threads=1 must reject the upstream unimplemented serial branch")
    result["runs"] = all_reports
    result["passed"] = True
    if output_dir:
        output_dir.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(output_dir / "scene.npz", wall=wall, pose=pose,
                            moving=moving, **{"frame_{}".format(i): frame for i, frame in enumerate(frames)})
        (output_dir / "smoke.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path)
    run(parser.parse_args().output_dir)
