#!/usr/bin/env python3
"""Unit and optional real-backend tests for the DUFOMap export adapter."""

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = PROJECT_ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))

import dufomap_export_filter as adapter
from dufomap_export_filter import (
    FilterError,
    PcdCloud,
    _load_official_backend,
    read_pcd,
    run_filter,
    write_pcd,
)


def make_cloud(points, intensity=None, ring=None, time=None):
    names = ["x", "y", "z"]
    formats = ["<f4", "<f4", "<f4"]
    columns = [np.asarray(points, dtype=np.float32)[:, 0],
               np.asarray(points, dtype=np.float32)[:, 1],
               np.asarray(points, dtype=np.float32)[:, 2]]
    if intensity is not None:
        names.append("intensity")
        formats.append("<f4")
        columns.append(np.asarray(intensity, dtype=np.float32))
    if ring is not None:
        names.append("ring")
        formats.append("<u2")
        columns.append(np.asarray(ring, dtype=np.uint16))
    if time is not None:
        names.append("time")
        formats.append("<f8")
        columns.append(np.asarray(time, dtype=np.float64))
    data = np.zeros(len(columns[0]), dtype=np.dtype(list(zip(names, formats))))
    for name, values in zip(names, columns):
        data[name] = values
    metadata = {
        "version": ".7",
        "fields": names,
        "size": [np.dtype(fmt).itemsize for fmt in formats],
        "type": ["F" if fmt[1] in "f" else "U" for fmt in formats],
        "count": [1] * len(names),
        "width": len(columns[0]),
        "height": 1,
        "viewpoint": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        "points": len(columns[0]),
        "data": "binary",
    }
    return PcdCloud(metadata, data)


class FakeDufomap:
    instances = []
    events = []

    def __init__(self, resolution, d_s, d_p, num_threads=0):
        self.resolution = resolution
        self.d_s = d_s
        self.d_p = d_p
        self.num_threads = num_threads
        self.runs = []
        self.segment_calls = []
        FakeDufomap.instances.append(self)
        FakeDufomap.events.append("init")

    def run(self, points, pose, cloud_transform=True):
        assert cloud_transform is True
        self.runs.append((np.asarray(points).copy(), np.asarray(pose).copy()))
        FakeDufomap.events.append("run")

    def oncePropagateCluster(self, if_propagate=False, if_cluster=False):
        assert if_propagate is True
        assert if_cluster is False
        FakeDufomap.events.append("propagate")

    def segment(self, points, pose, cloud_transform=True):
        assert cloud_transform is False
        assert np.allclose(pose, np.eye(4))
        self.segment_calls.append(np.asarray(points).copy())
        FakeDufomap.events.append("segment")
        # Controlled stand-in: world x >= 5 is "dynamic".
        return (np.asarray(points)[:, 0] >= 5.0).astype(np.uint8)


def fake_factory(parameters):
    return FakeDufomap(
        parameters["resolution"],
        parameters["d_s"],
        parameters["d_p"],
        parameters.get("num_threads", 0),
    )


class DatasetFixture:
    def __init__(self, root: Path):
        self.root = root
        self.dataset = root / "dataset"
        self.output = root / "output"
        (self.dataset / "scans").mkdir(parents=True)
        (self.dataset / "targets").mkdir()
        self.output.mkdir()

    def add_frame(self, index, local_points, pose=None):
        if pose is None:
            pose = np.eye(4)
        write_pcd(
            self.dataset / "scans" / f"{index:06d}.pcd",
            make_cloud(local_points, intensity=np.arange(len(local_points), dtype=np.float32)),
        )
        return {
            "index": index,
            "scan": f"scans/{index:06d}.pcd",
            "pose": np.asarray(pose, dtype=np.float64).reshape(-1).tolist(),
        }

    def add_target(self, name, points, output, **kwargs):
        write_pcd(self.dataset / "targets" / f"{name}_raw.pcd", make_cloud(points, **kwargs))
        return {
            "name": name,
            "input": f"targets/{name}_raw.pcd",
            "output": output,
        }

    def write(self, frames, targets, parameters=None):
        manifest = {
            "format": "lio_sam_dufomap_export",
            "version": 1,
            "parameters": {
                "resolution": 0.1,
                "d_s": 0.2,
                "d_p": 2,
                "num_threads": 0,
                "min_range": 0.2,
                "max_range": 0.0,
                "if_propagate": True,
                "if_cluster": False,
            },
            "frames": frames,
            "targets": targets,
            "summary": "summary.json",
        }
        if parameters:
            manifest["parameters"].update(parameters)
        (self.dataset / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
        return manifest


class AdapterTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.fixture = DatasetFixture(self.root)
        FakeDufomap.instances.clear()
        FakeDufomap.events.clear()

    def tearDown(self):
        self.temporary.cleanup()

    def test_rejects_unsupported_propagation_and_single_thread(self):
        frames = [self.fixture.add_frame(0, [[1, 0, 0]])]
        targets = [self.fixture.add_target("corner", [[1, 0, 0]], "corner.pcd")]
        for parameters in ({"d_p": 5}, {"num_threads": 1}):
            with self.subTest(parameters=parameters):
                self.fixture.write(frames, targets, parameters=parameters)
                with self.assertRaises(FilterError):
                    run_filter(self.fixture.dataset, self.fixture.output,
                               backend_factory=fake_factory)

    def test_filters_targets_through_official_api_contract_and_preserves_fields(self):
        frames = [
            self.fixture.add_frame(
                0,
                [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
                pose=[[1, 0, 0, 1.0], [0, 1, 0, 2.0], [0, 0, 1, 3.0], [0, 0, 0, 1]],
            )
        ]
        target = self.fixture.add_target(
            "corner",
            [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
            "corner.pcd",
            intensity=[11.0, 22.0, 33.0],
            ring=[4, 5, 6],
            time=[0.1, 0.2, 0.3],
        )
        self.fixture.write(frames, [target])

        summary = run_filter(
            self.fixture.dataset,
            self.fixture.output,
            self.root / "summary.json",
            backend_factory=fake_factory,
        )

        backend = FakeDufomap.instances[0]
        self.assertEqual(len(backend.runs), 1)
        # The local scan keeps the lidar-frame coordinates.
        np.testing.assert_allclose(
            backend.runs[0][0],
            np.asarray([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [10.0, 0.0, 0.0]], dtype=np.float32),
        )
        np.testing.assert_allclose(
            backend.runs[0][1],
            np.asarray(frames[0]["pose"], dtype=np.float64).reshape(4, 4),
        )
        self.assertEqual(FakeDufomap.events[0], "init")
        self.assertLess(
            FakeDufomap.events.index("propagate"),
            FakeDufomap.events.index("segment"),
        )
        self.assertEqual(len(backend.segment_calls), 1)

        output = read_pcd(self.fixture.output / "corner.pcd")
        self.assertEqual(output.data.shape[0], 2)
        np.testing.assert_array_equal(output.data["x"], np.asarray([1.0, 2.0], dtype=np.float32))
        np.testing.assert_array_equal(output.data["intensity"], np.asarray([11.0, 22.0], dtype=np.float32))
        np.testing.assert_array_equal(output.data["ring"], np.asarray([4, 5], dtype=np.uint16))
        np.testing.assert_array_equal(output.data["time"], np.asarray([0.1, 0.2], dtype=np.float64))

        saved_summary = json.loads((self.root / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(saved_summary["frames"], 1)
        self.assertEqual(saved_summary["targets"][0]["points_kept"], 2)
        self.assertEqual(saved_summary["targets"][0]["points_removed"], 1)
        self.assertEqual(summary["backend"], "injected-test-backend")

    def test_ground_is_not_a_special_target(self):
        # Ground is no longer part of the dataset. A surface point coincident
        # with a ground-like scan return is classified like any other target;
        # the adapter must not invent a ground output or require one.
        frame = self.fixture.add_frame(
            0, [[1.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 0.0, 0.0]])
        target = self.fixture.add_target(
            "surface", [[1.0, 0.0, 0.0], [10.0, 0.0, 0.0]], "surface.pcd")
        self.fixture.write([frame], [target])

        summary = run_filter(
            self.fixture.dataset,
            self.fixture.output,
            self.root / "summary.json",
            backend_factory=fake_factory,
        )

        self.assertEqual([item["name"] for item in summary["targets"]], ["surface"])
        output = read_pcd(self.fixture.output / "surface.pcd")
        np.testing.assert_array_equal(
            output.data["x"], np.asarray([1.0], dtype=np.float32))
        self.assertFalse((self.fixture.output / "ground.pcd").exists())
        # The full scan still contains the ground-like return as ray evidence.
        self.assertEqual(FakeDufomap.instances[0].runs[0][0].shape[0], 3)

    def test_missing_scan_is_a_failure_not_an_empty_map(self):
        frame = self.fixture.add_frame(0, [[1.0, 0.0, 0.0]])
        target = self.fixture.add_target("surf", [[1.0, 0.0, 0.0]], "surf.pcd")
        manifest = self.fixture.write([frame], [target])
        (self.fixture.dataset / frame["scan"]).unlink()
        with self.assertRaises(FilterError) as caught:
            run_filter(self.fixture.dataset, self.fixture.output, backend_factory=fake_factory)
        self.assertIn("cannot open", str(caught.exception))
        self.assertFalse((self.fixture.output / "surf.pcd").exists())

    def test_missing_target_is_a_failure(self):
        frame = self.fixture.add_frame(0, [[1.0, 0.0, 0.0]])
        target = self.fixture.add_target("surf", [[1.0, 0.0, 0.0]], "surf.pcd")
        self.fixture.write([frame], [target])
        (self.fixture.dataset / target["input"]).unlink()
        with self.assertRaises(FilterError) as caught:
            run_filter(self.fixture.dataset, self.fixture.output, backend_factory=fake_factory)
        self.assertIn("cannot open", str(caught.exception))

    def test_missing_official_backend_has_an_actionable_error(self):
        with mock.patch.dict(sys.modules, {"dufomap": None}):
            with self.assertRaises(FilterError) as caught:
                _load_official_backend()
        message = str(caught.exception)
        self.assertIn("official DUFOMap", message)
        self.assertIn("install_dufomap_backend.sh --smoke", message)

    def test_backend_exception_propagates_and_does_not_write_output(self):
        frame = self.fixture.add_frame(0, [[1.0, 0.0, 0.0]])
        target = self.fixture.add_target("surf", [[1.0, 0.0, 0.0]], "surf.pcd")
        self.fixture.write([frame], [target])

        class BrokenBackend(FakeDufomap):
            def segment(self, points, pose, cloud_transform=True):
                raise RuntimeError("backend exploded")

        with self.assertRaises(RuntimeError):
            run_filter(
                self.fixture.dataset,
                self.fixture.output,
                backend_factory=lambda parameters: BrokenBackend(0.1, 0.2, 2),
            )
        self.assertFalse((self.fixture.output / "surf.pcd").exists())

    def test_empty_scan_after_range_gate_fails(self):
        frame = self.fixture.add_frame(0, [[0.01, 0.0, 0.0]])
        target = self.fixture.add_target("surf", [[1.0, 0.0, 0.0]], "surf.pcd")
        self.fixture.write([frame], [target])
        with self.assertRaises(FilterError) as caught:
            run_filter(self.fixture.dataset, self.fixture.output, backend_factory=fake_factory)
        self.assertIn("range gate", str(caught.exception))


class CommandLineTest(unittest.TestCase):
    def test_cli_runs_fake_official_package_and_writes_summary(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = DatasetFixture(root)
            frame = fixture.add_frame(0, [[1.0, 0.0, 0.0], [10.0, 0.0, 0.0]])
            target = fixture.add_target(
                "global",
                [[1.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
                "global.pcd",
                intensity=[7.0, 8.0],
            )
            fixture.write([frame], [target])

            package = root / "fake_pythonpath" / "dufomap"
            package.mkdir(parents=True)
            (package / "__init__.py").write_text(
                "import numpy as np\n"
                "__version__ = 'fake-1.0'\n"
                "class dufomap:\n"
                "    def __init__(self, resolution, d_s, d_p, num_threads=0):\n"
                "        self.runs = 0\n"
                "    def run(self, points, pose, cloud_transform=True):\n"
                "        self.runs += 1\n"
                "    def oncePropagateCluster(self, if_propagate=False, if_cluster=False):\n"
                "        assert if_propagate and not if_cluster\n"
                "    def segment(self, points, pose, cloud_transform=False):\n"
                "        return (np.asarray(points)[:, 0] >= 5.0).astype(np.uint8)\n",
                encoding="utf-8",
            )
            environment = dict(os.environ)
            environment["PYTHONPATH"] = str(package.parent)
            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPTS / "dufomap_export_filter.py"),
                    "--dataset",
                    str(fixture.dataset),
                    "--output",
                    str(fixture.output),
                    "--summary",
                    str(root / "summary.json"),
                ],
                env=environment,
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            summary = json.loads((root / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(summary["backend_version"], "fake-1.0")
            self.assertEqual(summary["targets"][0]["points_kept"], 1)
            output = read_pcd(fixture.output / "global.pcd")
            np.testing.assert_array_equal(output.data["intensity"], np.asarray([7.0], dtype=np.float32))

    def test_cli_returns_nonzero_when_backend_is_missing(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = DatasetFixture(root)
            frame = fixture.add_frame(0, [[1.0, 0.0, 0.0]])
            target = fixture.add_target("global", [[1.0, 0.0, 0.0]], "global.pcd")
            fixture.write([frame], [target])
            environment = dict(os.environ)
            (root / "dufomap.py").write_text("raise ImportError('test backend unavailable')\n")
            environment["PYTHONPATH"] = str(root)
            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPTS / "dufomap_export_filter.py"),
                    "--dataset",
                    str(fixture.dataset),
                    "--output",
                    str(fixture.output),
                ],
                env=environment,
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("official DUFOMap", result.stderr)


def _official_dufomap_available():
    try:
        import dufomap  # noqa: F401
    except Exception:
        return False
    return True


@unittest.skipUnless(
    _official_dufomap_available(),
    "official dufomap package is not installed for this interpreter",
)
class OfficialBackendSmokeTest(unittest.TestCase):
    """Real default-parameter scene with dense rays, movement and occlusion."""

    def test_static_wall_survives_and_moving_object_is_removed(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = DatasetFixture(root)
            # d_p erodes unknown space: isolated sparse returns cannot establish
            # the free-space neighborhood required for a dynamic classification.
            wall = np.asarray(
                [[10.05, y / 20 + .05, z / 20 + .05]
                 for y in range(-60, 61) for z in range(-40, 41)],
                dtype=np.float32,
            )
            pose = np.eye(4, dtype=np.float64)
            pose[:3, 3] = .05
            moving = []
            frames = []
            for index, y in enumerate((.35, 1.35, -1.35, 1.35)):
                point = np.asarray([5.05, y, .35], dtype=np.float32)
                ratio = (point[0] - .05) / (wall[:, 0] - .05)
                intersection = .05 + (wall - .05) * ratio[:, None]
                # Remove wall rays occluded by the foreground footprint.
                hidden = ((np.abs(intersection[:, 1] - y) < .15) &
                          (np.abs(intersection[:, 2] - .35) < .15))
                world = np.vstack((wall[~hidden], point))
                local = (world - pose[:3, 3]).astype(np.float32)
                frames.append(fixture.add_frame(index, local, pose))
                moving.append(point)
            occluded = np.asarray([[12.05, .35, .35], [12.05, -.35, -.35]], dtype=np.float32)
            target_points = np.vstack((wall, moving, occluded))
            ids = np.arange(len(target_points))
            target = fixture.add_target(
                "global", target_points, "global.pcd",
                intensity=ids.astype(np.float32),
                ring=(ids % 32).astype(np.uint16),
                time=ids.astype(np.float64) * .0001,
            )
            # Exercise actual shipping defaults: .1m resolution, d_s=.2m,
            # d_p=2, automatic threads. There is no stand-in backend here.
            fixture.write(frames, [target])
            summary = run_filter(fixture.dataset, fixture.output, root / "summary.json")
            output = read_pcd(fixture.output / "global.pcd")
            input_cloud = read_pcd(fixture.dataset / target["input"])
            kept_ids = output.data["intensity"].astype(np.int64)
            np.testing.assert_array_equal(
                np.sort(kept_ids[kept_ids < len(wall)]), np.arange(len(wall))
            )
            self.assertNotIn(len(wall), kept_ids, "the first moving position must become free")
            self.assertTrue(np.all(np.isin(ids[-2:], kept_ids)), "occluded unknown space stays static")
            self.assertGreater(summary["targets"][0]["points_removed"], 0)
            # Every retained XYZ/intensity/ring/time value must be untouched.
            np.testing.assert_array_equal(output.data, input_cloud.data[kept_ids])


if __name__ == "__main__":
    unittest.main()
