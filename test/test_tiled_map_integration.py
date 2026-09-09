#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import yaml


INTEGRATION_DIR = Path(__file__).resolve().parent / "integration"
if str(INTEGRATION_DIR) not in sys.path:
    sys.path.insert(0, str(INTEGRATION_DIR))

from localization_regression import tiled_fault_violations, tiled_metric_violations
from stability_common import (
    DatasetError,
    prepare_tiled_fault_case,
    validate_dataset,
    write_runtime_parameters,
)


class TiledMapIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.bag = self.root / "bag"
        self.map = self.root / "map"
        self.tiled = self.map / "tiled"
        self.bag.mkdir()
        self.map.mkdir()
        (self.bag / "metadata.yaml").write_text(
            """rosbag2_bagfile_information:
  duration: {nanoseconds: 1}
  message_count: 1
  topics_with_message_count: []
""",
            encoding="utf-8",
        )
        (self.bag / "data.db3").write_bytes(b"bag")
        (self.map / "GlobalMap.pcd").write_bytes(
            b"VERSION .7\nPOINTS 1\nDATA binary\n" + b"\0" * 16
        )
        (self.map / "GroundMap.pcd").write_bytes(b"ground")
        (self.map / "sc_db.bin").write_bytes(b"sc")
        self.template = self.root / "params.yaml"
        self.template.write_text(
            """localization:
  map_path: old-map
  map_mode: auto
  ground_map_path: old-ground
  sc_db_path: old-db
""",
            encoding="utf-8",
        )

    def tearDown(self):
        self.temporary_directory.cleanup()

    def test_runtime_parameters_select_tiled_directory(self):
        self.tiled.mkdir()
        runtime = write_runtime_parameters(
            self.template, self.map, self.root / "out", tiled_dir=self.tiled
        )
        parameters = yaml.safe_load(runtime.read_text(encoding="utf-8"))["localization"]
        self.assertEqual(parameters["map_mode"], "tiled")
        self.assertEqual(parameters["map_path"], str(self.tiled.resolve()))
        self.assertEqual(parameters["sc_db_path"], str((self.map / "sc_db.bin").resolve()))

    def test_tiled_validation_requires_manifest(self):
        self.tiled.mkdir()
        with self.assertRaisesRegex(DatasetError, "manifest.yaml"):
            validate_dataset(self.bag, self.map, tiled_dir=self.tiled)
        (self.tiled / "manifest.yaml").write_text("schema_version: 1\n", encoding="utf-8")
        validate_dataset(self.bag, self.map, tiled_dir=self.tiled)

    def test_legacy_validation_does_not_require_manifest(self):
        validate_dataset(self.bag, self.map)

    def test_tiled_validation_rejects_directory_outside_map_root(self):
        outside = self.root / "outside"
        outside.mkdir()
        (outside / "manifest.yaml").write_text("schema_version: 1\n", encoding="utf-8")
        with self.assertRaisesRegex(DatasetError, "inside selected map root"):
            validate_dataset(self.bag, self.map, tiled_dir=outside)

    def test_tiled_metric_invariants(self):
        healthy = {
            "first_ok_seconds": 1.0,
            "map_mode": "tiled",
            "map_cached_tile_count": 25,
            "map_cache_mib": 1024,
            "map_tile_load_failures": 0,
        }
        self.assertEqual(tiled_metric_violations(healthy, 5.0), [])
        broken = dict(healthy)
        broken.update(
            map_mode="global",
            map_cached_tile_count=26,
            map_cache_mib=1025,
            map_tile_load_failures=1,
            first_ok_seconds=None,
        )
        self.assertEqual(len(tiled_metric_violations(broken, 5.0)), 5)

    def test_fault_fixtures_are_small_and_remove_selected_index(self):
        self.tiled.mkdir()
        for relative in ("global/0_0.pcd", "global/1_0.pcd", "global/8_8.pcd"):
            path = self.tiled / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(relative.encode())
        manifest = {
            "schema_version": 1,
            "frame_id": "map",
            "point_type": "XYZI",
            "tile_size_m": 100.0,
            "origin_xy": [0.0, 0.0],
            "source": {
                "global_map_name": "GlobalMap.pcd", "global_map_bytes": 1,
                "ground_map_name": "GroundMap.pcd", "ground_map_bytes": 1,
            },
            "tiles": {"global": [], "ground": []},
        }
        for index, relative in (((0, 0), "global/0_0.pcd"),
                                ((1, 0), "global/1_0.pcd"),
                                ((8, 8), "global/8_8.pcd")):
            manifest["tiles"]["global"].append({
                "index": list(index), "path": relative,
                "point_count": 1, "bytes": (self.tiled / relative).stat().st_size,
            })
        (self.tiled / "manifest.yaml").write_text(
            yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8"
        )
        fixture = self.root / "fault"
        missing = prepare_tiled_fault_case(
            self.tiled, fixture, "missing-neighbor"
        )
        fixture_manifest = yaml.safe_load(
            (fixture / "manifest.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual(missing, "[1,0]")
        self.assertEqual(
            [entry["index"] for entry in fixture_manifest["tiles"]["global"]],
            [[0, 0]],
        )
        self.assertFalse((fixture / "global/8_8.pcd").exists())

    def test_fault_invariants_require_refusal_or_exact_degraded_index(self):
        refused = {"map_tile_load_state": "error", "first_ok_seconds": None}
        self.assertEqual(
            tiled_fault_violations(refused, "corrupt-manifest"), []
        )
        degraded = {
            "map_tile_load_state": "ready",
            "map_tile_load_error": "missing tiles: [1,0]",
            "max_diagnostic_level": 1,
        }
        self.assertEqual(
            tiled_fault_violations(degraded, "missing-neighbor", "[1,0]"), []
        )

    def test_shell_entrypoint_exposes_all_tiled_modes(self):
        script = (Path(__file__).resolve().parents[1] / "scripts" /
                  "run_localization_stability.sh").read_text(encoding="utf-8")
        for mode in ("tiled-generate", "tiled-quick", "tiled-faults"):
            with self.subTest(mode=mode):
                self.assertIn(mode, script)


if __name__ == "__main__":
    unittest.main()
