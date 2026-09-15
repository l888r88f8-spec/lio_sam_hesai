#!/usr/bin/env python3
import json
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

    def _write_small_tiled_map(self, global_paths=None):
        # exist_ok: a safety test may need its files in place before the
        # manifest records their byte sizes.
        self.tiled.mkdir(parents=True, exist_ok=True)
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
        declared = global_paths or ["global/0_0.pcd", "global/1_0.pcd",
                                    "global/8_8.pcd"]
        for index, relative in zip(((0, 0), (1, 0), (8, 8)), declared):
            manifest["tiles"]["global"].append({
                "index": list(index), "path": relative,
                "point_count": 1, "bytes": (self.tiled / relative).stat().st_size,
            })
        (self.tiled / "manifest.yaml").write_text(
            yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8"
        )

    def _prepare(self, fault_case="missing-neighbor", center_index=((0, 0))):
        return prepare_tiled_fault_case(
            self.tiled, self.root / "fault", fault_case,
            center_index=center_index,
        )

    def test_fault_fixture_copies_the_map_and_keeps_the_manifest(self):
        self._write_small_tiled_map()
        prepared = self._prepare()

        self.assertEqual(prepared.center, (0, 0))
        self.assertEqual(prepared.fault_index, (1, 0))
        self.assertEqual(prepared.expected_index, "[1,0]")

        fixture_manifest = yaml.safe_load(
            (prepared.tiled_root / "manifest.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual(
            [entry["index"] for entry in fixture_manifest["tiles"]["global"]],
            [[0, 0], [1, 0], [8, 8]],
        )
        # The whole map is copied and nothing is removed up front: the fault is
        # a runtime load failure, not a manifest validation failure.
        for relative in ("global/0_0.pcd", "global/1_0.pcd", "global/8_8.pcd"):
            self.assertTrue((prepared.tiled_root / relative).is_file())

    def test_fault_injection_moves_only_the_faulty_payload(self):
        self._write_small_tiled_map()
        prepared = self._prepare()

        quarantined = prepared.inject()

        self.assertFalse((prepared.tiled_root / prepared.fault_relative).exists())
        self.assertTrue(quarantined.is_file())
        self.assertEqual(quarantined.name, "1_0.pcd")
        # The manifest still declares it, and the other tiles are untouched.
        fixture_manifest = yaml.safe_load(
            (prepared.tiled_root / "manifest.yaml").read_text(encoding="utf-8")
        )
        self.assertIn([1, 0],
                      [entry["index"] for entry in fixture_manifest["tiles"]["global"]])
        self.assertTrue((prepared.tiled_root / "global/0_0.pcd").is_file())
        self.assertTrue((prepared.tiled_root / "global/8_8.pcd").is_file())

    def test_fault_fixture_requires_a_declared_center(self):
        self._write_small_tiled_map()
        with self.assertRaises(DatasetError):
            prepare_tiled_fault_case(
                self.tiled, self.root / "fault-none", "missing-neighbor",
                center_index=None)
        with self.assertRaises(DatasetError):
            prepare_tiled_fault_case(
                self.tiled, self.root / "fault-absent", "missing-neighbor",
                center_index=((5, 5)))
        with self.assertRaises(DatasetError):
            prepare_tiled_fault_case(
                self.tiled, self.root / "fault-bool", "missing-neighbor",
                center_index=((True, 0)))

    def test_corrupt_manifest_requires_a_center(self):
        # A None center used to reach the tuple() below and raise TypeError
        # instead of the documented DatasetError.
        self._write_small_tiled_map()
        with self.assertRaises(DatasetError):
            prepare_tiled_fault_case(
                self.tiled, self.root / "fault-none", "corrupt-manifest",
                center_index=None)
        self.assertFalse((self.root / "fault-none").exists())

    def test_fault_fixture_refuses_unsafe_tile_paths(self):
        self._write_small_tiled_map()
        manifest = yaml.safe_load(
            (self.tiled / "manifest.yaml").read_text(encoding="utf-8"))
        manifest["tiles"]["global"][1]["path"] = "../escape.pcd"
        (self.tiled / "manifest.yaml").write_text(
            yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
        with self.assertRaises(DatasetError):
            self._prepare()

    def test_corrupt_manifest_fixture_has_no_payload_to_inject(self):
        self._write_small_tiled_map()
        prepared = prepare_tiled_fault_case(
            self.tiled, self.root / "fault", "corrupt-manifest",
            center_index=((0, 0)))
        self.assertIsNone(prepared.expected_index)
        with self.assertRaises(DatasetError):
            prepared.inject()

    def test_corrupt_manifest_requires_the_node_to_die(self):
        # Both signals are required: the node itself exiting non-zero, and the
        # reason it refused.
        passing = {
            "corrupt_manifest_node_died": True,
            "corrupt_manifest_map_preparation_failed": True,
        }
        self.assertEqual(
            tiled_fault_violations(passing, "corrupt-manifest"), [])

        self.assertEqual(
            tiled_fault_violations({}, "corrupt-manifest"),
            ["corrupt-manifest must make the localization node exit "
             "non-zero, not merely kill it during cleanup",
             "corrupt-manifest must fail map preparation"],
        )
        self.assertTrue(tiled_fault_violations(
            dict(passing, corrupt_manifest_node_died=False), "corrupt-manifest"))
        self.assertTrue(tiled_fault_violations(
            dict(passing, corrupt_manifest_map_preparation_failed=False),
            "corrupt-manifest"))
        self.assertTrue(tiled_fault_violations(
            dict(passing, first_ok_seconds=1.0), "corrupt-manifest"))

    def test_cleanup_deaths_alone_do_not_satisfy_corrupt_manifest(self):
        # The normal teardown kills the static TF publishers and the ground
        # segmentation node, so a bare "process has died" must not count.
        import localization_regression as regression

        cleanup_only = (
            "[static_transform_publisher-1]: process has died "
            "[pid 1, exit code -2, cmd '...'].\n"
            "[lio_sam_hesai_imageProjection-4]: process has died "
            "[pid 2, exit code -2, cmd '...'].\n"
        )
        self.assertIsNone(regression._RELOCALIZATION_DEATH.search(cleanup_only))

        # The real refusal: the localization node exits non-zero.
        refused = (
            "[lio_sam_hesai_relocalization-3]: process has died "
            "[pid 3, exit code -6, cmd '...'].\n"
            "Failed to prepare localization map."
        )
        death = regression._RELOCALIZATION_DEATH.search(refused)
        self.assertIsNotNone(death)
        self.assertEqual(int(death.group(1)), -6)
        self.assertIn(regression._MAP_PREPARATION_FAILURE, refused)

        # A clean exit of the same node is not a refusal either.
        clean = ("[lio_sam_hesai_relocalization-3]: process has died "
                 "[pid 3, exit code 0, cmd '...'].")
        self.assertEqual(
            int(regression._RELOCALIZATION_DEATH.search(clean).group(1)), 0)

    def test_missing_center_requires_a_real_failure_and_no_output(self):
        passing = {
            "map_tile_events": [["ready", "0", ""], ["ready", "1", "center [0,0]"]],
            "first_ok_seconds": None,
            "valid_odom_ratio": 0.0,
        }
        self.assertEqual(
            tiled_fault_violations(passing, "missing-center", "[0,0]"), [])
        # Producing output means the centre was never actually lost.
        with_output = dict(passing, first_ok_seconds=2.0)
        self.assertTrue(
            tiled_fault_violations(with_output, "missing-center", "[0,0]"))

    def test_missing_neighbor_requires_output_and_a_named_failure(self):
        passing = {
            "map_tile_events": [
                ["ready", "0", ""],
                ["ready", "1", "missing tiles: [1,0]"],
            ],
            "first_ok_seconds": 2.5,
            "valid_odom_ratio": 1.0,
        }
        self.assertEqual(
            tiled_fault_violations(passing, "missing-neighbor", "[1,0]"), [])
        # A degraded run that never localized is not a pass.
        self.assertTrue(tiled_fault_violations(
            dict(passing, first_ok_seconds=None), "missing-neighbor", "[1,0]"))
        self.assertTrue(tiled_fault_violations(
            dict(passing, valid_odom_ratio=0.0), "missing-neighbor", "[1,0]"))

    def test_false_pass_paths_are_rejected(self):
        localized = {"first_ok_seconds": 2.5, "valid_odom_ratio": 1.0}

        # The target tile was never requested: only the startup state is seen.
        startup_only = dict(
            localized, map_tile_events=[["ready", "0", ""]])
        self.assertTrue(tiled_fault_violations(
            startup_only, "missing-neighbor", "[1,0]"))

        # A startup WARN with no tile state at all proves nothing.
        startup_warn = dict(localized, map_tile_events=[])
        self.assertTrue(tiled_fault_violations(
            startup_warn, "missing-neighbor", "[1,0]"))

        # The index appears only in the final summary, never during a request.
        final_error_only = dict(localized, map_tile_events=[
            ["ready", "0", ""],
            ["ready", "1", "unrelated tile failed"],
        ], map_tile_load_error="missing tiles: [1,0]")
        self.assertTrue(tiled_fault_violations(
            final_error_only, "missing-neighbor", "[1,0]"))

        # Startup timed out: no diagnostics were ever observed.
        self.assertTrue(tiled_fault_violations(
            localized, "missing-neighbor", "[1,0]"))

    def test_unknown_fault_case_is_reported(self):
        self.assertEqual(
            tiled_fault_violations({}, "no-such-case"),
            ["unknown tiled fault case: no-such-case"],
        )

    def test_shell_entrypoint_exposes_all_tiled_modes(self):
        script = (Path(__file__).resolve().parents[1] / "scripts" /
                  "run_localization_stability.sh").read_text(encoding="utf-8")
        for mode in ("tiled-generate", "tiled-quick", "tiled-faults"):
            with self.subTest(mode=mode):
                self.assertIn(mode, script)

    def test_fault_fixture_refuses_paths_that_climb_out(self):
        # A ".." element is what let a symlinked source directory, which absorbs
        # one level, turn into a destination outside the fixture.
        self.tiled.mkdir(parents=True, exist_ok=True)
        (self.tiled / "x" / "y" / "z").mkdir(parents=True)
        (self.tiled / "a").symlink_to(Path("x") / "y" / "z")
        for name in ("0_0.pcd", "1_0.pcd", "8_8.pcd"):
            (self.tiled / name).write_bytes(b"x")
        self._write_small_tiled_map(
            ["a/../../../0_0.pcd", "a/../../../1_0.pcd", "a/../../../8_8.pcd"])
        with self.assertRaises(DatasetError):
            self._prepare()

    def test_fault_fixture_refuses_symlinked_tile_directories(self):
        # A symlinked parent redirects the whole subtree while the leaf still
        # looks like an ordinary file.
        self.tiled.mkdir(parents=True, exist_ok=True)
        (self.tiled / "real").mkdir()
        for name in ("0_0.pcd", "1_0.pcd", "8_8.pcd"):
            (self.tiled / "real" / name).write_bytes(b"x")
        (self.tiled / "linked").symlink_to(Path("real"))
        self._write_small_tiled_map(
            ["linked/0_0.pcd", "linked/1_0.pcd", "linked/8_8.pcd"])
        with self.assertRaises(DatasetError):
            self._prepare()

    def test_fault_injection_refuses_a_planted_quarantine(self):
        # rename() replaces silently, so a quarantine symlink pointing at the
        # real map could overwrite a same-named map file.
        self._write_small_tiled_map()
        prepared = self._prepare()
        real_map = self.root / "real-map"
        real_map.mkdir()
        (real_map / "1_0.pcd").write_bytes(b"REAL MAP DATA")
        (prepared.root / "quarantine").symlink_to(real_map)

        with self.assertRaises(DatasetError):
            prepared.inject()

        self.assertEqual((real_map / "1_0.pcd").read_bytes(), b"REAL MAP DATA")

    def test_fault_injection_refuses_to_overwrite_quarantined_payload(self):
        self._write_small_tiled_map()
        prepared = self._prepare()
        quarantine = prepared.root / "quarantine"
        quarantine.mkdir()
        (quarantine / "1_0.pcd").write_bytes(b"already here")

        with self.assertRaises(DatasetError):
            prepared.inject()

        self.assertEqual((quarantine / "1_0.pcd").read_bytes(), b"already here")

    def test_injection_refuses_a_directory_that_is_not_a_fixture(self):
        # A --tiled-dir pointed at a real map must not be able to move live
        # tiles: without the fixture marker, nothing is touched.
        self._write_small_tiled_map()
        from stability_common import TiledFaultFixture

        real_map = TiledFaultFixture(
            root=self.tiled, tiled_root=self.tiled, center=(0, 0),
            fault_case="missing-neighbor", fault_index=(1, 0),
            fault_relative="global/1_0.pcd", active_radius=1, prefetch_radius=1)
        with self.assertRaises(DatasetError):
            real_map.inject()
        self.assertTrue((self.tiled / "global/1_0.pcd").is_file())
        self.assertTrue((self.tiled / "manifest.yaml").is_file())

    def test_injection_refuses_a_mismatched_marker(self):
        self._write_small_tiled_map()
        prepared = self._prepare()
        marker = prepared.tiled_root / ".p22-fault-fixture.json"
        payload = json.loads(marker.read_text(encoding="utf-8"))
        payload["fault_relative"] = "global/0_0.pcd"
        marker.write_text(json.dumps(payload), encoding="utf-8")

        with self.assertRaises(DatasetError):
            prepared.inject()
        self.assertTrue((prepared.tiled_root / "global/1_0.pcd").is_file())

    def test_injection_refuses_a_payload_the_manifest_does_not_declare(self):
        self._write_small_tiled_map()
        prepared = self._prepare()
        fixture_manifest = prepared.tiled_root / "manifest.yaml"
        manifest = yaml.safe_load(fixture_manifest.read_text(encoding="utf-8"))
        manifest["tiles"]["global"] = [
            entry for entry in manifest["tiles"]["global"]
            if entry["index"] != [1, 0]
        ]
        fixture_manifest.write_text(
            yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")

        with self.assertRaises(DatasetError):
            prepared.inject()

    def test_loading_a_fixture_requires_the_marker(self):
        from stability_common import load_tiled_fault_fixture

        self._write_small_tiled_map()
        with self.assertRaises(DatasetError):
            load_tiled_fault_fixture(self.tiled)

        prepared = self._prepare()
        loaded = load_tiled_fault_fixture(prepared.tiled_root)
        self.assertEqual(loaded.fault_case, "missing-neighbor")
        self.assertEqual(loaded.fault_index, (1, 0))
        self.assertEqual(loaded.fault_relative, "global/1_0.pcd")


if __name__ == "__main__":
    unittest.main()
