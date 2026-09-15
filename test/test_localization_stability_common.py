#!/usr/bin/env python3
import hashlib
import shutil
import signal
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

import yaml

from stability_common import (
    DatasetError,
    terminate_process_group,
    validate_dataset,
    write_runtime_parameters,
)


# Mirrors MapArtifactLayout (include/common/map_artifact_layout.h). Python
# cannot link the C++ value object, so the names live here once instead of
# being repeated at every call site.
GLOBAL_MAP_NAME = "GlobalMap.pcd"
GROUND_MAP_NAME = "GroundMap.pcd"
SCAN_CONTEXT_DB_NAME = "sc_db.bin"


class LocalizationStabilityCommonTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.bag_dir = self.root / "bag"
        self.map_dir = self.root / "map"
        self._write_fixture()

    def tearDown(self):
        self.temporary_directory.cleanup()

    def _write_fixture(self):
        self.bag_dir.mkdir()
        (self.bag_dir / "metadata.yaml").write_text(
            """rosbag2_bagfile_information:
  duration:
    nanoseconds: 756378139407
  message_count: 46875
  topics_with_message_count:
    - topic_metadata:
        name: /imu
      message_count: 42617
    - topic_metadata:
        name: /lidar_points
      message_count: 4258
""",
            encoding="utf-8",
        )
        (self.bag_dir / "sample.db3").write_bytes(b"sample bag data")

        self.map_dir.mkdir()
        (self.map_dir / GLOBAL_MAP_NAME).write_bytes(
            b"# .PCD v0.7\n"
            b"VERSION .7\n"
            b"FIELDS x y z intensity\n"
            b"SIZE 4 4 4 4\n"
            b"TYPE F F F F\n"
            b"COUNT 1 1 1 1\n"
            b"WIDTH 4\n"
            b"HEIGHT 1\n"
            b"POINTS 4\n"
            b"DATA binary\n"
            b"\x00\xff\n\x80"
        )
        (self.map_dir / GROUND_MAP_NAME).write_bytes(b"ground map")
        (self.map_dir / SCAN_CONTEXT_DB_NAME).write_bytes(b"scan context")

    def test_validate_dataset_returns_complete_identity(self):
        identity = validate_dataset(self.bag_dir, self.map_dir)

        self.assertEqual(identity.duration_ns, 756378139407)
        self.assertEqual(identity.message_count, 46875)
        self.assertEqual(identity.topic_counts, {"/imu": 42617, "/lidar_points": 4258})
        self.assertEqual(identity.db3_bytes, len(b"sample bag data"))
        self.assertEqual(identity.map_points, 4)
        self.assertEqual(
            dict(identity.map_artifacts),
            {
                GLOBAL_MAP_NAME: hashlib.sha256(
                    (self.map_dir / GLOBAL_MAP_NAME).read_bytes()).hexdigest(),
                GROUND_MAP_NAME: hashlib.sha256(b"ground map").hexdigest(),
                SCAN_CONTEXT_DB_NAME: hashlib.sha256(
                    b"scan context").hexdigest(),
            },
        )
        with self.assertRaises(TypeError):
            identity.map_artifacts[GLOBAL_MAP_NAME] = "0" * 64

    def test_validate_dataset_accepts_binary_pcd_header_followed_by_payload(self):
        global_map = self.map_dir / GLOBAL_MAP_NAME
        payload = b"\xff\x00\x80" * 1024
        global_map.write_bytes(b"VERSION .7\nPOINTS 7\nDATA binary\n" + payload)

        identity = validate_dataset(self.bag_dir, self.map_dir)

        self.assertEqual(identity.map_points, 7)
        self.assertEqual(
            identity.map_artifacts[GLOBAL_MAP_NAME],
            hashlib.sha256(global_map.read_bytes()).hexdigest())

    def test_validate_dataset_rejects_pcd_data_without_encoding(self):
        (self.map_dir / GLOBAL_MAP_NAME).write_bytes(b"POINTS 4\nDATA\n")

        with self.assertRaisesRegex(DatasetError, "DATA"):
            validate_dataset(self.bag_dir, self.map_dir)

    def test_validate_dataset_rejects_oversized_or_unterminated_pcd_header_line(self):
        header_limit = 64 * 1024
        prefix = b"POINTS 4\n"
        data_line_without_newline = b"DATA binary"
        comment_line = (
            b"#"
            + b"x" * (header_limit - len(prefix) - len(data_line_without_newline) - 2)
            + b"\n"
        )
        invalid_headers = {
            "oversized": prefix + b"#" + b"x" * header_limit,
            "unterminated at limit": prefix + comment_line + data_line_without_newline,
        }

        for name, header in invalid_headers.items():
            with self.subTest(name=name):
                (self.map_dir / GLOBAL_MAP_NAME).write_bytes(header)
                with self.assertRaises(DatasetError):
                    validate_dataset(self.bag_dir, self.map_dir)

    def test_validate_dataset_exposes_immutable_topic_counts(self):
        identity = validate_dataset(self.bag_dir, self.map_dir)

        self.assertEqual(identity.topic_counts["/imu"], 42617)
        self.assertEqual(identity.topic_counts, {"/imu": 42617, "/lidar_points": 4258})
        with self.assertRaises(TypeError):
            identity.topic_counts["/imu"] = 0

    def test_map_identity_distinguishes_same_size_artifacts(self):
        before = validate_dataset(self.bag_dir, self.map_dir)
        (self.map_dir / GROUND_MAP_NAME).write_bytes(b"6round map")

        after = validate_dataset(self.bag_dir, self.map_dir)

        self.assertEqual((self.map_dir / GROUND_MAP_NAME).stat().st_size,
                         len(b"ground map"))
        self.assertNotEqual(before.map_artifacts[GROUND_MAP_NAME],
                            after.map_artifacts[GROUND_MAP_NAME])

    def test_map_identity_includes_the_tiled_manifest(self):
        tiled = self.map_dir / "tiled"
        tiled.mkdir()
        manifest = b"schema_version: 1\n"
        (tiled / "manifest.yaml").write_bytes(manifest)

        identity = validate_dataset(
            self.bag_dir, self.map_dir, tiled_dir=tiled)

        self.assertEqual(identity.map_artifacts["tiled/manifest.yaml"],
                         hashlib.sha256(manifest).hexdigest())
        self.assertEqual(
            set(identity.map_artifacts),
            {GLOBAL_MAP_NAME, GROUND_MAP_NAME, SCAN_CONTEXT_DB_NAME,
             "tiled/manifest.yaml"},
        )

    def test_map_identity_requires_every_artifact(self):
        for name in (GLOBAL_MAP_NAME, GROUND_MAP_NAME, SCAN_CONTEXT_DB_NAME):
            with self.subTest(name=name):
                backup = (self.map_dir / name).read_bytes()
                (self.map_dir / name).unlink()
                try:
                    with self.assertRaisesRegex(DatasetError,
                                                name.replace(".", "\\.")):
                        validate_dataset(self.bag_dir, self.map_dir)
                finally:
                    (self.map_dir / name).write_bytes(backup)

    def test_map_identity_requires_the_tiled_manifest(self):
        tiled = self.map_dir / "tiled"
        tiled.mkdir()

        with self.assertRaisesRegex(DatasetError, "manifest\\.yaml"):
            validate_dataset(self.bag_dir, self.map_dir, tiled_dir=tiled)

    def test_map_identity_fails_closed_on_an_unreadable_artifact(self):
        real_open = Path.open

        def unreadable(path, *args, **kwargs):
            if path.name == GROUND_MAP_NAME:
                raise PermissionError("permission denied")
            return real_open(path, *args, **kwargs)

        with patch.object(Path, "open", unreadable):
            with self.assertRaisesRegex(DatasetError,
                                        "Unable to hash map artifact"):
                validate_dataset(self.bag_dir, self.map_dir)

    def test_map_identity_fails_closed_on_an_unreadable_tiled_manifest(self):
        tiled = self.map_dir / "tiled"
        tiled.mkdir()
        (tiled / "manifest.yaml").write_bytes(b"schema_version: 1\n")
        real_open = Path.open

        def unreadable(path, *args, **kwargs):
            if path.name == "manifest.yaml":
                raise PermissionError("permission denied")
            return real_open(path, *args, **kwargs)

        with patch.object(Path, "open", unreadable):
            with self.assertRaisesRegex(DatasetError,
                                        "Unable to hash map artifact"):
                validate_dataset(self.bag_dir, self.map_dir, tiled_dir=tiled)

    def test_map_identity_is_stable_across_a_fixture_map(self):
        # scripts/run_localization_stability.sh builds each fault case root by
        # symlinking the real map artifacts and copying the tiled map, then
        # loads the reference captured against the source map. The identity
        # must therefore follow content, not the local path, and must ignore
        # the fixture marker.
        tiled = self.map_dir / "tiled"
        tiled.mkdir()
        (tiled / "manifest.yaml").write_bytes(b"schema_version: 1\n")
        case_root = self.root / "missing-neighbor-map"
        case_root.mkdir()
        for name in (GLOBAL_MAP_NAME, GROUND_MAP_NAME, SCAN_CONTEXT_DB_NAME):
            (case_root / name).symlink_to(self.map_dir / name)
        shutil.copytree(tiled, case_root / "tiled")
        (case_root / "tiled" / ".p22-fault-fixture.json").write_text(
            '{"fault_case": "missing-neighbor"}', encoding="utf-8")

        source = validate_dataset(self.bag_dir, self.map_dir, tiled_dir=tiled)
        fixture = validate_dataset(self.bag_dir, case_root,
                                   tiled_dir=case_root / "tiled")

        self.assertEqual(dict(source.map_artifacts),
                         dict(fixture.map_artifacts))

    def test_validate_dataset_rejects_incomplete_map(self):
        (self.map_dir / GROUND_MAP_NAME).unlink()

        with self.assertRaisesRegex(DatasetError, "GroundMap\\.pcd"):
            validate_dataset(self.bag_dir, self.map_dir)

    def test_write_runtime_parameters_replaces_all_map_paths(self):
        template = self.root / "relocalization.yaml"
        template.write_text(
            """map_path: old-map
ground_map_path: old-ground
sc_db_path: old-db
""",
            encoding="utf-8",
        )
        output_dir = self.root / "runtime"

        runtime_path = write_runtime_parameters(template, self.map_dir, output_dir)
        runtime_text = runtime_path.read_text(encoding="utf-8")

        self.assertIn(f'map_path: "{(self.map_dir / GLOBAL_MAP_NAME).resolve()}"', runtime_text)
        self.assertIn(
            f'ground_map_path: "{(self.map_dir / GROUND_MAP_NAME).resolve()}"',
            runtime_text,
        )
        self.assertIn(f'sc_db_path: "{(self.map_dir / SCAN_CONTEXT_DB_NAME).resolve()}"', runtime_text)

    def test_write_runtime_parameters_quotes_special_map_paths_safely(self):
        special_map_dir = self.root / 'map"and\\backslash'
        special_map_dir.mkdir()
        template = self.root / "relocalization.yaml"
        template.write_text(
            """map_path: old-map
ground_map_path: old-ground
sc_db_path: old-db
""",
            encoding="utf-8",
        )

        runtime_path = write_runtime_parameters(
            template, special_map_dir, self.root / "runtime"
        )
        parameters = yaml.safe_load(runtime_path.read_text(encoding="utf-8"))

        self.assertEqual(
            parameters["map_path"], str((special_map_dir / GLOBAL_MAP_NAME).resolve())
        )
        self.assertEqual(
            parameters["ground_map_path"],
            str((special_map_dir / GROUND_MAP_NAME).resolve()),
        )
        self.assertEqual(
            parameters["sc_db_path"], str((special_map_dir / SCAN_CONTEXT_DB_NAME).resolve())
        )

    def test_write_runtime_parameters_preserves_inline_comments(self):
        template = self.root / "relocalization.yaml"
        template.write_text(
            """      map_path: old-map      # 全局地图
      sc_db_path: old-db       # 空值表示与 GlobalMap.pcd 同目录
      ground_map_path: old-ground  # 空=与map_path同目录 地面高度约束
""",
            encoding="utf-8",
        )

        runtime_path = write_runtime_parameters(template, self.map_dir, self.root / "runtime")
        runtime_text = runtime_path.read_text(encoding="utf-8")

        self.assertIn(
            f'      map_path: "{(self.map_dir / GLOBAL_MAP_NAME).resolve()}"      # 全局地图',
            runtime_text,
        )
        self.assertIn(
            f'      sc_db_path: "{(self.map_dir / SCAN_CONTEXT_DB_NAME).resolve()}"'
            "       # 空值表示与 GlobalMap.pcd 同目录",
            runtime_text,
        )
        self.assertIn(
            f'      ground_map_path: "{(self.map_dir / GROUND_MAP_NAME).resolve()}"'
            "  # 空=与map_path同目录 地面高度约束",
            runtime_text,
        )

    def test_write_runtime_parameters_keeps_hashes_inside_quoted_values(self):
        template = self.root / "relocalization.yaml"
        template.write_text(
            """map_path: "old # literal"
ground_map_path: old-ground  # trailing comment
sc_db_path: old-db
""",
            encoding="utf-8",
        )

        runtime_path = write_runtime_parameters(template, self.map_dir, self.root / "runtime")
        runtime_text = runtime_path.read_text(encoding="utf-8")
        parameters = yaml.safe_load(runtime_text)

        self.assertEqual(
            parameters["map_path"], str((self.map_dir / GLOBAL_MAP_NAME).resolve())
        )
        self.assertEqual(
            parameters["ground_map_path"],
            str((self.map_dir / GROUND_MAP_NAME).resolve()),
        )
        self.assertEqual(
            parameters["sc_db_path"], str((self.map_dir / SCAN_CONTEXT_DB_NAME).resolve())
        )
        self.assertNotIn("# literal", runtime_text)
        self.assertIn("  # trailing comment", runtime_text)

    def test_write_runtime_parameters_rejects_duplicate_key(self):
        template = self.root / "relocalization.yaml"
        template.write_text(
            """map_path: old-map
map_path: duplicate-map
ground_map_path: old-ground
sc_db_path: old-db
""",
            encoding="utf-8",
        )

        with self.assertRaisesRegex(DatasetError, "exactly once"):
            write_runtime_parameters(template, self.map_dir, self.root / "runtime")

    def test_validate_dataset_rejects_missing_db3(self):
        (self.bag_dir / "sample.db3").unlink()

        with self.assertRaisesRegex(DatasetError, "\\.db3"):
            validate_dataset(self.bag_dir, self.map_dir)

    def test_terminate_process_group_interrupts_or_ignores_dead_group(self):
        with patch("stability_common.os.killpg") as killpg:
            terminate_process_group(1234)

        killpg.assert_called_once_with(1234, signal.SIGINT)

        with patch("stability_common.os.killpg", side_effect=ProcessLookupError):
            terminate_process_group(1234)


if __name__ == "__main__":
    unittest.main()
