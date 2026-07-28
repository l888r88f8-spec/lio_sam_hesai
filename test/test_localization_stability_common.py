#!/usr/bin/env python3
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
        (self.map_dir / "GlobalMap.pcd").write_bytes(
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
        (self.map_dir / "GroundMap.pcd").write_bytes(b"ground map")
        (self.map_dir / "sc_db.bin").write_bytes(b"scan context")

    def test_validate_dataset_returns_complete_identity(self):
        identity = validate_dataset(self.bag_dir, self.map_dir)

        self.assertEqual(identity.duration_ns, 756378139407)
        self.assertEqual(identity.message_count, 46875)
        self.assertEqual(identity.topic_counts, {"/imu": 42617, "/lidar_points": 4258})
        self.assertEqual(identity.db3_bytes, len(b"sample bag data"))
        self.assertEqual(identity.map_points, 4)
        self.assertEqual(identity.map_bytes, (self.map_dir / "GlobalMap.pcd").stat().st_size)

    def test_validate_dataset_rejects_incomplete_map(self):
        (self.map_dir / "GroundMap.pcd").unlink()

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

        self.assertIn(f'map_path: "{(self.map_dir / "GlobalMap.pcd").resolve()}"', runtime_text)
        self.assertIn(
            f'ground_map_path: "{(self.map_dir / "GroundMap.pcd").resolve()}"',
            runtime_text,
        )
        self.assertIn(f'sc_db_path: "{(self.map_dir / "sc_db.bin").resolve()}"', runtime_text)

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
            parameters["map_path"], str((special_map_dir / "GlobalMap.pcd").resolve())
        )
        self.assertEqual(
            parameters["ground_map_path"],
            str((special_map_dir / "GroundMap.pcd").resolve()),
        )
        self.assertEqual(
            parameters["sc_db_path"], str((special_map_dir / "sc_db.bin").resolve())
        )

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
