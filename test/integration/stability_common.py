from dataclasses import dataclass
import os
from pathlib import Path
import re
import signal

import yaml


class DatasetError(RuntimeError):
    pass


@dataclass(frozen=True)
class DatasetIdentity:
    duration_ns: int
    message_count: int
    topic_counts: dict[str, int]
    db3_bytes: int
    map_points: int
    map_bytes: int


_REQUIRED_MAP_FILES = ("GlobalMap.pcd", "GroundMap.pcd", "sc_db.bin")
_RUNTIME_PARAMETER_LINE = re.compile(
    r"^(?P<indent>\s*)(?P<key>map_path|ground_map_path|sc_db_path)\s*:.*$"
)
_PCD_HEADER_LIMIT_BYTES = 64 * 1024


def validate_dataset(bag_dir: Path, map_dir: Path) -> DatasetIdentity:
    bag_path = Path(bag_dir)
    map_path = Path(map_dir)
    _require_directory(bag_path, "Bag")
    _require_directory(map_path, "Map")

    metadata = _load_metadata(bag_path / "metadata.yaml")
    bag_info = _require_mapping(
        metadata.get("rosbag2_bagfile_information"),
        "metadata rosbag2_bagfile_information",
    )
    duration = _require_mapping(bag_info.get("duration"), "metadata duration")
    duration_ns = _require_nonnegative_int(duration.get("nanoseconds"), "metadata duration nanoseconds")
    message_count = _require_nonnegative_int(
        bag_info.get("message_count"), "metadata message_count"
    )
    topic_counts = _topic_counts(bag_info.get("topics_with_message_count"))

    db3_files = sorted(path for path in bag_path.glob("*.db3") if path.is_file())
    if len(db3_files) != 1:
        raise DatasetError(
            f"Expected exactly one *.db3 file in {bag_path}, found {len(db3_files)}"
        )

    required_maps = _required_map_files(map_path)
    global_map = required_maps["GlobalMap.pcd"]
    try:
        db3_bytes = db3_files[0].stat().st_size
        map_bytes = global_map.stat().st_size
    except OSError as error:
        raise DatasetError(f"Unable to stat dataset file: {error}") from error

    return DatasetIdentity(
        duration_ns=duration_ns,
        message_count=message_count,
        topic_counts=topic_counts,
        db3_bytes=db3_bytes,
        map_points=_pcd_points(global_map),
        map_bytes=map_bytes,
    )


def write_runtime_parameters(template, map_dir, output_dir):
    template_path = Path(template)
    if not template_path.is_file():
        raise DatasetError(f"Runtime parameter template is missing: {template_path}")

    try:
        template_text = template_path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise DatasetError(f"Unable to read runtime parameter template {template_path}: {error}") from error

    map_path = Path(map_dir).resolve()
    replacements = {
        "map_path": map_path / "GlobalMap.pcd",
        "ground_map_path": map_path / "GroundMap.pcd",
        "sc_db_path": map_path / "sc_db.bin",
    }
    replacement_counts = {key: 0 for key in replacements}
    rendered_lines = []

    for line in template_text.splitlines(keepends=True):
        content = line.rstrip("\r\n")
        line_ending = line[len(content) :]
        match = _RUNTIME_PARAMETER_LINE.fullmatch(content)
        if match is None:
            rendered_lines.append(line)
            continue

        key = match.group("key")
        replacement_counts[key] += 1
        rendered_lines.append(
            f'{match.group("indent")}{key}: "{replacements[key]}"{line_ending}'
        )

    invalid_keys = [key for key, count in replacement_counts.items() if count != 1]
    if invalid_keys:
        counts = ", ".join(f"{key}={replacement_counts[key]}" for key in replacements)
        raise DatasetError(
            f"Runtime parameter keys must each occur exactly once ({counts})"
        )

    output_path = Path(output_dir) / "relocalization.runtime.yaml"
    try:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text("".join(rendered_lines), encoding="utf-8")
    except OSError as error:
        raise DatasetError(f"Unable to write runtime parameters {output_path}: {error}") from error
    return output_path


def terminate_process_group(pid, timeout_seconds=10.0):
    del timeout_seconds
    try:
        os.killpg(pid, signal.SIGINT)
    except ProcessLookupError:
        pass


def _require_directory(path: Path, label: str) -> None:
    if not path.is_dir():
        raise DatasetError(f"{label} directory is missing: {path}")


def _required_map_files(map_path: Path) -> dict[str, Path]:
    files = {name: map_path / name for name in _REQUIRED_MAP_FILES}
    for name, path in files.items():
        if not path.is_file():
            raise DatasetError(f"Required map file is missing: {name} ({path})")
    return files


def _load_metadata(metadata_path: Path) -> dict:
    if not metadata_path.is_file():
        raise DatasetError(f"Bag metadata is missing: {metadata_path}")
    try:
        with metadata_path.open(encoding="utf-8") as metadata_file:
            metadata = yaml.safe_load(metadata_file)
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise DatasetError(f"Unable to read bag metadata {metadata_path}: {error}") from error
    return _require_mapping(metadata, "bag metadata")


def _topic_counts(topics) -> dict[str, int]:
    if not isinstance(topics, list):
        raise DatasetError("metadata topics_with_message_count must be a list")

    counts = {}
    for index, topic in enumerate(topics):
        topic = _require_mapping(topic, f"metadata topic entry {index}")
        topic_metadata = _require_mapping(
            topic.get("topic_metadata"), f"metadata topic entry {index} topic_metadata"
        )
        name = topic_metadata.get("name")
        if not isinstance(name, str) or not name:
            raise DatasetError(f"metadata topic entry {index} has no valid name")
        if name in counts:
            raise DatasetError(f"metadata contains duplicate topic name: {name}")
        counts[name] = _require_nonnegative_int(
            topic.get("message_count"), f"metadata topic entry {index} message_count"
        )
    return counts


def _require_mapping(value, label: str) -> dict:
    if not isinstance(value, dict):
        raise DatasetError(f"{label} must be a mapping")
    return value


def _require_nonnegative_int(value, label: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise DatasetError(f"{label} must be a non-negative integer")
    return value


def _pcd_points(pcd_path: Path) -> int:
    points = None
    header_bytes = 0
    try:
        with pcd_path.open("rb") as pcd_file:
            while header_bytes < _PCD_HEADER_LIMIT_BYTES:
                remaining = _PCD_HEADER_LIMIT_BYTES - header_bytes
                raw_line = pcd_file.readline(remaining)
                if not raw_line:
                    break
                header_bytes += len(raw_line)
                line = raw_line.strip()
                if not line or line.startswith(b"#"):
                    continue
                fields = line.split(maxsplit=1)
                key = fields[0].upper()
                if key == b"POINTS":
                    if len(fields) != 2:
                        raise DatasetError(f"PCD POINTS header is invalid: {pcd_path}")
                    try:
                        points = int(fields[1])
                    except ValueError as error:
                        raise DatasetError(f"PCD POINTS header is invalid: {pcd_path}") from error
                    if points < 0:
                        raise DatasetError(f"PCD POINTS header is invalid: {pcd_path}")
                elif key == b"DATA":
                    if points is None:
                        raise DatasetError(f"PCD POINTS header is missing: {pcd_path}")
                    return points
    except OSError as error:
        raise DatasetError(f"Unable to read PCD header {pcd_path}: {error}") from error

    raise DatasetError(f"PCD DATA header is missing or too large: {pcd_path}")
