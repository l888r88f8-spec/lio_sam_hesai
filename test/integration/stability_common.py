from collections.abc import Mapping
from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import signal
import shutil
from types import MappingProxyType

import yaml


class DatasetError(RuntimeError):
    pass


@dataclass(frozen=True)
class DatasetIdentity:
    duration_ns: int
    message_count: int
    topic_counts: Mapping[str, int]
    db3_bytes: int
    map_points: int
    map_bytes: int


_REQUIRED_MAP_FILES = ("GlobalMap.pcd", "GroundMap.pcd", "sc_db.bin")
_RUNTIME_PARAMETER_LINE = re.compile(
    r"^(?P<indent>\s*)(?P<key>map_path|map_mode|ground_map_path|sc_db_path)\s*:"
    r"(?P<value>.*)$"
)
_SENSOR_TOPIC_HEADER_LINE = re.compile(r"^(?P<indent>\s*)sensor_topic\s*:(?P<value>.*)$")
_YAML_SCALAR_LINE = re.compile(r"^(?P<indent>\s*)(?P<key>[^\s:#][^:#]*)\s*:(?P<value>.*)$")
_PCD_HEADER_LIMIT_BYTES = 64 * 1024
_PCD_DATA_ENCODINGS = frozenset((b"ascii", b"binary", b"binary_compressed"))


def validate_dataset(bag_dir: Path, map_dir: Path, *, tiled_dir=None) -> DatasetIdentity:
    bag_path = Path(bag_dir)
    map_path = Path(map_dir)
    _require_directory(bag_path, "Bag")
    _require_directory(map_path, "Map")
    if tiled_dir is not None:
        tiled_path = Path(tiled_dir)
        _require_directory(tiled_path, "Tiled map")
        resolved_map = map_path.resolve()
        resolved_tiled = tiled_path.resolve()
        if not resolved_tiled.is_relative_to(resolved_map):
            raise DatasetError(
                f"Tiled map directory must be inside selected map root: {resolved_tiled}"
            )
        manifest = resolved_tiled / "manifest.yaml"
        if not manifest.is_file():
            raise DatasetError(f"Tiled map manifest.yaml is missing: {manifest}")

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
        topic_counts=MappingProxyType(topic_counts),
        db3_bytes=db3_bytes,
        map_points=_pcd_points(global_map),
        map_bytes=map_bytes,
    )


def write_runtime_parameters(
    template, map_dir, output_dir, topic_overrides=None, *, tiled_dir=None
):
    template_path = Path(template)
    if not template_path.is_file():
        raise DatasetError(f"Runtime parameter template is missing: {template_path}")

    try:
        template_text = template_path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise DatasetError(f"Unable to read runtime parameter template {template_path}: {error}") from error

    map_path = Path(map_dir).resolve()
    replacements = {
        "map_path": Path(tiled_dir).resolve() if tiled_dir is not None else map_path / "GlobalMap.pcd",
        "ground_map_path": map_path / "GroundMap.pcd",
        "sc_db_path": map_path / "sc_db.bin",
    }
    if tiled_dir is not None:
        replacements["map_mode"] = "tiled"
    topic_overrides = _validate_topic_overrides(topic_overrides)
    replacement_counts = {key: 0 for key in replacements}
    topic_override_counts = {key: 0 for key in topic_overrides}
    sensor_topic_indent = None
    rendered_lines = []

    for line in template_text.splitlines(keepends=True):
        content = line.rstrip("\r\n")
        line_ending = line[len(content) :]

        sensor_topic_header = _SENSOR_TOPIC_HEADER_LINE.fullmatch(content)
        if sensor_topic_header is not None:
            sensor_topic_indent = len(sensor_topic_header.group("indent"))
            rendered_lines.append(line)
            continue

        scalar_line = _YAML_SCALAR_LINE.fullmatch(content)
        if sensor_topic_indent is not None and scalar_line is not None:
            indentation = len(scalar_line.group("indent"))
            if indentation <= sensor_topic_indent:
                sensor_topic_indent = None
            elif scalar_line.group("key").strip() in topic_overrides:
                key = scalar_line.group("key").strip()
                topic_override_counts[key] += 1
                _, comment = _split_inline_yaml_comment(scalar_line.group("value"))
                rendered_lines.append(
                    f"{scalar_line.group('indent')}{key}: "
                    f"{json.dumps(topic_overrides[key])}{comment}{line_ending}"
                )
                continue

        match = _RUNTIME_PARAMETER_LINE.fullmatch(content)
        if match is None:
            rendered_lines.append(line)
            continue

        key = match.group("key")
        if key not in replacements:
            rendered_lines.append(line)
            continue
        replacement_counts[key] += 1
        _, comment = _split_inline_yaml_comment(match.group("value"))
        replacement = replacements[key]
        rendered_lines.append(
            f"{match.group('indent')}{key}: "
            f"{json.dumps(str(replacement))}{comment}{line_ending}"
        )

    invalid_keys = [key for key, count in replacement_counts.items() if count != 1]
    if invalid_keys:
        counts = ", ".join(f"{key}={replacement_counts[key]}" for key in replacements)
        raise DatasetError(
            f"Runtime parameter keys must each occur exactly once ({counts})"
        )

    invalid_topics = [key for key, count in topic_override_counts.items() if count != 1]
    if invalid_topics:
        counts = ", ".join(
            f"sensor_topic.{key}={topic_override_counts[key]}"
            for key in topic_override_counts
        )
        raise DatasetError(
            "Runtime parameter sensor_topic leaves must each occur exactly once "
            f"({counts})"
        )

    output_path = Path(output_dir) / "relocalization.runtime.yaml"
    try:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text("".join(rendered_lines), encoding="utf-8")
    except OSError as error:
        raise DatasetError(f"Unable to write runtime parameters {output_path}: {error}") from error
    return output_path


def _validate_topic_overrides(topic_overrides) -> dict[str, str]:
    if topic_overrides is None:
        return {}
    if not isinstance(topic_overrides, Mapping):
        raise DatasetError("topic_overrides must be a mapping")

    validated = {}
    for leaf, topic in topic_overrides.items():
        if not isinstance(leaf, str) or not leaf:
            raise DatasetError("topic_overrides keys must be non-empty strings")
        if not isinstance(topic, str) or not topic:
            raise DatasetError("topic_overrides values must be non-empty strings")
        validated[leaf] = topic
    return validated


def terminate_process_group(pid, timeout_seconds=10.0):
    del timeout_seconds
    try:
        os.killpg(pid, signal.SIGINT)
    except ProcessLookupError:
        pass


def prepare_tiled_fault_case(source, destination, fault_case, center_index=None):
    """Create a small, self-contained manifest fixture for replay fault checks."""
    source_path = Path(source).resolve()
    destination_path = Path(destination)
    manifest_path = source_path / "manifest.yaml"
    try:
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise DatasetError(f"Unable to read tiled manifest {manifest_path}: {error}") from error
    manifest = _require_mapping(manifest, "tiled manifest")
    global_tiles = _manifest_tiles(manifest, "global")
    ground_tiles = _manifest_tiles(manifest, "ground")
    if not global_tiles:
        raise DatasetError("Tiled manifest has no global tiles")
    if center_index is None:
        center = (0, 0)
    else:
        center = tuple(int(value) for value in center_index)
        if len(center) != 2:
            raise DatasetError("Tiled fault center index must contain two integers")
    selected = {
        (center[0] + dx, center[1] + dy)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
    }
    if fault_case != "missing-center" and not any(
        tuple(entry["index"]) == center for entry in global_tiles
    ):
        raise DatasetError(
            "Tiled fault center tile is absent; set LIO_SAM_TILED_FAULT_CENTER_INDEX"
        )
    neighbor = (center[0] + 1, center[1])
    missing = center if fault_case == "missing-center" else neighbor
    if fault_case not in ("corrupt-manifest", "missing-center", "missing-neighbor"):
        raise DatasetError(f"Unknown tiled fault case: {fault_case}")

    destination_path.mkdir(parents=True, exist_ok=False)
    if fault_case == "corrupt-manifest":
        (destination_path / "manifest.yaml").write_text("schema_version: [\n", encoding="utf-8")
        return f"[{missing[0]},{missing[1]}]"

    retained = {}
    for kind, entries in (("global", global_tiles), ("ground", ground_tiles)):
        retained[kind] = [
            entry for entry in entries
            if tuple(entry["index"]) in selected and tuple(entry["index"]) != missing
        ]
        for entry in retained[kind]:
            _copy_manifest_relative_file(source_path, destination_path, entry["path"])
    manifest["tiles"]["global"] = retained["global"]
    manifest["tiles"]["ground"] = retained["ground"]
    overview = manifest.get("overview")
    if isinstance(overview, Mapping) and isinstance(overview.get("path"), str):
        _copy_manifest_relative_file(source_path, destination_path, overview["path"])
    (destination_path / "manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8"
    )
    return f"[{missing[0]},{missing[1]}]"


def _manifest_tiles(manifest, kind):
    tiles = _require_mapping(manifest.get("tiles"), "tiled manifest tiles")
    entries = tiles.get(kind)
    if not isinstance(entries, list):
        raise DatasetError(f"Tiled manifest tiles.{kind} must be a list")
    for entry in entries:
        if (not isinstance(entry, Mapping) or not isinstance(entry.get("index"), list)
                or len(entry["index"]) != 2 or not isinstance(entry.get("path"), str)):
            raise DatasetError(f"Invalid tiled manifest tiles.{kind} entry")
    return entries


def _copy_manifest_relative_file(source_root, destination_root, relative):
    relative_path = Path(relative)
    source = (source_root / relative_path).resolve()
    if relative_path.is_absolute() or not source.is_relative_to(source_root):
        raise DatasetError(f"Manifest path escapes tiled map root: {relative}")
    destination = destination_root / relative_path
    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        shutil.copy2(source, destination)
    except OSError as error:
        raise DatasetError(f"Unable to copy tiled fixture file {relative}: {error}") from error


def _split_inline_yaml_comment(value_tail: str) -> tuple[str, str]:
    in_single_quote = False
    in_double_quote = False
    index = 0

    while index < len(value_tail):
        character = value_tail[index]
        if in_double_quote:
            if character == "\\":
                index += 2
                continue
            if character == '"':
                in_double_quote = False
        elif in_single_quote:
            if character == "'":
                if index + 1 < len(value_tail) and value_tail[index + 1] == "'":
                    index += 2
                    continue
                in_single_quote = False
        elif character == "'":
            in_single_quote = True
        elif character == '"':
            in_double_quote = True
        elif character == "#" and (index == 0 or value_tail[index - 1].isspace()):
            comment_start = index
            while comment_start > 0 and value_tail[comment_start - 1].isspace():
                comment_start -= 1
            return value_tail[:comment_start], value_tail[comment_start:]
        index += 1

    return value_tail, ""


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
                raw_line = pcd_file.readline(remaining + 1)
                if not raw_line:
                    break
                if not raw_line.endswith(b"\n"):
                    raise DatasetError(
                        f"PCD header line exceeds the size limit or is not newline-terminated: "
                        f"{pcd_path}"
                    )
                header_bytes += len(raw_line)
                if header_bytes > _PCD_HEADER_LIMIT_BYTES:
                    raise DatasetError(f"PCD header exceeds the size limit: {pcd_path}")
                line = raw_line.strip()
                if not line or line.startswith(b"#"):
                    continue
                fields = line.split()
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
                    if len(fields) != 2 or fields[1] not in _PCD_DATA_ENCODINGS:
                        raise DatasetError(f"PCD DATA header is invalid: {pcd_path}")
                    if points is None:
                        raise DatasetError(f"PCD POINTS header is missing: {pcd_path}")
                    return points
    except OSError as error:
        raise DatasetError(f"Unable to read PCD header {pcd_path}: {error}") from error

    raise DatasetError(f"PCD DATA header is missing or too large: {pcd_path}")
