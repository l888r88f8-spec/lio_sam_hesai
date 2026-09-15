from collections.abc import Mapping
from dataclasses import dataclass
import hashlib
import json
import math
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
    template, map_dir, output_dir, topic_overrides=None, *, tiled_dir=None,
    auto_relocalization=None
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
    # A fault replay must not let Scan Context pick its own tile: the whole
    # point is to observe the request the reference pose asks for.
    forced_scalars = {}
    if auto_relocalization is not None:
        forced_scalars["auto_relocalization"] = bool(auto_relocalization)
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
        if (scalar_line is not None and
                scalar_line.group("key").strip() in forced_scalars):
            key = scalar_line.group("key").strip()
            _, comment = _split_inline_yaml_comment(scalar_line.group("value"))
            rendered_lines.append(
                f"{scalar_line.group('indent')}{key}: "
                f"{json.dumps(forced_scalars[key])}{comment}{line_ending}"
            )
            continue
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


def _fault_index(value, name):
    """Return a tile index element, rejecting bool and float coercion."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise DatasetError(f"{name} must be an integer, not {value!r}")
    return value


def _tile_index(entry, kind):
    index = entry["index"]
    if not isinstance(index, list) or len(index) != 2:
        raise DatasetError(f"Tiled manifest tiles.{kind} index must have two entries")
    return (_fault_index(index[0], f"tiles.{kind} index[0]"),
            _fault_index(index[1], f"tiles.{kind} index[1]"))


def _declared_tile_path(root, relative, kind):
    """
    Resolve a manifest tile path under `root`, refusing every escape.

    Used for both the source map and the fixture, because the two can disagree:
    a symlinked directory in the source absorbs one `..` level, so the same
    relative path that stays inside the source can climb out of the fixture.
    """
    if not isinstance(relative, str) or not relative:
        raise DatasetError(f"Tiled manifest tiles.{kind} path must be a string")
    relative_path = Path(relative)
    if relative_path.is_absolute():
        raise DatasetError(f"Tiled manifest tiles.{kind} path must be relative: {relative}")
    # A generated map bundle never needs `..`; counting it is what made the
    # source and the fixture disagree about depth.
    if ".." in relative_path.parts:
        raise DatasetError(
            f"Tiled manifest tiles.{kind} path must not contain '..': {relative}")
    # Every component, not just the leaf: a symlinked parent redirects the
    # whole subtree while the leaf still looks ordinary.
    current = root
    for part in relative_path.parts:
        current = current / part
        if current.is_symlink():
            raise DatasetError(
                f"Tiled manifest tiles.{kind} path must not cross a symlink: {relative}")
    candidate = root / relative_path
    if not candidate.resolve().is_relative_to(root.resolve()):
        raise DatasetError(
            f"Tiled manifest tiles.{kind} path escapes its root: {relative}")
    return candidate


def _fixture_destination(destination_root, relative, kind):
    """Resolve a copy destination inside the fixture, refusing an escape."""
    destination = destination_root / Path(relative)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not destination.resolve().is_relative_to(destination_root.resolve()):
        raise DatasetError(
            f"Tiled manifest tiles.{kind} path escapes the fixture: {relative}")
    return destination


# Written into every fixture by prepare_tiled_fault_case. inject() refuses to
# move anything without it, so a --tiled-dir pointed at a real map can never be
# mutated.
_FAULT_FIXTURE_MARKER = ".p22-fault-fixture.json"


@dataclass(frozen=True)
class TiledFaultFixture:
    """
    A self-contained copy of a tiled map with one identified faulty tile.

    Copying is complete and non-destructive: every manifest entry survives and
    every tile file is present, so the fixture loads exactly like the real map
    and the replay's trajectory stays inside its coverage. The fault is applied
    separately by :meth:`inject`, after the node has prepared its map, so that
    it surfaces as a runtime load failure instead of a manifest validation
    failure.
    """

    root: Path
    tiled_root: Path
    center: tuple
    fault_case: str
    fault_index: object          # None for corrupt-manifest
    fault_relative: object       # None for corrupt-manifest
    active_radius: int
    prefetch_radius: int

    @property
    def expected_index(self):
        """Return the bracket string the degraded diagnostic must contain."""
        if self.fault_index is None:
            return None
        return f"[{self.fault_index[0]},{self.fault_index[1]}]"

    def _verify_provenance(self):
        """
        Prove this tiled root was created as a fault fixture for this fault.

        Without this, pointing the CLI at a real map would let inject() rename a
        live tile -- or manifest.yaml -- into the quarantine.
        """
        marker = self.tiled_root / _FAULT_FIXTURE_MARKER
        if marker.is_symlink() or not marker.is_file():
            raise DatasetError(
                "refusing to inject: the tiled root has no fault-fixture marker, "
                "so it is not a fixture created by prepare_tiled_fault_case")
        try:
            payload = json.loads(marker.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            raise DatasetError(f"unreadable fault-fixture marker: {error}") from error
        payload = _require_mapping(payload, "fault fixture marker")
        if payload.get("fault_case") != self.fault_case:
            raise DatasetError(
                f"fault-fixture marker is for {payload.get('fault_case')!r}, "
                f"not {self.fault_case!r}")
        if payload.get("fault_relative") != self.fault_relative:
            raise DatasetError(
                "fault-fixture marker does not select this payload")
        if list(payload.get("fault_index") or []) != list(self.fault_index or []):
            raise DatasetError("fault-fixture marker does not match this index")
        return payload

    def _verify_declared_tile(self):
        """Return only when the payload is a tile the fixture manifest declares."""
        manifest_path = self.tiled_root / "manifest.yaml"
        try:
            manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, yaml.YAMLError) as error:
            raise DatasetError(f"unable to read fixture manifest: {error}") from error
        manifest = _require_mapping(manifest, "fixture manifest")
        for entry in _manifest_tiles(manifest, "global"):
            if entry["path"] == self.fault_relative:
                if list(_tile_index(entry, "global")) != list(self.fault_index or []):
                    raise DatasetError(
                        "fixture manifest maps the payload to a different index")
                return
        raise DatasetError(
            f"fixture manifest does not declare {self.fault_relative}")

    def inject(self):
        """Move the faulty global payload aside, leaving the manifest intact."""
        if self.fault_relative is None:
            raise DatasetError("corrupt-manifest has no payload to inject")
        self._verify_provenance()
        self._verify_declared_tile()
        target = _declared_tile_path(self.tiled_root, self.fault_relative, "global")
        if not target.is_file():
            raise DatasetError(f"fault payload is missing: {self.fault_relative}")
        quarantine = self.root / "quarantine"
        if quarantine.is_symlink():
            raise DatasetError("fault quarantine must not be a symlink")
        quarantine.mkdir(parents=True, exist_ok=True)
        if not quarantine.resolve().is_relative_to(self.root.resolve()):
            raise DatasetError("fault quarantine must live inside the fixture")
        destination = quarantine / Path(self.fault_relative).name
        # Renaming replaces silently, so a planted quarantine symlink pointing
        # at the real map could overwrite a same-named map file.
        if destination.exists() or destination.is_symlink():
            raise DatasetError(
                f"refusing to overwrite {destination.name} in the fault quarantine")
        try:
            target.rename(destination)
        except OSError as error:
            raise DatasetError(
                f"Unable to inject tiled fault {self.fault_relative}: {error}"
            ) from error
        return destination


def prepare_tiled_fault_case(source, destination, fault_case, center_index,
                             active_radius=1, prefetch_radius=1):
    """
    Copy a tiled map into a self-contained fixture for a replay fault check.

    `center_index` is required: the map's first tile says nothing about where
    the replay actually drives, and using it produced fixtures the trajectory
    never reached.
    """
    if fault_case not in ("corrupt-manifest", "missing-center", "missing-neighbor"):
        raise DatasetError(f"Unknown tiled fault case: {fault_case}")
    if center_index is None and fault_case != "corrupt-manifest":
        raise DatasetError(
            "Tiled fault center index is required; pass the replay reference tile"
        )
    if active_radius < 0 or prefetch_radius < active_radius:
        raise DatasetError("Tiled fault radii must satisfy 0 <= active <= prefetch")

    source_path = Path(source).resolve()
    destination_path = Path(destination)
    if destination_path.resolve().is_relative_to(source_path):
        raise DatasetError("Tiled fault fixture must not be created inside the source map")
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

    center = tuple(_fault_index(value, "fault center index") for value in center_index)
    if len(center) != 2:
        raise DatasetError("Tiled fault center index must contain two integers")
    declared = {_tile_index(entry, "global"): entry for entry in global_tiles}
    if center not in declared:
        raise DatasetError(f"Tiled fault center tile {center} is not declared")

    destination_path.mkdir(parents=True, exist_ok=False)

    if fault_case == "corrupt-manifest":
        (destination_path / "manifest.yaml").write_text(
            "schema_version: [\n", encoding="utf-8")
        return TiledFaultFixture(
            root=destination_path, tiled_root=destination_path, center=center,
            fault_case=fault_case, fault_index=None, fault_relative=None,
            active_radius=active_radius, prefetch_radius=prefetch_radius,
        )

    if fault_case == "missing-center":
        fault_index = center
    else:
        neighbourhood = sorted(
            (center[0] + dx, center[1] + dy)
            for dx in range(-active_radius, active_radius + 1)
            for dy in range(-active_radius, active_radius + 1)
        )
        neighbours = [index for index in neighbourhood
                      if index != center and index in declared]
        if not neighbours:
            raise DatasetError(
                f"Tiled fault centre {center} declares no neighbour tile to remove"
            )
        fault_index = neighbours[0]

    # Copy every declared payload, so the fixture covers exactly what the real
    # map covers and the replay cannot fall off its edge.
    for kind, entries in (("global", global_tiles), ("ground", ground_tiles)):
        for entry in entries:
            source_file = _declared_tile_path(source_path, entry["path"], kind)
            if not source_file.is_file():
                raise DatasetError(f"declared tile is missing: {entry['path']}")
            destination_file = _fixture_destination(
                destination_path, entry["path"], kind)
            try:
                shutil.copy2(source_file, destination_file)
            except OSError as error:
                raise DatasetError(
                    f"Unable to copy tiled fixture file {entry['path']}: {error}"
                ) from error
    overview = manifest.get("overview")
    if isinstance(overview, Mapping) and isinstance(overview.get("path"), str):
        overview_source = _declared_tile_path(source_path, overview["path"], "overview")
        if overview_source.is_file():
            shutil.copy2(overview_source, _fixture_destination(
                destination_path, overview["path"], "overview"))
    # Copied byte-for-byte, not re-serialised: the fixture is meant to be a
    # faithful copy of the map, and re-dumping changed the bytes, which made
    # the recorded map fingerprint (a hash of this file) differ between the
    # capture run and the fault run.
    shutil.copy2(manifest_path, destination_path / "manifest.yaml")
    (destination_path / _FAULT_FIXTURE_MARKER).write_text(
        json.dumps({
            "fault_case": fault_case,
            "fault_index": list(fault_index),
            "fault_relative": declared[fault_index]["path"],
            "center": list(center),
            "root": str(destination_path.resolve()),
        }, sort_keys=True),
        encoding="utf-8",
    )

    return TiledFaultFixture(
        root=destination_path, tiled_root=destination_path, center=center,
        fault_case=fault_case, fault_index=fault_index,
        fault_relative=declared[fault_index]["path"],
        active_radius=active_radius, prefetch_radius=prefetch_radius,
    )


def load_tiled_fault_fixture(tiled_root, active_radius=1, prefetch_radius=1):
    """
    Load a fixture previously created by prepare_tiled_fault_case.

    The replay CLI must not be able to *construct* a fixture from an arbitrary
    directory: that would let a mistyped --tiled-dir move real map files.
    """
    tiled_root = Path(tiled_root)
    marker = tiled_root / _FAULT_FIXTURE_MARKER
    if marker.is_symlink() or not marker.is_file():
        raise DatasetError(
            f"{tiled_root} is not a fault fixture; run "
            "prepare_tiled_fault_case first")
    try:
        payload = json.loads(marker.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise DatasetError(f"unreadable fault-fixture marker: {error}") from error
    payload = _require_mapping(payload, "fault fixture marker")
    return TiledFaultFixture(
        root=tiled_root,
        tiled_root=tiled_root,
        center=tuple(payload.get("center") or ()),
        fault_case=payload.get("fault_case"),
        fault_index=tuple(payload.get("fault_index") or ()),
        fault_relative=payload.get("fault_relative"),
        active_radius=active_radius,
        prefetch_radius=prefetch_radius,
    )


def _manifest_tiles(manifest, kind):
    tiles = _require_mapping(manifest.get("tiles"), "tiled manifest tiles")
    entries = tiles.get(kind)
    if not isinstance(entries, list):
        raise DatasetError(f"Tiled manifest tiles.{kind} must be a list")
    for entry in entries:
        if (not isinstance(entry, Mapping) or not isinstance(entry.get("index"), list)
                or len(entry["index"]) != 2 or not isinstance(entry.get("path"), str)):
            raise DatasetError(f"Invalid tiled manifest tiles.{kind} entry")
        _tile_index(entry, kind)
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


# --- Tiled fault reference ---------------------------------------------------
#
# A fault replay needs to know where the vehicle actually is, in map tile
# terms. Deriving that from the map's first tile says nothing about the
# trajectory, so the index is computed from a pose captured during a no-fault
# replay. The maths below is pure Python so the CI job that runs these tests
# without ROS can still exercise it.

_TILE_MATRIX_ROWS = 4
_TILE_INDEX_SCHEMA_VERSION = 1


def _identity_matrix():
    return [[1.0 if row == column else 0.0 for column in range(_TILE_MATRIX_ROWS)]
            for row in range(_TILE_MATRIX_ROWS)]


def _require_matrix4(value, name):
    if (not isinstance(value, (list, tuple)) or len(value) != _TILE_MATRIX_ROWS):
        raise DatasetError(f"{name} must be a 4x4 matrix")
    rows = []
    for row in value:
        if not isinstance(row, (list, tuple)) or len(row) != _TILE_MATRIX_ROWS:
            raise DatasetError(f"{name} must be a 4x4 matrix")
        rows.append([float(entry) for entry in row])
    return rows


def _matmul4(lhs, rhs):
    return [[sum(lhs[row][k] * rhs[k][column] for k in range(_TILE_MATRIX_ROWS))
             for column in range(_TILE_MATRIX_ROWS)]
            for row in range(_TILE_MATRIX_ROWS)]


def tile_index_for_pose(map_T_base, base_T_lidar, origin_xy, tile_size_m):
    """
    Return the tile index holding the lidar for a given base pose.

    ``map_T_lidar = map_T_base * base_T_lidar``, so the base-to-lidar lever arm
    is rotated by the base orientation before it is applied -- adding a fixed
    offset in map axes is only correct when the base is unrotated.
    """
    map_T_base = _require_matrix4(map_T_base, "map_T_base")
    base_T_lidar = _require_matrix4(base_T_lidar, "base_T_lidar")
    if tile_size_m <= 0.0:
        raise DatasetError("tile_size_m must be positive")
    if len(origin_xy) != 2:
        raise DatasetError("origin_xy must have two entries")

    map_T_lidar = _matmul4(map_T_base, base_T_lidar)
    lidar_x = map_T_lidar[0][3]
    lidar_y = map_T_lidar[1][3]
    return (int(math.floor((lidar_x - float(origin_xy[0])) / tile_size_m)),
            int(math.floor((lidar_y - float(origin_xy[1])) / tile_size_m)))


def reference_base_to_lidar(translation_xyz, rpy_radians=(0.0, 0.0, 0.0)):
    """Build base_T_lidar from a translation and fixed-axis roll/pitch/yaw."""
    roll, pitch, yaw = (float(value) for value in rpy_radians)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    tx, ty, tz = (float(value) for value in translation_xyz)
    return [
        [rotation[0][0], rotation[0][1], rotation[0][2], tx],
        [rotation[1][0], rotation[1][1], rotation[1][2], ty],
        [rotation[2][0], rotation[2][1], rotation[2][2], tz],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _dataset_identity_payload(identity):
    return {
        "duration_ns": int(identity.duration_ns),
        "message_count": int(identity.message_count),
        "topic_counts": {str(k): int(v)
                         for k, v in dict(identity.topic_counts).items()},
        "db3_bytes": int(identity.db3_bytes),
        "map_points": int(identity.map_points),
        "map_bytes": int(identity.map_bytes),
    }


def map_fingerprint(map_root, tiled_dir=None):
    """
    Fingerprint the map artifacts a reference pose is only valid against.

    `DatasetIdentity` covers GlobalMap.pcd; a pose measured on one tiled map is
    equally meaningless on another, and on a different ground map or Scan
    Context database.
    """
    map_root = Path(map_root)

    def size_of(path):
        try:
            return int(Path(path).stat().st_size)
        except OSError:
            return None

    digest = None
    if tiled_dir is not None:
        manifest = Path(tiled_dir) / "manifest.yaml"
        try:
            digest = hashlib.sha256(manifest.read_bytes()).hexdigest()
        except OSError:
            digest = None
    return {
        "ground_map_bytes": size_of(map_root / "GroundMap.pcd"),
        "sc_db_bytes": size_of(map_root / "sc_db.bin"),
        "tiled_manifest_sha256": digest,
    }


@dataclass(frozen=True)
class TiledFaultReference:
    """
    Where the vehicle actually is, captured from a no-fault replay.

    Bound to a dataset identity so a pose recorded against one bag or map can
    never be replayed against another: tile indices are only meaningful inside
    the map they were measured in.
    """

    dataset: Mapping
    bag_seconds: float
    map_T_base: tuple
    offset_seconds: float
    warmup_seconds: float


def make_tiled_fault_reference(identity, bag_seconds, map_T_base,
                               warmup_seconds=10.0, map_fingerprint_value=None):
    map_T_base = _require_matrix4(map_T_base, "map_T_base")
    if not math.isfinite(bag_seconds) or bag_seconds < 0.0:
        raise DatasetError("reference bag time must be finite and non-negative")
    if warmup_seconds <= 0.0:
        raise DatasetError("reference warmup must be positive")
    offset = max(0.0, bag_seconds - warmup_seconds)
    payload = _dataset_identity_payload(identity)
    if map_fingerprint_value is not None:
        payload.update(dict(map_fingerprint_value))
    return TiledFaultReference(
        dataset=payload,
        bag_seconds=float(bag_seconds),
        map_T_base=tuple(tuple(row) for row in map_T_base),
        offset_seconds=offset,
        warmup_seconds=float(warmup_seconds),
    )


def save_tiled_fault_reference(path, reference):
    payload = {
        "schema_version": _TILE_INDEX_SCHEMA_VERSION,
        "dataset": dict(reference.dataset),
        "bag_seconds": reference.bag_seconds,
        "map_T_base": [list(row) for row in reference.map_T_base],
        "offset_seconds": reference.offset_seconds,
        "warmup_seconds": reference.warmup_seconds,
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    return path


def load_tiled_fault_reference(path, identity, map_fingerprint_value=None):
    path = Path(path)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise DatasetError(f"Unable to read tiled fault reference {path}: {error}") from error
    payload = _require_mapping(payload, "tiled fault reference")
    if payload.get("schema_version") != _TILE_INDEX_SCHEMA_VERSION:
        raise DatasetError(
            f"Unsupported tiled fault reference schema: {payload.get('schema_version')!r}")

    recorded = _require_mapping(payload.get("dataset"), "reference dataset")
    expected = _dataset_identity_payload(identity)
    if map_fingerprint_value is not None:
        expected = dict(expected)
        for key, value in dict(map_fingerprint_value).items():
            if recorded.get(key) != value:
                raise DatasetError(
                    f"Tiled fault reference was recorded against a different map "
                    f"({key}: {recorded.get(key)!r} != {value!r}); recapture it")
    for key, value in expected.items():
        if recorded.get(key) != value:
            raise DatasetError(
                f"Tiled fault reference was recorded against a different dataset "
                f"({key}: {recorded.get(key)!r} != {value!r}); recapture it")
    return TiledFaultReference(
        dataset=recorded,
        bag_seconds=float(payload["bag_seconds"]),
        map_T_base=tuple(tuple(row) for row in
                         _require_matrix4(payload["map_T_base"], "map_T_base")),
        offset_seconds=float(payload.get("offset_seconds", 0.0)),
        warmup_seconds=float(payload.get("warmup_seconds", 10.0)),
    )
