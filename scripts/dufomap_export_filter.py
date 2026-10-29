#!/usr/bin/env python3
"""DUFOMap-based dynamic-point removal for an exported LIO-SAM map.

The exporter writes a self-contained dataset:

    manifest.json
    scans/000000.pcd        # full deskewed scan, lidar frame, XYZI
    targets/corner_raw.pcd  # raw map points, odometry frame
    ...

This tool integrates every scan into the official DUFOMap Python backend,
propagates the map, then asks the same backend for a static/dynamic label for
each point of each target cloud.  Only original static points are written back;
coordinates, intensity, and any extra PCD fields are preserved.

The backend is never replaced by a hand-written dynamic filter here.  The
optional ``backend_factory`` argument exists so the adapter and error paths can
be unit-tested with a controlled stand-in; production always loads ``dufomap``.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np


class FilterError(RuntimeError):
    """A user-facing failure that must be reported to the save service."""


# ---------------------------------------------------------------------------
# Minimal PCD I/O. PCL writes binary PCD with arbitrary fields; preserving the
# structured numpy dtype is what keeps original intensity/ring/time attributes.
# ---------------------------------------------------------------------------


_PCD_TYPES = {
    ("F", 4): np.dtype("<f4"),
    ("F", 8): np.dtype("<f8"),
    ("U", 1): np.dtype("u1"),
    ("U", 2): np.dtype("<u2"),
    ("U", 4): np.dtype("<u4"),
    ("U", 8): np.dtype("<u8"),
    ("I", 1): np.dtype("i1"),
    ("I", 2): np.dtype("<i2"),
    ("I", 4): np.dtype("<i4"),
    ("I", 8): np.dtype("<i8"),
}


@dataclass
class PcdCloud:
    metadata: Dict[str, Any]
    data: np.ndarray

    @property
    def xyz(self) -> np.ndarray:
        try:
            return np.column_stack(
                (
                    np.asarray(self.data["x"]).reshape(-1),
                    np.asarray(self.data["y"]).reshape(-1),
                    np.asarray(self.data["z"]).reshape(-1),
                )
            ).astype(np.float32, copy=False)
        except (KeyError, ValueError) as error:
            raise FilterError("PCD cloud is missing x/y/z fields") from error

    def subset(self, mask: np.ndarray) -> "PcdCloud":
        metadata = dict(self.metadata)
        data = self.data[mask].copy()
        metadata["width"] = int(data.shape[0])
        metadata["height"] = 1
        metadata["points"] = int(data.shape[0])
        return PcdCloud(metadata, data)


def _parse_header(lines: Sequence[str]) -> Dict[str, Any]:
    metadata: Dict[str, Any] = {}
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        key, _, value = line.partition(" ")
        key = key.lower()
        value = value.strip()
        if not value:
            continue
        if key == "fields":
            metadata["fields"] = value.split()
        elif key == "type":
            metadata["type"] = value.split()
        elif key == "size":
            metadata["size"] = [int(item) for item in value.split()]
        elif key == "count":
            metadata["count"] = [int(item) for item in value.split()]
        elif key in ("width", "height", "points"):
            metadata[key] = int(value)
        elif key == "viewpoint":
            metadata["viewpoint"] = [float(item) for item in value.split()]
        elif key == "version":
            metadata["version"] = value
        elif key == "data":
            metadata["data"] = value.lower()
    required = ("fields", "size", "type")
    if any(key not in metadata for key in required):
        raise FilterError("PCD header is missing FIELDS/SIZE/TYPE")
    metadata.setdefault("count", [1] * len(metadata["fields"]))
    metadata.setdefault("width", metadata.get("points", 0))
    metadata.setdefault("height", 1)
    metadata.setdefault("points", metadata["width"] * metadata["height"])
    metadata.setdefault("viewpoint", [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    metadata.setdefault("data", "binary")
    if len(metadata["count"]) != len(metadata["fields"]):
        raise FilterError("PCD COUNT does not match FIELDS")
    if len(metadata["type"]) != len(metadata["fields"]):
        raise FilterError("PCD TYPE does not match FIELDS")
    if len(metadata["size"]) != len(metadata["fields"]):
        raise FilterError("PCD SIZE does not match FIELDS")
    return metadata


def _structured_dtype(metadata: Mapping[str, Any]) -> np.dtype:
    fields: List[Tuple[str, Any]] = []
    try:
        for name, size, kind, count in zip(
            metadata["fields"],
            metadata["size"],
            metadata["type"],
            metadata["count"],
        ):
            base = _PCD_TYPES[(str(kind).upper(), int(size))]
            if int(count) == 1:
                fields.append((str(name), base))
            else:
                fields.append((str(name), base, (int(count),)))
    except KeyError as error:
        raise FilterError(
            "unsupported PCD field type/size: {}".format(error.args[0])
        ) from error
    return np.dtype(fields)


def read_pcd(path: Path) -> PcdCloud:
    try:
        stream = open(path, "rb")
    except OSError as error:
        raise FilterError("cannot open point cloud {}: {}".format(path, error)) from error
    with stream:
        header: List[str] = []
        while True:
            raw = stream.readline()
            if not raw:
                raise FilterError("truncated PCD header: {}".format(path))
            line = raw.decode("utf-8", errors="replace")
            header.append(line)
            if line.strip().upper().startswith("DATA"):
                break
        metadata = _parse_header(header)
        dtype = _structured_dtype(metadata)
        points = int(metadata["points"])
        mode = str(metadata["data"]).lower()
        if mode == "binary":
            payload = stream.read(points * dtype.itemsize)
            if len(payload) != points * dtype.itemsize:
                raise FilterError(
                    "truncated binary PCD {}: expected {} bytes, got {}".format(
                        path, points * dtype.itemsize, len(payload)
                    )
                )
            data = np.frombuffer(payload, dtype=dtype, count=points).copy()
        elif mode == "ascii":
            values = np.loadtxt(stream, dtype=np.float64, ndmin=2)
            if values.shape[0] != points:
                raise FilterError(
                    "ascii PCD {} has {} rows, expected {}".format(
                        path, values.shape[0], points
                    )
                )
            data = np.zeros(points, dtype=dtype)
            column = 0
            for name, count, kind, size in zip(
                metadata["fields"],
                metadata["count"],
                metadata["type"],
                metadata["size"],
            ):
                base = _PCD_TYPES[(str(kind).upper(), int(size))]
                width = int(count)
                block = values[:, column : column + width]
                if width == 1:
                    data[str(name)] = block[:, 0].astype(base)
                else:
                    data[str(name)] = block.astype(base)
                column += width
        else:
            raise FilterError(
                "unsupported PCD DATA mode {} in {}".format(mode, path)
            )
    return PcdCloud(metadata, data)


def _header_text(cloud: PcdCloud) -> bytes:
    metadata = cloud.metadata
    fields = " ".join(str(item) for item in metadata["fields"])
    sizes = " ".join(str(int(item)) for item in metadata["size"])
    types = " ".join(str(item).upper() for item in metadata["type"])
    counts = " ".join(str(int(item)) for item in metadata["count"])
    viewpoint = " ".join(
        "{:.17g}".format(float(item)) for item in metadata["viewpoint"]
    )
    lines = [
        "VERSION {}".format(metadata.get("version", ".7")),
        "FIELDS {}".format(fields),
        "SIZE {}".format(sizes),
        "TYPE {}".format(types),
        "COUNT {}".format(counts),
        "WIDTH {}".format(int(metadata["width"])),
        "HEIGHT {}".format(int(metadata["height"])),
        "VIEWPOINT {}".format(viewpoint),
        "POINTS {}".format(int(metadata["points"])),
        "DATA binary",
        "",
    ]
    return "\n".join(lines).encode("ascii")


def write_pcd(path: Path, cloud: PcdCloud) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp.{}".format(os.getpid()))
    try:
        with open(temporary, "wb") as stream:
            stream.write(_header_text(cloud))
            stream.write(np.ascontiguousarray(cloud.data).tobytes())
        os.replace(temporary, path)
    except OSError as error:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise FilterError("cannot write point cloud {}: {}".format(path, error)) from error


# ---------------------------------------------------------------------------
# Dataset / backend adapter
# ---------------------------------------------------------------------------


def _safe_relative(root: Path, relative: Any, what: str) -> Path:
    if not isinstance(relative, str) or not relative:
        raise FilterError("{} must be a non-empty relative path".format(what))
    candidate = Path(relative)
    if candidate.is_absolute():
        raise FilterError("{} must be relative: {}".format(what, relative))
    resolved = (root / candidate).resolve()
    try:
        resolved.relative_to(root.resolve())
    except ValueError as error:
        raise FilterError("{} escapes its root: {}".format(what, relative)) from error
    return resolved


def _load_official_backend() -> Any:
    try:
        from dufomap import dufomap as official_dufomap  # type: ignore
    except ImportError as error:
        raise FilterError(
            "official DUFOMap Python package is not installed for this "
            "interpreter. This is a legacy manual replay tool; run it with an "
            "interpreter that has the official dufomap package, or use the "
            "production native save path via use_dufomap: true. Run "
            "install_dufomap_backend.sh --smoke to build the pinned official "
            "native core."
        ) from error
    return official_dufomap


def _validate_manifest(manifest: Mapping[str, Any]) -> Dict[str, Any]:
    if manifest.get("format") != "lio_sam_dufomap_export":
        raise FilterError("manifest format is not lio_sam_dufomap_export")
    if int(manifest.get("version", 0)) != 1:
        raise FilterError("unsupported manifest version")
    parameters = manifest.get("parameters")
    frames = manifest.get("frames")
    targets = manifest.get("targets")
    if not isinstance(parameters, dict):
        raise FilterError("manifest parameters are missing")
    if not isinstance(frames, list) or not frames:
        raise FilterError("manifest has no evidence frames")
    if not isinstance(targets, list) or not targets:
        raise FilterError("manifest has no target clouds")
    return {
        "parameters": parameters,
        "frames": frames,
        "targets": targets,
        "summary": manifest.get("summary", "summary.json"),
    }


def _pose_matrix(frame: Mapping[str, Any]) -> np.ndarray:
    pose = frame.get("pose")
    try:
        matrix = np.asarray(pose, dtype=np.float64).reshape(4, 4)
    except (TypeError, ValueError) as error:
        raise FilterError("frame pose must be a 4x4 matrix") from error
    if not np.isfinite(matrix).all():
        raise FilterError("frame pose contains non-finite values")
    if not np.allclose(matrix[3, :], (0.0, 0.0, 0.0, 1.0), atol=1e-6):
        raise FilterError("frame pose bottom row must be [0, 0, 0, 1]")
    return matrix


def _range_mask(points: np.ndarray, minimum: float, maximum: float) -> np.ndarray:
    finite = np.isfinite(points).all(axis=1)
    ranges = np.linalg.norm(points, axis=1)
    mask = finite & (ranges >= minimum)
    if maximum > 0.0:
        mask &= ranges <= maximum
    return mask


def run_filter(
    dataset_directory: os.PathLike[str] | str,
    output_directory: os.PathLike[str] | str,
    summary_path: Optional[os.PathLike[str] | str] = None,
    backend_factory: Optional[Callable[[Mapping[str, Any]], Any]] = None,
) -> Dict[str, Any]:
    """Run the official DUFOMap backend over a manifest dataset.

    ``backend_factory`` is deliberately optional and only used by tests. It
    receives the manifest's parameter mapping and must return an object with
    the official ``run``/``oncePropagateCluster``/``segment`` methods.
    """
    started = time.time()
    dataset = Path(dataset_directory).resolve()
    output = Path(output_directory).resolve()
    manifest_path = dataset / "manifest.json"
    try:
        with open(manifest_path, "r", encoding="utf-8") as stream:
            raw_manifest = json.load(stream)
    except (OSError, ValueError) as error:
        raise FilterError("cannot read dataset manifest {}: {}".format(manifest_path, error)) from error

    parsed = _validate_manifest(raw_manifest)
    parameters = parsed["parameters"]
    frames = parsed["frames"]
    targets = parsed["targets"]
    try:
        resolution = float(parameters["resolution"])
        d_s = float(parameters["d_s"])
        d_p = int(parameters["d_p"])
        num_threads = int(parameters.get("num_threads", 0))
        min_range = float(parameters.get("min_range", 0.0))
        max_range = float(parameters.get("max_range", 0.0))
    except (KeyError, TypeError, ValueError) as error:
        raise FilterError("manifest DUFOMap parameters are invalid") from error
    if resolution <= 0.0 or not np.isfinite(resolution):
        raise FilterError("DUFOMap resolution must be finite and positive")
    if d_s < 0.0 or not np.isfinite(d_s):
        raise FilterError("DUFOMap d_s must be non-negative")
    if d_p < 0 or d_p > 4:
        raise FilterError("DUFOMap d_p must be between 0 and 4")
    if num_threads < 0 or num_threads == 1:
        raise FilterError("DUFOMap num_threads must be 0 (automatic) or at least 2")
    if min_range < 0.0 or not np.isfinite(min_range):
        raise FilterError("DUFOMap min_range must be non-negative")
    if max_range < 0.0 or not np.isfinite(max_range):
        raise FilterError("DUFOMap max_range must be non-negative")
    if max_range > 0.0 and max_range < min_range:
        raise FilterError("DUFOMap max_range must be zero or >= min_range")

    output.mkdir(parents=True, exist_ok=True)
    backend = (
        backend_factory(parameters)
        if backend_factory is not None
        else _load_official_backend()(resolution, d_s, d_p, num_threads=num_threads)
    )

    # One pass: integrate and immediately release each scan. No concatenation
    # of the complete session is kept in Python memory.
    integrated_points = 0
    for position, frame in enumerate(frames):
        scan_relative = frame.get("scan")
        scan_path = _safe_relative(dataset, scan_relative, "frame scan")
        pose = _pose_matrix(frame)
        cloud = read_pcd(scan_path)
        points = cloud.xyz
        mask = _range_mask(points, min_range, max_range)
        selected = np.ascontiguousarray(points[mask])
        if selected.shape[0] == 0:
            raise FilterError(
                "frame {} scan {} has no points after the range gate".format(
                    position, scan_relative
                )
            )
        backend.run(selected, pose, cloud_transform=True)
        integrated_points += int(selected.shape[0])

    # The official propagation pass is what turns the integrated ray evidence
    # into per-voxel dynamic awareness.
    backend.oncePropagateCluster(if_propagate=True, if_cluster=False)

    target_summaries: List[Dict[str, Any]] = []
    identity = np.eye(4, dtype=np.float64)
    for target in targets:
        name = str(target.get("name", ""))
        if not name:
            raise FilterError("target name must not be empty")
        input_path = _safe_relative(dataset, target.get("input"), "target input")
        output_path = _safe_relative(output, target.get("output"), "target output")
        cloud = read_pcd(input_path)
        total = int(cloud.data.shape[0])
        kept = 0
        if total > 0:
            xyz = cloud.xyz
            if not np.isfinite(xyz).all():
                raise FilterError(
                    "target {} contains non-finite coordinates; refusing to "
                    "label them".format(name)
                )
            labels = np.asarray(
                backend.segment(
                    np.ascontiguousarray(xyz.astype(np.float32, copy=False)),
                    identity,
                    cloud_transform=False,
                )
            ).reshape(-1)
            if labels.shape[0] != total:
                raise FilterError(
                    "backend returned {} labels for target {} with {} points".format(
                        labels.shape[0], name, total
                    )
                )
            if not np.all((labels == 0) | (labels == 1)):
                raise FilterError(
                    "backend returned non-binary labels for target {}".format(name)
                )
            static_mask = labels == 0
            kept = int(static_mask.sum())
            write_pcd(output_path, cloud.subset(static_mask))
        else:
            write_pcd(output_path, cloud)
        target_summaries.append(
            {
                "name": name,
                "input": str(input_path),
                "output": str(output_path),
                "points_in": total,
                "points_kept": kept,
                "points_removed": total - kept,
            }
        )

    backend_module = sys.modules.get("dufomap") if backend_factory is None else None
    backend_version = getattr(backend_module, "__version__", None)
    backend_kind = getattr(backend_module, "__backend_kind__", None)
    summary = {
        "format": "lio_sam_dufomap_export_summary",
        "version": 1,
        "backend": ("injected-test-backend" if backend_factory is not None else
                    backend_kind or "official-dufomap-python-api"),
        "backend_source": getattr(backend_module, "__backend__", None),
        "ufomap_revision": getattr(backend_module, "__ufomap_revision__", None),
        "backend_version": backend_version,
        "parameters": {
            "resolution": resolution,
            "d_s": d_s,
            "d_p": d_p,
            "num_threads": num_threads,
            "min_range": min_range,
            "max_range": max_range,
            "if_propagate": True,
            "if_cluster": False,
        },
        "frames": len(frames),
        "integrated_points": integrated_points,
        "targets": target_summaries,
        "elapsed_seconds": time.time() - started,
    }

    destination = (
        Path(summary_path).resolve()
        if summary_path is not None
        else _safe_relative(output, parsed["summary"], "summary path")
    )
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + ".tmp.{}".format(os.getpid()))
    try:
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, destination)
    except OSError as error:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise FilterError("cannot write DUFOMap summary {}: {}".format(destination, error)) from error
    summary["summary_path"] = str(destination)
    return summary


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Filter an exported LIO-SAM map with the official DUFOMap backend."
    )
    parser.add_argument("--dataset", required=True, help="dataset directory with manifest.json")
    parser.add_argument("--output", required=True, help="directory for filtered target clouds")
    parser.add_argument("--summary", default=None, help="optional summary JSON path")
    arguments = parser.parse_args(argv)
    try:
        summary = run_filter(arguments.dataset, arguments.output, arguments.summary)
    except FilterError as error:
        print("DUFOMap export filter failed: {}".format(error), file=sys.stderr)
        return 2
    except Exception as error:  # noqa: BLE001 - report backend failures verbatim
        print(
            "DUFOMap export filter failed unexpectedly: {}: {}".format(
                type(error).__name__, error
            ),
            file=sys.stderr,
        )
        return 2
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
