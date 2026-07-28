from __future__ import annotations

import copy
import math
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import yaml


class BaselineError(RuntimeError):
    pass


BASELINE = {
    "schema_version": 1,
    "dataset": {
        "duration_ns": 756378139407,
        "message_count": 46875,
        "topic_counts": {"/imu": 42617, "/lidar_points": 4258},
        "map_points": 178455,
    },
    "quick": {
        "start_offset_seconds": 0.0,
        "duration_seconds": 180.0,
        "rate": 1.0,
        "startup_grace_seconds": 30.0,
        "first_odom_max_seconds": 60.0,
        "min_valid_odom_ratio": 0.95,
        "max_odom_gap_seconds": 1.0,
        "ok_deadline_seconds": 90.0,
        "max_error_samples": 0,
        "max_non_ok_gap_seconds": 5.0,
        "health_score_min": 0.5,
        "reg_fitness_max": 1.0,
        "covariance_scale_max": 100.0,
        "lidar_rate_min_hz": 5.0,
        "imu_rate_min_hz": 50.0,
    },
    "memory": {
        "warmup_seconds": 300.0,
        "duration_seconds": 7200.0,
        "sample_period_seconds": 5.0,
        "max_rss_mib": 4096.0,
        "max_slope_mib_per_hour": 64.0,
    },
}


_TOP_LEVEL_KEYS = ("schema_version", "dataset", "quick", "memory")
_DATASET_KEYS = ("duration_ns", "message_count", "topic_counts", "map_points")
_TOPIC_COUNT_KEYS = ("/imu", "/lidar_points")
_QUICK_KEYS = (
    "start_offset_seconds",
    "duration_seconds",
    "rate",
    "startup_grace_seconds",
    "first_odom_max_seconds",
    "min_valid_odom_ratio",
    "max_odom_gap_seconds",
    "ok_deadline_seconds",
    "max_error_samples",
    "max_non_ok_gap_seconds",
    "health_score_min",
    "reg_fitness_max",
    "covariance_scale_max",
    "lidar_rate_min_hz",
    "imu_rate_min_hz",
)
_MEMORY_KEYS = (
    "warmup_seconds",
    "duration_seconds",
    "sample_period_seconds",
    "max_rss_mib",
    "max_slope_mib_per_hour",
)
_MISSING = object()


def load_baseline(path: Path) -> dict:
    """Load and validate a version-one localization stability baseline."""
    baseline_path = Path(path)
    if not baseline_path.is_file():
        raise BaselineError(f"Baseline file is missing: {baseline_path}")

    try:
        with baseline_path.open(encoding="utf-8") as baseline_file:
            contents = yaml.safe_load(baseline_file)
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise BaselineError(
            f"Unable to read baseline file {baseline_path}: {error}"
        ) from error

    try:
        return _validate_baseline(contents)
    except BaselineError as error:
        raise BaselineError(f"Invalid baseline file {baseline_path}: {error}") from error


def compare_quick_metrics(
    baseline: Mapping[str, Any], observed: Mapping[str, Any]
) -> list[str]:
    """Return one violation per quick metric that is absent or out of bounds."""
    quick = _validate_baseline(baseline)["quick"]
    if not isinstance(observed, Mapping):
        raise BaselineError("Observed quick metrics must be a mapping")

    violations: list[str] = []
    _append_upper_bound_violation(
        violations,
        observed,
        "first_odom_seconds",
        quick["first_odom_max_seconds"],
    )
    _append_lower_bound_violation(
        violations,
        observed,
        "valid_odom_ratio",
        quick["min_valid_odom_ratio"],
    )
    _append_upper_bound_violation(
        violations,
        observed,
        "max_odom_gap_seconds",
        quick["max_odom_gap_seconds"],
    )
    _append_upper_bound_violation(
        violations,
        observed,
        "first_ok_seconds",
        quick["ok_deadline_seconds"],
    )
    _append_upper_bound_violation(
        violations,
        observed,
        "error_samples",
        quick["max_error_samples"],
    )
    _append_upper_bound_violation(
        violations,
        observed,
        "max_non_ok_gap_seconds",
        quick["max_non_ok_gap_seconds"],
    )
    _append_lower_bound_violation(
        violations,
        observed,
        "min_health_score",
        quick["health_score_min"],
    )
    _append_upper_bound_violation(
        violations,
        observed,
        "max_reg_fitness",
        quick["reg_fitness_max"],
    )
    _append_upper_bound_violation(
        violations,
        observed,
        "max_covariance_scale",
        quick["covariance_scale_max"],
    )
    _append_lower_bound_violation(
        violations,
        observed,
        "min_lidar_rate_hz",
        quick["lidar_rate_min_hz"],
    )
    _append_lower_bound_violation(
        violations,
        observed,
        "min_imu_rate_hz",
        quick["imu_rate_min_hz"],
    )
    if observed.get("timestamp_monotonic", _MISSING) is not True:
        violations.append(
            "timestamp_monotonic must be exactly True, observed "
            f"{observed.get('timestamp_monotonic', _MISSING)!r}"
        )

    return violations


def write_candidate(path, identity, observed):
    """Write a conservative candidate baseline from a dataset identity and observations."""
    if not isinstance(observed, Mapping):
        raise BaselineError("Observed quick metrics must be a mapping")

    first_odom = _candidate_number(observed, "first_odom_seconds")
    valid_odom_ratio = _candidate_number(observed, "valid_odom_ratio")
    max_odom_gap = _candidate_number(observed, "max_odom_gap_seconds")
    first_ok = _candidate_number(observed, "first_ok_seconds")
    max_non_ok_gap = _candidate_number(observed, "max_non_ok_gap_seconds")
    min_health_score = _candidate_number(observed, "min_health_score")
    max_reg_fitness = _candidate_number(observed, "max_reg_fitness")
    max_covariance_scale = _candidate_number(observed, "max_covariance_scale")
    min_lidar_rate = _candidate_number(observed, "min_lidar_rate_hz")
    min_imu_rate = _candidate_number(observed, "min_imu_rate_hz")
    error_samples = _candidate_error_samples(observed)

    candidate = {
        "schema_version": 1,
        "dataset": {
            "duration_ns": _identity_value(identity, "duration_ns"),
            "message_count": _identity_value(identity, "message_count"),
            "topic_counts": copy.deepcopy(_identity_value(identity, "topic_counts")),
            "map_points": _identity_value(identity, "map_points"),
        },
        "quick": {
            "start_offset_seconds": 0.0,
            "duration_seconds": 180.0,
            "rate": 1.0,
            "startup_grace_seconds": 30.0,
            "first_odom_max_seconds": max(first_odom * 1.25, first_odom + 2.0),
            "min_valid_odom_ratio": max(0.0, valid_odom_ratio * 0.90),
            "max_odom_gap_seconds": max(max_odom_gap * 1.25, max_odom_gap + 0.1),
            "ok_deadline_seconds": first_ok * 1.10,
            "max_error_samples": error_samples,
            "max_non_ok_gap_seconds": max_non_ok_gap * 1.10,
            "health_score_min": min_health_score * 0.90,
            "reg_fitness_max": max_reg_fitness * 1.10,
            "covariance_scale_max": max_covariance_scale * 1.10,
            "lidar_rate_min_hz": min_lidar_rate * 0.90,
            "imu_rate_min_hz": min_imu_rate * 0.90,
        },
        "memory": {
            "warmup_seconds": 300.0,
            "duration_seconds": 7200.0,
            "sample_period_seconds": 5.0,
            "max_rss_mib": 4096.0,
            "max_slope_mib_per_hour": 64.0,
        },
    }
    candidate = _validate_baseline(candidate)

    candidate_path = Path(path)
    try:
        candidate_path.parent.mkdir(parents=True, exist_ok=True)
        with candidate_path.open("w", encoding="utf-8") as candidate_file:
            yaml.safe_dump(candidate, candidate_file, sort_keys=False)
    except (OSError, yaml.YAMLError) as error:
        raise BaselineError(
            f"Unable to write candidate baseline {candidate_path}: {error}"
        ) from error
    return candidate_path


def _validate_baseline(contents: Any) -> dict:
    baseline = _require_mapping(contents, "baseline")
    _require_exact_keys(baseline, _TOP_LEVEL_KEYS, "baseline")

    version = baseline["schema_version"]
    if not isinstance(version, int) or isinstance(version, bool) or version != 1:
        raise BaselineError("baseline.schema_version must be the supported version 1")

    dataset = _validate_dataset(baseline["dataset"])
    quick = _validate_quick(baseline["quick"])
    memory = _validate_memory(baseline["memory"])
    return {
        "schema_version": version,
        "dataset": dataset,
        "quick": quick,
        "memory": memory,
    }


def _validate_dataset(value: Any) -> dict:
    dataset = _require_mapping(value, "dataset")
    _require_exact_keys(dataset, _DATASET_KEYS, "dataset")
    topic_counts = _require_mapping(dataset["topic_counts"], "dataset.topic_counts")
    _require_exact_keys(topic_counts, _TOPIC_COUNT_KEYS, "dataset.topic_counts")
    return {
        "duration_ns": _require_nonnegative_int(
            dataset["duration_ns"], "dataset.duration_ns"
        ),
        "message_count": _require_nonnegative_int(
            dataset["message_count"], "dataset.message_count"
        ),
        "topic_counts": {
            topic: _require_nonnegative_int(
                topic_counts[topic], f"dataset.topic_counts.{topic}"
            )
            for topic in _TOPIC_COUNT_KEYS
        },
        "map_points": _require_nonnegative_int(dataset["map_points"], "dataset.map_points"),
    }


def _validate_quick(value: Any) -> dict:
    quick = _require_mapping(value, "quick")
    _require_exact_keys(quick, _QUICK_KEYS, "quick")
    validated = {
        "start_offset_seconds": _require_nonnegative_number(
            quick["start_offset_seconds"], "quick.start_offset_seconds"
        ),
        "duration_seconds": _require_positive_number(
            quick["duration_seconds"], "quick.duration_seconds"
        ),
        "rate": _require_positive_number(quick["rate"], "quick.rate"),
        "startup_grace_seconds": _require_nonnegative_number(
            quick["startup_grace_seconds"], "quick.startup_grace_seconds"
        ),
        "first_odom_max_seconds": _require_nonnegative_number(
            quick["first_odom_max_seconds"], "quick.first_odom_max_seconds"
        ),
        "min_valid_odom_ratio": _require_unit_interval_number(
            quick["min_valid_odom_ratio"], "quick.min_valid_odom_ratio"
        ),
        "max_odom_gap_seconds": _require_nonnegative_number(
            quick["max_odom_gap_seconds"], "quick.max_odom_gap_seconds"
        ),
        "ok_deadline_seconds": _require_nonnegative_number(
            quick["ok_deadline_seconds"], "quick.ok_deadline_seconds"
        ),
        "max_error_samples": _require_nonnegative_int(
            quick["max_error_samples"], "quick.max_error_samples"
        ),
        "max_non_ok_gap_seconds": _require_nonnegative_number(
            quick["max_non_ok_gap_seconds"], "quick.max_non_ok_gap_seconds"
        ),
        "health_score_min": _require_unit_interval_number(
            quick["health_score_min"], "quick.health_score_min"
        ),
        "reg_fitness_max": _require_nonnegative_number(
            quick["reg_fitness_max"], "quick.reg_fitness_max"
        ),
        "covariance_scale_max": _require_nonnegative_number(
            quick["covariance_scale_max"], "quick.covariance_scale_max"
        ),
        "lidar_rate_min_hz": _require_nonnegative_number(
            quick["lidar_rate_min_hz"], "quick.lidar_rate_min_hz"
        ),
        "imu_rate_min_hz": _require_nonnegative_number(
            quick["imu_rate_min_hz"], "quick.imu_rate_min_hz"
        ),
    }
    return validated


def _validate_memory(value: Any) -> dict:
    memory = _require_mapping(value, "memory")
    _require_exact_keys(memory, _MEMORY_KEYS, "memory")
    return {
        "warmup_seconds": _require_nonnegative_number(
            memory["warmup_seconds"], "memory.warmup_seconds"
        ),
        "duration_seconds": _require_positive_number(
            memory["duration_seconds"], "memory.duration_seconds"
        ),
        "sample_period_seconds": _require_positive_number(
            memory["sample_period_seconds"], "memory.sample_period_seconds"
        ),
        "max_rss_mib": _require_nonnegative_number(
            memory["max_rss_mib"], "memory.max_rss_mib"
        ),
        "max_slope_mib_per_hour": _require_nonnegative_number(
            memory["max_slope_mib_per_hour"], "memory.max_slope_mib_per_hour"
        ),
    }


def _append_upper_bound_violation(violations, observed, key, maximum) -> None:
    value = _finite_observed_value(observed, key)
    if value is None:
        violations.append(
            f"{key} must be a finite number <= {maximum}, observed "
            f"{observed.get(key, _MISSING)!r}"
        )
    elif value > maximum:
        violations.append(f"{key} must be <= {maximum}, observed {value}")


def _append_lower_bound_violation(violations, observed, key, minimum) -> None:
    value = _finite_observed_value(observed, key)
    if value is None:
        violations.append(
            f"{key} must be a finite number >= {minimum}, observed "
            f"{observed.get(key, _MISSING)!r}"
        )
    elif value < minimum:
        violations.append(f"{key} must be >= {minimum}, observed {value}")


def _finite_observed_value(observed: Mapping[str, Any], key: str) -> float | None:
    value = observed.get(key, _MISSING)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    numeric_value = float(value)
    if not math.isfinite(numeric_value):
        return None
    return numeric_value


def _candidate_number(observed: Mapping[str, Any], key: str) -> float:
    value = _finite_observed_value(observed, key)
    if value is None:
        raise BaselineError(
            f"Candidate observed value {key} must be a finite number, observed "
            f"{observed.get(key, _MISSING)!r}"
        )
    if value < 0:
        raise BaselineError(f"Candidate observed value {key} must be non-negative")
    return value


def _candidate_error_samples(observed: Mapping[str, Any]) -> int:
    value = observed.get("error_samples", _MISSING)
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise BaselineError(
            "Candidate observed value error_samples must be a non-negative integer"
        )
    return value


def _identity_value(identity: Any, key: str) -> Any:
    if isinstance(identity, Mapping):
        value = identity.get(key, _MISSING)
    else:
        value = getattr(identity, key, _MISSING)
    if value is _MISSING:
        raise BaselineError(f"Candidate identity is missing required field: {key}")
    return value


def _require_mapping(value: Any, key_path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise BaselineError(f"{key_path} must be a mapping")
    return value


def _require_exact_keys(
    mapping: Mapping[str, Any], expected_keys: tuple[str, ...], key_path: str
) -> None:
    missing = [key for key in expected_keys if key not in mapping]
    if missing:
        raise BaselineError(f"Missing required baseline key: {key_path}.{missing[0]}")
    unexpected = [key for key in mapping if key not in expected_keys]
    if unexpected:
        raise BaselineError(f"Unexpected baseline key: {key_path}.{unexpected[0]}")


def _require_nonnegative_int(value: Any, key_path: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise BaselineError(f"{key_path} must be a non-negative integer")
    return value


def _require_finite_number(value: Any, key_path: str) -> float | int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise BaselineError(f"{key_path} must be a finite number")
    if not math.isfinite(float(value)):
        raise BaselineError(f"{key_path} must be a finite number")
    return value


def _require_nonnegative_number(value: Any, key_path: str) -> float | int:
    numeric_value = _require_finite_number(value, key_path)
    if numeric_value < 0:
        raise BaselineError(f"{key_path} must be a non-negative finite number")
    return numeric_value


def _require_positive_number(value: Any, key_path: str) -> float | int:
    numeric_value = _require_finite_number(value, key_path)
    if numeric_value <= 0:
        raise BaselineError(f"{key_path} must be a positive finite number")
    return numeric_value


def _require_unit_interval_number(value: Any, key_path: str) -> float | int:
    numeric_value = _require_finite_number(value, key_path)
    if not 0 <= numeric_value <= 1:
        raise BaselineError(f"{key_path} must be a finite number between 0 and 1")
    return numeric_value
