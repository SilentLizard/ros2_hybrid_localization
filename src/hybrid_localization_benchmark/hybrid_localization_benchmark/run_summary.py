"""Aggregate benchmark streams and evaluate deterministic Stage-5 acceptance criteria."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import csv
import json
import math
from pathlib import Path
import statistics
from typing import Final, Iterable

from .run_schema import (
    BENCHMARK_SCHEMA_VERSION,
    MANIFEST_FILENAME,
    OUTPUT_STREAMS,
    SUMMARY_FILENAME,
)


@dataclass(frozen=True)
class AcceptanceCriteria:
    """Optional quantitative limits plus mandatory stream-completeness minima."""

    min_localization_samples: int = 1
    min_belief_runtime_samples: int = 1
    min_resource_samples: int = 1
    max_position_rmse_m: float | None = None
    max_position_error_m: float | None = None
    max_yaw_rmse_rad: float | None = None
    max_yaw_error_rad: float | None = None
    max_mean_cpu_percent: float | None = None
    max_peak_rss_bytes: int | None = None
    max_mean_update_duration_ns: float | None = None
    max_update_duration_ns: int | None = None

    def validate(self) -> None:
        for name, value in (
            ("min_localization_samples", self.min_localization_samples),
            ("min_belief_runtime_samples", self.min_belief_runtime_samples),
            ("min_resource_samples", self.min_resource_samples),
        ):
            if value < 1:
                raise ValueError(f"{name} must be at least one")

        for name, value in (
            ("max_position_rmse_m", self.max_position_rmse_m),
            ("max_position_error_m", self.max_position_error_m),
            ("max_yaw_rmse_rad", self.max_yaw_rmse_rad),
            ("max_yaw_error_rad", self.max_yaw_error_rad),
            ("max_mean_cpu_percent", self.max_mean_cpu_percent),
            ("max_mean_update_duration_ns", self.max_mean_update_duration_ns),
        ):
            if value is not None and (not math.isfinite(value) or value < 0.0):
                raise ValueError(f"{name} must be finite and non-negative when provided")
        if self.max_peak_rss_bytes is not None and self.max_peak_rss_bytes < 0:
            raise ValueError("max_peak_rss_bytes must be non-negative when provided")
        if self.max_update_duration_ns is not None and self.max_update_duration_ns < 0:
            raise ValueError("max_update_duration_ns must be non-negative when provided")


@dataclass(frozen=True)
class AcceptanceCheck:
    name: str
    passed: bool
    actual: int | float
    operator: str
    limit: int | float


@dataclass(frozen=True)
class StreamData:
    rows: tuple[dict[str, str], ...]

    @property
    def count(self) -> int:
        return len(self.rows)


def _percentile(values: list[float], fraction: float) -> float:
    """Deterministic nearest-rank percentile for non-empty finite data."""
    if not values:
        raise ValueError("percentile requires at least one value")
    if not 0.0 <= fraction <= 1.0:
        raise ValueError("percentile fraction must lie in [0, 1]")
    ordered = sorted(values)
    rank = max(1, math.ceil(fraction * len(ordered)))
    return ordered[rank - 1]


def _float_values(rows: Iterable[dict[str, str]], column: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        try:
            value = float(row[column])
        except (KeyError, ValueError) as exc:
            raise ValueError(f"invalid numeric value in column {column!r}") from exc
        if not math.isfinite(value):
            raise ValueError(f"column {column!r} contains a non-finite value")
        values.append(value)
    return values


def _int_values(rows: Iterable[dict[str, str]], column: str) -> list[int]:
    values: list[int] = []
    for row in rows:
        try:
            values.append(int(row[column]))
        except (KeyError, ValueError) as exc:
            raise ValueError(f"invalid integer value in column {column!r}") from exc
    return values


def _read_stream(run_directory: Path, filename: str, expected_columns: tuple[str, ...]) -> StreamData:
    path = run_directory / filename
    if not path.is_file():
        raise FileNotFoundError(f"required benchmark stream is missing: {path}")
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if tuple(reader.fieldnames or ()) != expected_columns:
            raise ValueError(f"unexpected columns in {filename}")
        rows = tuple(reader)
    return StreamData(rows=rows)


def _mean(values: list[float]) -> float:
    if not values:
        raise ValueError("mean requires at least one value")
    return statistics.fmean(values)


def _rmse(values: list[float]) -> float:
    if not values:
        raise ValueError("RMSE requires at least one value")
    return math.sqrt(statistics.fmean(value * value for value in values))


def _localization_metrics(data: StreamData) -> dict[str, int | float]:
    position = _float_values(data.rows, "position_error_m")
    yaw = _float_values(data.rows, "yaw_error_rad")
    timestamps = _int_values(data.rows, "sim_time_ns")
    if not position:
        return {"sample_count": 0}
    return {
        "sample_count": data.count,
        "duration_ns": max(0, timestamps[-1] - timestamps[0]),
        "position_error_mean_m": _mean(position),
        "position_error_rmse_m": _rmse(position),
        "position_error_p95_m": _percentile(position, 0.95),
        "position_error_max_m": max(position),
        "yaw_error_mean_rad": _mean(yaw),
        "yaw_error_rmse_rad": _rmse(yaw),
        "yaw_error_p95_rad": _percentile(yaw, 0.95),
        "yaw_error_max_rad": max(yaw),
    }


def _belief_metrics(data: StreamData) -> dict[str, int | float]:
    if not data.rows:
        return {"sample_count": 0}
    particle = _float_values(data.rows, "particle_count")
    ess = _float_values(data.rows, "effective_sample_size")
    components = _float_values(data.rows, "gmm_component_count")
    represented = _float_values(data.rows, "gmm_represented_weight")
    discarded = _float_values(data.rows, "gmm_discarded_weight")
    entropy = _float_values(data.rows, "gmm_entropy")
    dominant = _float_values(data.rows, "dominant_component_weight")
    ess_ratio = [e / p for e, p in zip(ess, particle, strict=True)]
    return {
        "sample_count": data.count,
        "particle_count_mean": _mean(particle),
        "particle_count_max": int(max(particle)),
        "effective_sample_size_mean": _mean(ess),
        "effective_sample_size_ratio_mean": _mean(ess_ratio),
        "effective_sample_size_ratio_min": min(ess_ratio),
        "gmm_component_count_mean": _mean(components),
        "gmm_component_count_max": int(max(components)),
        "gmm_represented_weight_mean": _mean(represented),
        "gmm_represented_weight_min": min(represented),
        "gmm_discarded_weight_mean": _mean(discarded),
        "gmm_discarded_weight_max": max(discarded),
        "gmm_entropy_mean": _mean(entropy),
        "gmm_entropy_max": max(entropy),
        "dominant_component_weight_mean": _mean(dominant),
        "dominant_component_weight_min": min(dominant),
    }


def _resource_metrics(data: StreamData) -> dict[str, int | float]:
    if not data.rows:
        return {"sample_count": 0}
    cpu = _float_values(data.rows, "process_cpu_percent")
    rss = _int_values(data.rows, "process_rss_bytes")
    duration = _float_values(data.rows, "update_duration_ns")
    period = _float_values(data.rows, "update_period_ns")
    nonzero_period = [value for value in period if value > 0.0]
    return {
        "sample_count": data.count,
        "process_cpu_percent_mean": _mean(cpu),
        "process_cpu_percent_p95": _percentile(cpu, 0.95),
        "process_cpu_percent_max": max(cpu),
        "process_rss_bytes_mean": _mean([float(value) for value in rss]),
        "process_rss_bytes_max": max(rss),
        "update_duration_ns_mean": _mean(duration),
        "update_duration_ns_p95": _percentile(duration, 0.95),
        "update_duration_ns_max": int(max(duration)),
        "update_period_ns_mean": _mean(nonzero_period) if nonzero_period else 0.0,
        "update_period_ns_p95": _percentile(nonzero_period, 0.95) if nonzero_period else 0.0,
    }


def _append_check(
    checks: list[AcceptanceCheck], name: str, actual: int | float, operator: str, limit: int | float
) -> None:
    if operator == ">=":
        passed = actual >= limit
    elif operator == "<=":
        passed = actual <= limit
    else:
        raise ValueError(f"unsupported acceptance operator {operator!r}")
    checks.append(AcceptanceCheck(name, passed, actual, operator, limit))


def _evaluate_acceptance(
    criteria: AcceptanceCriteria,
    localization: dict[str, int | float],
    belief: dict[str, int | float],
    resources: dict[str, int | float],
) -> list[AcceptanceCheck]:
    checks: list[AcceptanceCheck] = []
    _append_check(checks, "localization_sample_count", localization["sample_count"], ">=", criteria.min_localization_samples)
    _append_check(checks, "belief_runtime_sample_count", belief["sample_count"], ">=", criteria.min_belief_runtime_samples)
    _append_check(checks, "resource_sample_count", resources["sample_count"], ">=", criteria.min_resource_samples)

    optional = (
        ("position_error_rmse_m", localization, "position_error_rmse_m", criteria.max_position_rmse_m),
        ("position_error_max_m", localization, "position_error_max_m", criteria.max_position_error_m),
        ("yaw_error_rmse_rad", localization, "yaw_error_rmse_rad", criteria.max_yaw_rmse_rad),
        ("yaw_error_max_rad", localization, "yaw_error_max_rad", criteria.max_yaw_error_rad),
        ("process_cpu_percent_mean", resources, "process_cpu_percent_mean", criteria.max_mean_cpu_percent),
        ("process_rss_bytes_max", resources, "process_rss_bytes_max", criteria.max_peak_rss_bytes),
        ("update_duration_ns_mean", resources, "update_duration_ns_mean", criteria.max_mean_update_duration_ns),
        ("update_duration_ns_max", resources, "update_duration_ns_max", criteria.max_update_duration_ns),
    )
    for check_name, metrics, metric_name, limit in optional:
        if limit is not None:
            _append_check(checks, check_name, metrics[metric_name], "<=", limit)
    return checks


def build_run_summary(run_directory: Path, criteria: AcceptanceCriteria | None = None) -> dict[str, object]:
    """Read one completed run and return deterministic aggregate metrics/acceptance."""
    run_directory = Path(run_directory)
    criteria = criteria or AcceptanceCriteria()
    criteria.validate()

    manifest_path = run_directory / MANIFEST_FILENAME
    if not manifest_path.is_file():
        raise FileNotFoundError(f"benchmark manifest is missing: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema_version") != BENCHMARK_SCHEMA_VERSION:
        raise ValueError("benchmark manifest schema_version does not match this reader")

    stream_map = {stream.filename: stream for stream in OUTPUT_STREAMS}
    localization_data = _read_stream(
        run_directory, "localization.csv", stream_map["localization.csv"].columns
    )
    belief_data = _read_stream(
        run_directory, "belief_runtime.csv", stream_map["belief_runtime.csv"].columns
    )
    resource_data = _read_stream(
        run_directory, "resources.csv", stream_map["resources.csv"].columns
    )

    localization = _localization_metrics(localization_data)
    belief = _belief_metrics(belief_data)
    resources = _resource_metrics(resource_data)
    checks = _evaluate_acceptance(criteria, localization, belief, resources)

    return {
        "schema_version": BENCHMARK_SCHEMA_VERSION,
        "run": manifest.get("run", {}),
        "metrics": {
            "localization": localization,
            "belief_runtime": belief,
            "resources": resources,
        },
        "acceptance": {
            "passed": all(check.passed for check in checks),
            "criteria": asdict(criteria),
            "checks": [asdict(check) for check in checks],
        },
    }


def write_run_summary(run_directory: Path, criteria: AcceptanceCriteria | None = None) -> Path:
    """Create summary.json once; refuse to overwrite an existing summary."""
    run_directory = Path(run_directory)
    path = run_directory / SUMMARY_FILENAME
    if path.exists():
        raise FileExistsError(f"refusing to overwrite existing benchmark summary: {path}")
    summary = build_run_summary(run_directory, criteria)
    path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path
