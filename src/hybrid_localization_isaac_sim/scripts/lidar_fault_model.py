#!/usr/bin/env python3
"""ROS-independent deterministic LaserScan degradation for #44 Stage 4.

The model transforms only scan ranges. Scan geometry/timing is represented by
``LaserScanSample`` and remains immutable. Dropped, occluded, unavailable, or
corrupted-out-of-range returns are represented as ``+inf`` (no return), which
is compatible with the ROS LaserScan contract and avoids inventing zero-range
obstacles.

Random operations consume one fault-owned ``random.Random`` stream in beam
index order, making the same seed + parameters + input scan sequence reproduce
exactly the same corrupted sequence.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import Any, Mapping, Sequence


class LidarFaultModelError(ValueError):
    """Raised when a scan or LiDAR-fault configuration is invalid."""


@dataclass(frozen=True)
class LaserScanSample:
    sim_time_s: float
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: tuple[float, ...]


@dataclass(frozen=True)
class LidarFaultParameters:
    gaussian_noise_stddev_m: float = 0.0
    dropout_fraction: float = 0.0
    sector_start_rad: float | None = None
    sector_end_rad: float | None = None
    max_range_m: float | None = None
    outlier_fraction: float = 0.0
    outlier_min_m: float | None = None
    outlier_max_m: float | None = None
    complete_scan_loss: bool = False


def parameters_from_mapping(parameters: Mapping[str, Any]) -> LidarFaultParameters:
    """Translate the already validated Stage-1 parameter mapping."""

    return LidarFaultParameters(
        gaussian_noise_stddev_m=float(parameters.get("gaussian_noise_stddev_m", 0.0)),
        dropout_fraction=float(parameters.get("dropout_fraction", 0.0)),
        sector_start_rad=(
            None if "sector_start_rad" not in parameters else float(parameters["sector_start_rad"])
        ),
        sector_end_rad=(
            None if "sector_end_rad" not in parameters else float(parameters["sector_end_rad"])
        ),
        max_range_m=(None if "max_range_m" not in parameters else float(parameters["max_range_m"])),
        outlier_fraction=float(parameters.get("outlier_fraction", 0.0)),
        outlier_min_m=(
            None if "outlier_min_m" not in parameters else float(parameters["outlier_min_m"])
        ),
        outlier_max_m=(
            None if "outlier_max_m" not in parameters else float(parameters["outlier_max_m"])
        ),
        complete_scan_loss=bool(parameters.get("complete_scan_loss", False)),
    )


def _validate_scan(scan: LaserScanSample) -> None:
    scalar_values = (
        scan.sim_time_s,
        scan.angle_min,
        scan.angle_increment,
        scan.range_min,
        scan.range_max,
    )
    if not all(math.isfinite(value) for value in scalar_values):
        raise LidarFaultModelError("LaserScan metadata must be finite")
    if scan.sim_time_s < 0.0:
        raise LidarFaultModelError("LaserScan simulation time must be >= 0")
    if scan.angle_increment == 0.0:
        raise LidarFaultModelError("LaserScan angle_increment must be non-zero")
    if scan.range_min < 0.0 or scan.range_max <= scan.range_min:
        raise LidarFaultModelError("LaserScan requires 0 <= range_min < range_max")
    if not scan.ranges:
        raise LidarFaultModelError("LaserScan must contain at least one range")
    for value in scan.ranges:
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise LidarFaultModelError("LaserScan ranges must be numeric")


def _angle_in_sector(angle: float, start: float, end: float) -> bool:
    """Return whether a wrapped angle lies in the inclusive directed sector.

    Stage-1 parameters are normalized to [-pi, pi). If ``start <= end`` this
    is the ordinary interval. Otherwise the interval crosses the +/-pi seam.
    """

    wrapped = math.atan2(math.sin(angle), math.cos(angle))
    if start <= end:
        return start <= wrapped <= end
    return wrapped >= start or wrapped <= end


class DeterministicLidarFaultModel:
    """Pass through nominal scans or deterministically degrade ranges while active."""

    def __init__(self) -> None:
        self._active = False
        self._parameters = LidarFaultParameters()
        self._rng = random.Random(0)
        self._last_sim_time_s: float | None = None

    @property
    def active(self) -> bool:
        return self._active

    def activate(self, *, seed: int, parameters: Mapping[str, Any]) -> None:
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise LidarFaultModelError("seed must be a non-negative integer")
        self._parameters = parameters_from_mapping(parameters)
        self._rng = random.Random(seed)
        self._last_sim_time_s = None
        self._active = True

    def deactivate(self) -> None:
        self._active = False
        self._parameters = LidarFaultParameters()
        self._last_sim_time_s = None

    def process(self, scan: LaserScanSample) -> LaserScanSample:
        _validate_scan(scan)
        if not self._active:
            return scan

        if self._last_sim_time_s is not None and scan.sim_time_s <= self._last_sim_time_s:
            raise LidarFaultModelError(
                "active LaserScan samples must have strictly increasing simulation time"
            )
        self._last_sim_time_s = scan.sim_time_s

        parameters = self._parameters
        effective_range_max = scan.range_max
        if parameters.max_range_m is not None:
            effective_range_max = min(scan.range_max, parameters.max_range_m)

        output: list[float] = []
        for index, source_range in enumerate(scan.ranges):
            angle = scan.angle_min + float(index) * scan.angle_increment
            value = float(source_range)

            # Canonicalize all input invalid/no-return values while a fault is
            # active. Nominal pass-through above remains byte-for-value exact.
            if not math.isfinite(value) or value < scan.range_min or value > scan.range_max:
                output.append(math.inf)
                continue

            if parameters.complete_scan_loss:
                output.append(math.inf)
                continue

            if (
                parameters.sector_start_rad is not None
                and parameters.sector_end_rad is not None
                and _angle_in_sector(
                    angle,
                    parameters.sector_start_rad,
                    parameters.sector_end_rad,
                )
            ):
                output.append(math.inf)
                continue

            if parameters.dropout_fraction > 0.0 and self._rng.random() < parameters.dropout_fraction:
                output.append(math.inf)
                continue

            if value > effective_range_max:
                output.append(math.inf)
                continue

            if parameters.gaussian_noise_stddev_m > 0.0:
                value += self._rng.gauss(0.0, parameters.gaussian_noise_stddev_m)

            if parameters.outlier_fraction > 0.0 and self._rng.random() < parameters.outlier_fraction:
                assert parameters.outlier_min_m is not None
                assert parameters.outlier_max_m is not None
                value = self._rng.uniform(parameters.outlier_min_m, parameters.outlier_max_m)

            if not math.isfinite(value) or value < scan.range_min or value > effective_range_max:
                output.append(math.inf)
            else:
                output.append(value)

        return LaserScanSample(
            sim_time_s=scan.sim_time_s,
            angle_min=scan.angle_min,
            angle_increment=scan.angle_increment,
            range_min=scan.range_min,
            range_max=effective_range_max,
            ranges=tuple(output),
        )
