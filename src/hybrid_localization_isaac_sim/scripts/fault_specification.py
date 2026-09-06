#!/usr/bin/env python3
"""ROS-independent deterministic localization fault specification.

The module defines the Issue #44 Stage-1 contract only.  It deliberately has
no ROS 2, Isaac Sim, sensor-message, or simulator imports.  Runtime injectors
consume the validated result in later stages.
"""

from __future__ import annotations

from enum import Enum
import math
from types import MappingProxyType
from typing import Any, Mapping, NamedTuple, Sequence


class FaultSpecificationError(ValueError):
    """Raised when a fault definition violates the deterministic contract."""


class FaultType(str, Enum):
    KIDNAPPED_ROBOT = "kidnapped_robot"
    ODOMETRY_DEGRADATION = "odometry_degradation"
    LIDAR_DEGRADATION = "lidar_degradation"
    MAP_MISMATCH = "map_mismatch"
    DYNAMIC_OBSTRUCTION = "dynamic_obstruction"


class FaultSpecification(NamedTuple):
    """Validated, immutable representation of one scheduled fault."""

    fault_id: str
    fault_type: FaultType
    start_time_s: float
    end_time_s: float | None
    seed: int
    parameters: Mapping[str, Any]

    @property
    def duration_s(self) -> float | None:
        if self.end_time_s is None:
            return None
        return self.end_time_s - self.start_time_s


_TIME_TOLERANCE_S = 1e-12


def _is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _require_object(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise FaultSpecificationError(f"{path} must be an object")
    return value


def _require_non_empty_string(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise FaultSpecificationError(f"{path} must be a non-empty string")
    return value


def _require_finite(value: Any, path: str) -> float:
    if not _is_finite_number(value):
        raise FaultSpecificationError(f"{path} must be finite")
    return float(value)


def _require_non_negative(value: Any, path: str) -> float:
    result = _require_finite(value, path)
    if result < 0.0:
        raise FaultSpecificationError(f"{path} must be >= 0")
    return result


def _require_positive(value: Any, path: str) -> float:
    result = _require_finite(value, path)
    if result <= 0.0:
        raise FaultSpecificationError(f"{path} must be > 0")
    return result


def _require_fraction(value: Any, path: str) -> float:
    result = _require_finite(value, path)
    if not 0.0 <= result <= 1.0:
        raise FaultSpecificationError(f"{path} must be in [0, 1]")
    return result


def _reject_unknown(parameters: dict[str, Any], allowed: set[str], path: str) -> None:
    unknown = set(parameters).difference(allowed)
    if unknown:
        raise FaultSpecificationError(
            f"{path} contains unsupported parameter(s): " + ", ".join(sorted(unknown))
        )


def _validate_kidnapped_robot(parameters: dict[str, Any], duration_s: float | None) -> None:
    path = "fault.parameters"
    required = {"x_offset_m", "y_offset_m", "yaw_offset_rad"}
    if set(parameters) != required:
        raise FaultSpecificationError(
            f"{path} for kidnapped_robot must contain exactly x_offset_m, y_offset_m, and yaw_offset_rad"
        )
    x = _require_finite(parameters["x_offset_m"], f"{path}.x_offset_m")
    y = _require_finite(parameters["y_offset_m"], f"{path}.y_offset_m")
    yaw = _require_finite(parameters["yaw_offset_rad"], f"{path}.yaw_offset_rad")
    if not -math.pi <= yaw < math.pi:
        raise FaultSpecificationError(f"{path}.yaw_offset_rad must be in [-pi, pi)")
    if x == 0.0 and y == 0.0 and yaw == 0.0:
        raise FaultSpecificationError("kidnapped_robot displacement must be non-zero")
    if duration_s is not None:
        raise FaultSpecificationError("kidnapped_robot is instantaneous and must not define a duration")


def _validate_odometry(parameters: dict[str, Any], _duration_s: float | None) -> None:
    path = "fault.parameters"
    allowed = {
        "linear_scale",
        "angular_scale",
        "linear_bias_m",
        "angular_bias_rad",
        "linear_noise_stddev_m",
        "angular_noise_stddev_rad",
        "freeze",
    }
    _reject_unknown(parameters, allowed, path)
    if not parameters:
        raise FaultSpecificationError("odometry_degradation requires at least one parameter")

    if "linear_scale" in parameters:
        _require_positive(parameters["linear_scale"], f"{path}.linear_scale")
    if "angular_scale" in parameters:
        _require_positive(parameters["angular_scale"], f"{path}.angular_scale")
    if "linear_bias_m" in parameters:
        _require_finite(parameters["linear_bias_m"], f"{path}.linear_bias_m")
    if "angular_bias_rad" in parameters:
        _require_finite(parameters["angular_bias_rad"], f"{path}.angular_bias_rad")
    if "linear_noise_stddev_m" in parameters:
        _require_non_negative(parameters["linear_noise_stddev_m"], f"{path}.linear_noise_stddev_m")
    if "angular_noise_stddev_rad" in parameters:
        _require_non_negative(parameters["angular_noise_stddev_rad"], f"{path}.angular_noise_stddev_rad")
    if "freeze" in parameters and not isinstance(parameters["freeze"], bool):
        raise FaultSpecificationError(f"{path}.freeze must be a boolean")


def _validate_lidar(parameters: dict[str, Any], _duration_s: float | None) -> None:
    path = "fault.parameters"
    allowed = {
        "gaussian_noise_stddev_m",
        "dropout_fraction",
        "sector_start_rad",
        "sector_end_rad",
        "max_range_m",
        "outlier_fraction",
        "outlier_min_m",
        "outlier_max_m",
        "complete_scan_loss",
    }
    _reject_unknown(parameters, allowed, path)
    if not parameters:
        raise FaultSpecificationError("lidar_degradation requires at least one parameter")

    if "gaussian_noise_stddev_m" in parameters:
        _require_non_negative(
            parameters["gaussian_noise_stddev_m"], f"{path}.gaussian_noise_stddev_m"
        )
    if "dropout_fraction" in parameters:
        _require_fraction(parameters["dropout_fraction"], f"{path}.dropout_fraction")
    if "max_range_m" in parameters:
        _require_positive(parameters["max_range_m"], f"{path}.max_range_m")
    if "outlier_fraction" in parameters:
        _require_fraction(parameters["outlier_fraction"], f"{path}.outlier_fraction")
    if "complete_scan_loss" in parameters and not isinstance(parameters["complete_scan_loss"], bool):
        raise FaultSpecificationError(f"{path}.complete_scan_loss must be a boolean")

    sector_keys = {"sector_start_rad", "sector_end_rad"}
    if bool(sector_keys.intersection(parameters)) and not sector_keys.issubset(parameters):
        raise FaultSpecificationError("LiDAR sector dropout requires both sector_start_rad and sector_end_rad")
    for key in sector_keys.intersection(parameters):
        angle = _require_finite(parameters[key], f"{path}.{key}")
        if not -math.pi <= angle < math.pi:
            raise FaultSpecificationError(f"{path}.{key} must be in [-pi, pi)")

    outlier_range_keys = {"outlier_min_m", "outlier_max_m"}
    if bool(outlier_range_keys.intersection(parameters)) and not outlier_range_keys.issubset(parameters):
        raise FaultSpecificationError("LiDAR outliers require both outlier_min_m and outlier_max_m")
    if outlier_range_keys.issubset(parameters):
        minimum = _require_non_negative(parameters["outlier_min_m"], f"{path}.outlier_min_m")
        maximum = _require_positive(parameters["outlier_max_m"], f"{path}.outlier_max_m")
        if maximum <= minimum:
            raise FaultSpecificationError("fault.parameters.outlier_max_m must be > outlier_min_m")


def _validate_map_mismatch(parameters: dict[str, Any], duration_s: float | None) -> None:
    path = "fault.parameters"
    if set(parameters) != {"severity", "variant"}:
        raise FaultSpecificationError(
            f"{path} for map_mismatch must contain exactly severity and variant"
        )
    if parameters["severity"] not in {"partial", "severe"}:
        raise FaultSpecificationError(f"{path}.severity must be 'partial' or 'severe'")
    _require_non_empty_string(parameters["variant"], f"{path}.variant")
    if duration_s is not None:
        raise FaultSpecificationError("map_mismatch is scenario-persistent and must not define a duration")


def _validate_dynamic_obstruction(parameters: dict[str, Any], duration_s: float | None) -> None:
    path = "fault.parameters"
    if set(parameters) != {"profile"}:
        raise FaultSpecificationError(f"{path} for dynamic_obstruction must contain exactly profile")
    _require_non_empty_string(parameters["profile"], f"{path}.profile")
    if duration_s is None:
        raise FaultSpecificationError("dynamic_obstruction must define duration_s or end_time_s")


_PARAMETER_VALIDATORS = {
    FaultType.KIDNAPPED_ROBOT: _validate_kidnapped_robot,
    FaultType.ODOMETRY_DEGRADATION: _validate_odometry,
    FaultType.LIDAR_DEGRADATION: _validate_lidar,
    FaultType.MAP_MISMATCH: _validate_map_mismatch,
    FaultType.DYNAMIC_OBSTRUCTION: _validate_dynamic_obstruction,
}


def parse_fault(value: Any) -> FaultSpecification:
    """Validate and canonicalize one JSON-compatible fault object."""

    fault = _require_object(value, "fault")
    required = {"id", "type", "start_time_s", "seed", "parameters"}
    optional = {"duration_s", "end_time_s"}
    missing = required.difference(fault)
    if missing:
        raise FaultSpecificationError(
            "fault is missing required field(s): " + ", ".join(sorted(missing))
        )
    unknown = set(fault).difference(required | optional)
    if unknown:
        raise FaultSpecificationError(
            "fault contains unsupported field(s): " + ", ".join(sorted(unknown))
        )
    if "duration_s" in fault and "end_time_s" in fault:
        raise FaultSpecificationError("fault must define at most one of duration_s and end_time_s")

    fault_id = _require_non_empty_string(fault["id"], "fault.id")
    try:
        fault_type = FaultType(fault["type"])
    except (TypeError, ValueError):
        raise FaultSpecificationError(
            "fault.type must be one of: " + ", ".join(item.value for item in FaultType)
        ) from None

    start_time_s = _require_non_negative(fault["start_time_s"], "fault.start_time_s")
    end_time_s: float | None = None
    duration_s: float | None = None
    if "duration_s" in fault:
        duration_s = _require_positive(fault["duration_s"], "fault.duration_s")
        end_time_s = start_time_s + duration_s
        if not math.isfinite(end_time_s):
            raise FaultSpecificationError("fault end time must be finite")
    elif "end_time_s" in fault:
        end_time_s = _require_finite(fault["end_time_s"], "fault.end_time_s")
        if end_time_s <= start_time_s:
            raise FaultSpecificationError("fault.end_time_s must be > fault.start_time_s")
        duration_s = end_time_s - start_time_s

    seed = fault["seed"]
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise FaultSpecificationError("fault.seed must be a non-negative integer")

    parameters = _require_object(fault["parameters"], "fault.parameters")
    _PARAMETER_VALIDATORS[fault_type](parameters, duration_s)

    # Sorting gives a stable representation independent of JSON object key order.
    stable_parameters = MappingProxyType({key: parameters[key] for key in sorted(parameters)})
    return FaultSpecification(
        fault_id=fault_id,
        fault_type=fault_type,
        start_time_s=start_time_s,
        end_time_s=end_time_s,
        seed=seed,
        parameters=stable_parameters,
    )


def _intervals_overlap(left: FaultSpecification, right: FaultSpecification) -> bool:
    left_end = math.inf if left.end_time_s is None else left.end_time_s
    right_end = math.inf if right.end_time_s is None else right.end_time_s
    return (
        left.start_time_s < right_end - _TIME_TOLERANCE_S
        and right.start_time_s < left_end - _TIME_TOLERANCE_S
    )


def validate_fault_combination(faults: Sequence[FaultSpecification]) -> None:
    """Reject combinations whose composition order is deliberately undefined in Stage 1."""

    for index, left in enumerate(faults):
        for right in faults[index + 1 :]:
            if left.fault_type != right.fault_type:
                continue
            if left.fault_type not in {
                FaultType.ODOMETRY_DEGRADATION,
                FaultType.LIDAR_DEGRADATION,
            }:
                continue
            if _intervals_overlap(left, right):
                raise FaultSpecificationError(
                    f"overlapping {left.fault_type.value} faults are unsupported: "
                    f"{left.fault_id!r} and {right.fault_id!r}"
                )


def parse_faults(value: Any) -> tuple[FaultSpecification, ...]:
    """Parse a deterministic fault list, rejecting duplicate IDs and unsupported composition."""

    if not isinstance(value, list):
        raise FaultSpecificationError("scenario.faults must be an array")

    result: list[FaultSpecification] = []
    ids: set[str] = set()
    for index, raw_fault in enumerate(value):
        try:
            fault = parse_fault(raw_fault)
        except FaultSpecificationError as exc:
            raise FaultSpecificationError(f"scenario.faults[{index}]: {exc}") from exc
        if fault.fault_id in ids:
            raise FaultSpecificationError(f"duplicate fault ID {fault.fault_id!r}")
        ids.add(fault.fault_id)
        result.append(fault)

    validate_fault_combination(result)
    return tuple(result)
