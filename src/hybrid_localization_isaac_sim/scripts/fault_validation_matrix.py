#!/usr/bin/env python3
"""ROS-independent #44 Stage-6 S0-S9 live-validation contract.

This module deliberately stops short of #45 automation.  It defines one
representative runtime scenario per failure family plus deterministic live
acceptance expectations that a human-driven Stage-6 campaign can validate.
"""

from __future__ import annotations

from types import MappingProxyType
from typing import Any, Iterable, Mapping, NamedTuple


class FaultValidationError(ValueError):
    pass


class FamilyValidationCase(NamedTuple):
    family: str
    scenario_id: str
    description: str
    expected_fault_states: Mapping[str, str]
    benchmark_duration_s: float
    requires_motion: bool
    require_multimodal_particles: bool = False


_CASES = (
    FamilyValidationCase(
        "S0", "S0_BASELINE", "Nominal known-pose tracking in the S08 warehouse fixture.",
        MappingProxyType({}), 20.0, True,
    ),
    FamilyValidationCase(
        "S1", "S1_GLOBAL_INITIALIZATION", "Unknown initial pose / AMCL global initialization.",
        MappingProxyType({}), 30.0, True,
    ),
    FamilyValidationCase(
        "S2", "S2_MULTIMODAL_SYMMETRIC", "Corridor-heavy global initialization used to expose plausible competing modes.",
        MappingProxyType({}), 30.0, True, True,
    ),
    FamilyValidationCase(
        "S3", "S3_KIDNAPPED", "Instantaneous physical displacement without localization reset.",
        MappingProxyType({"kidnap_1": "completed"}), 35.0, True,
    ),
    FamilyValidationCase(
        "S4", "S4_ODOMETRY_DRIFT", "Deterministic accumulated odometry scale drift.",
        MappingProxyType({"odom_drift_1": "completed"}), 35.0, True,
    ),
    FamilyValidationCase(
        "S5", "S5_LIDAR_DROPOUT", "Deterministic 30% LiDAR beam dropout.",
        MappingProxyType({"lidar_dropout_1": "completed"}), 35.0, True,
    ),
    FamilyValidationCase(
        "S6", "S6_PARTIAL_MAP_MISMATCH", "Persistent localized physical/map disagreement.",
        MappingProxyType({"map_partial_1": "active"}), 25.0, True,
    ),
    FamilyValidationCase(
        "S7", "S7_SEVERE_MAP_MISMATCH", "Persistent severe structural physical/map disagreement.",
        MappingProxyType({"map_severe_1": "active"}), 25.0, True,
    ),
    FamilyValidationCase(
        "S8", "S8_DYNAMIC_OBSTRUCTION", "Temporary physical obstruction inserted and removed in simulation time.",
        MappingProxyType({"dynamic_front_1": "completed"}), 30.0, True,
    ),
    FamilyValidationCase(
        "S9", "S9_COMBINED", "Overlapping moderate odometry drift and partial LiDAR corruption.",
        MappingProxyType({"combined_odom_1": "completed", "combined_lidar_1": "completed"}), 35.0, True,
    ),
)


def validation_cases() -> tuple[FamilyValidationCase, ...]:
    return _CASES


def case_for_scenario(scenario_id: str) -> FamilyValidationCase:
    matches = [case for case in _CASES if case.scenario_id == scenario_id]
    if len(matches) != 1:
        raise FaultValidationError(f"scenario {scenario_id!r} is not a Stage-6 representative case")
    return matches[0]


def validate_matrix(catalog: Mapping[str, Mapping[str, Any]], scenario_faults) -> None:
    """Validate that committed scenarios still implement the Stage-6 matrix."""
    families = [case.family for case in _CASES]
    if families != [f"S{i}" for i in range(10)]:
        raise FaultValidationError("Stage-6 matrix must cover S0 through S9 exactly once")

    scenario_ids = [case.scenario_id for case in _CASES]
    if len(set(scenario_ids)) != len(scenario_ids):
        raise FaultValidationError("Stage-6 representative scenario IDs must be unique")

    for case in _CASES:
        if case.scenario_id not in catalog:
            raise FaultValidationError(f"missing committed scenario {case.scenario_id}")
        faults = scenario_faults(catalog[case.scenario_id])
        actual_ids = {fault.fault_id for fault in faults}
        expected_ids = set(case.expected_fault_states)
        if actual_ids != expected_ids:
            raise FaultValidationError(
                f"{case.scenario_id} fault IDs mismatch: expected {sorted(expected_ids)}, got {sorted(actual_ids)}"
            )


def parse_fault_status(payload: str, expected_scenario: str) -> dict[str, Any]:
    import json

    try:
        value = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise FaultValidationError("fault status is not valid JSON") from exc
    if not isinstance(value, dict):
        raise FaultValidationError("fault status must be a JSON object")
    if value.get("scenario_id") != expected_scenario:
        raise FaultValidationError("fault status belongs to a different scenario")
    fault_id = value.get("fault_id")
    state = value.get("state")
    if not isinstance(fault_id, str) or not fault_id:
        raise FaultValidationError("fault status requires a non-empty fault_id")
    if state not in {"scheduled", "active", "completed", "error"}:
        raise FaultValidationError(f"unsupported fault state {state!r}")
    return value


def update_observed_statuses(
    observed: dict[str, dict[str, Any]], payload: str, expected_scenario: str
) -> None:
    value = parse_fault_status(payload, expected_scenario)
    fault_id = value["fault_id"]
    previous = observed.get(fault_id)
    order = {"scheduled": 0, "active": 1, "completed": 2, "error": 3}
    if previous is None or order[value["state"]] >= order[previous["state"]]:
        observed[fault_id] = value


def validate_observed_statuses(
    case: FamilyValidationCase,
    observed: Mapping[str, Mapping[str, Any]],
) -> None:
    unexpected = set(observed).difference(case.expected_fault_states)
    if unexpected:
        raise FaultValidationError(
            f"unexpected fault status for {case.scenario_id}: {', '.join(sorted(unexpected))}"
        )
    for fault_id, expected_state in case.expected_fault_states.items():
        value = observed.get(fault_id)
        if value is None:
            raise FaultValidationError(f"no status observed for fault {fault_id}")
        if value.get("state") == "error":
            raise FaultValidationError(
                f"fault {fault_id} entered error: {value.get('error') or value.get('detail') or 'unknown'}"
            )
        if value.get("state") != expected_state:
            raise FaultValidationError(
                f"fault {fault_id} expected {expected_state}, got {value.get('state')!r}"
            )


def validate_gateway_publishers(topic_publishers: Mapping[str, Iterable[str]]) -> None:
    expected = {
        "/odom": "hybrid_localization_odometry_fault_injector",
        "/scan": "hybrid_localization_lidar_fault_injector",
    }
    for topic, owner in expected.items():
        names = {name.lstrip("/") for name in topic_publishers.get(topic, ())}
        if names != {owner}:
            raise FaultValidationError(
                f"{topic} must have exactly one publisher {owner!r}, got {sorted(names)}"
            )


def validate_ground_truth_stamps(stamps_ns: Iterable[int], *, minimum_samples: int = 2) -> None:
    values = list(stamps_ns)
    if len(values) < minimum_samples:
        raise FaultValidationError(
            f"ground truth needs at least {minimum_samples} samples, got {len(values)}"
        )
    if any(value < 0 for value in values):
        raise FaultValidationError("ground-truth timestamps must be non-negative")
    if any(b <= a for a, b in zip(values, values[1:])):
        raise FaultValidationError("ground-truth timestamps must increase strictly")
