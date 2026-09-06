from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_ROOT / "scripts" / "fault_specification.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("fault_specification", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


faults = _load_module()


def _kidnap(**overrides):
    value = {
        "id": "kidnap_1",
        "type": "kidnapped_robot",
        "start_time_s": 10.0,
        "seed": 1777,
        "parameters": {
            "x_offset_m": 2.0,
            "y_offset_m": 1.0,
            "yaw_offset_rad": 0.0,
        },
    }
    value.update(overrides)
    return value


def test_kidnapped_robot_parses_to_stable_immutable_contract():
    parsed = faults.parse_fault(_kidnap())
    assert parsed.fault_id == "kidnap_1"
    assert parsed.fault_type is faults.FaultType.KIDNAPPED_ROBOT
    assert parsed.start_time_s == 10.0
    assert parsed.end_time_s is None
    assert parsed.seed == 1777
    assert list(parsed.parameters) == ["x_offset_m", "y_offset_m", "yaw_offset_rad"]


def test_duration_is_canonicalized_to_end_time():
    parsed = faults.parse_fault(
        {
            "id": "odom_1",
            "type": "odometry_degradation",
            "start_time_s": 5,
            "duration_s": 3.5,
            "seed": 10,
            "parameters": {"linear_scale": 1.04},
        }
    )
    assert parsed.start_time_s == 5.0
    assert parsed.end_time_s == 8.5
    assert parsed.duration_s == 3.5


def test_end_time_is_supported_but_cannot_be_combined_with_duration():
    parsed = faults.parse_fault(
        {
            "id": "lidar_1",
            "type": "lidar_degradation",
            "start_time_s": 2,
            "end_time_s": 7,
            "seed": 11,
            "parameters": {"dropout_fraction": 0.2},
        }
    )
    assert parsed.duration_s == 5.0

    invalid = dict(_kidnap())
    invalid["duration_s"] = 1.0
    invalid["end_time_s"] = 2.0
    with pytest.raises(faults.FaultSpecificationError, match="at most one"):
        faults.parse_fault(invalid)


def test_duplicate_fault_ids_are_rejected():
    with pytest.raises(faults.FaultSpecificationError, match="duplicate fault ID"):
        faults.parse_faults([_kidnap(), _kidnap()])


@pytest.mark.parametrize("field,value", [("start_time_s", -1), ("seed", -1), ("start_time_s", float("nan"))])
def test_invalid_time_or_seed_is_rejected(field, value):
    invalid = _kidnap(**{field: value})
    with pytest.raises(faults.FaultSpecificationError):
        faults.parse_fault(invalid)


def test_kidnapped_robot_rejects_zero_displacement_and_duration():
    invalid = _kidnap()
    invalid["parameters"] = {"x_offset_m": 0.0, "y_offset_m": 0.0, "yaw_offset_rad": 0.0}
    with pytest.raises(faults.FaultSpecificationError, match="non-zero"):
        faults.parse_fault(invalid)

    invalid = _kidnap(duration_s=1.0)
    with pytest.raises(faults.FaultSpecificationError, match="instantaneous"):
        faults.parse_fault(invalid)


def test_odometry_parameters_are_validated():
    valid = {
        "id": "odom",
        "type": "odometry_degradation",
        "start_time_s": 1.0,
        "seed": 12,
        "parameters": {
            "linear_scale": 1.05,
            "angular_bias_rad": 0.01,
            "linear_noise_stddev_m": 0.02,
        },
    }
    faults.parse_fault(valid)

    invalid = dict(valid)
    invalid["parameters"] = {"linear_scale": 0.0}
    with pytest.raises(faults.FaultSpecificationError, match="linear_scale"):
        faults.parse_fault(invalid)


def test_lidar_parameters_are_validated():
    faults.parse_fault(
        {
            "id": "scan",
            "type": "lidar_degradation",
            "start_time_s": 2.0,
            "duration_s": 5.0,
            "seed": 13,
            "parameters": {
                "gaussian_noise_stddev_m": 0.03,
                "dropout_fraction": 0.2,
                "sector_start_rad": -0.5,
                "sector_end_rad": 0.5,
                "outlier_fraction": 0.05,
                "outlier_min_m": 0.2,
                "outlier_max_m": 10.0,
            },
        }
    )

    with pytest.raises(faults.FaultSpecificationError, match="both sector_start_rad and sector_end_rad"):
        faults.parse_fault(
            {
                "id": "scan",
                "type": "lidar_degradation",
                "start_time_s": 2.0,
                "seed": 13,
                "parameters": {"sector_start_rad": -0.5},
            }
        )


def test_map_mismatch_and_dynamic_obstruction_contracts_are_explicit():
    map_fault = faults.parse_fault(
        {
            "id": "map_partial",
            "type": "map_mismatch",
            "start_time_s": 0.0,
            "seed": 14,
            "parameters": {"severity": "partial", "variant": "missing_pallet_01"},
        }
    )
    assert map_fault.parameters["severity"] == "partial"

    obstruction = faults.parse_fault(
        {
            "id": "crossing_object",
            "type": "dynamic_obstruction",
            "start_time_s": 10.0,
            "duration_s": 5.0,
            "seed": 15,
            "parameters": {"profile": "cross_corridor_once"},
        }
    )
    assert obstruction.duration_s == 5.0

    with pytest.raises(faults.FaultSpecificationError, match="must define duration"):
        faults.parse_fault(
            {
                "id": "crossing_object",
                "type": "dynamic_obstruction",
                "start_time_s": 10.0,
                "seed": 15,
                "parameters": {"profile": "cross_corridor_once"},
            }
        )


def test_combined_odometry_and_lidar_faults_are_supported():
    parsed = faults.parse_faults(
        [
            {
                "id": "odom",
                "type": "odometry_degradation",
                "start_time_s": 5.0,
                "duration_s": 20.0,
                "seed": 16,
                "parameters": {"linear_scale": 1.04},
            },
            {
                "id": "scan",
                "type": "lidar_degradation",
                "start_time_s": 10.0,
                "duration_s": 15.0,
                "seed": 17,
                "parameters": {"dropout_fraction": 0.2},
            },
        ]
    )
    assert [fault.fault_id for fault in parsed] == ["odom", "scan"]


def test_overlapping_same_channel_faults_are_rejected_until_composition_order_is_defined():
    one = {
        "id": "odom_a",
        "type": "odometry_degradation",
        "start_time_s": 5.0,
        "duration_s": 10.0,
        "seed": 18,
        "parameters": {"linear_scale": 1.03},
    }
    two = {
        "id": "odom_b",
        "type": "odometry_degradation",
        "start_time_s": 10.0,
        "duration_s": 10.0,
        "seed": 19,
        "parameters": {"angular_scale": 1.02},
    }
    with pytest.raises(faults.FaultSpecificationError, match="overlapping odometry_degradation"):
        faults.parse_faults([one, two])


def _load_runtime_module():
    script = PACKAGE_ROOT / "scripts" / "scenario_runtime.py"
    spec = importlib.util.spec_from_file_location("scenario_runtime_with_faults", script)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_runtime_scenario_accepts_optional_fault_schedule_without_changing_canonical_baselines():
    runtime = _load_runtime_module()
    scenario = {
        "id": "T44",
        "world_scenario": "S08",
        "seed": 1777,
        "robot": {"initial_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}},
        "localization": {
            "mode": "known_pose",
            "initial_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "xy_stddev": 0.25,
            "yaw_stddev": 0.1,
        },
        "faults": [_kidnap()],
    }
    runtime.validate_scenario(scenario, {"S08"})
    parsed = runtime.scenario_faults(scenario)
    assert len(parsed) == 1
    assert parsed[0].fault_id == "kidnap_1"

    committed = runtime.load_catalog()
    world_ids = runtime._load_world_ids(runtime.DEFAULT_WORLD_CATALOG)
    assert all(runtime.scenario_faults(committed[world_id]) == () for world_id in world_ids)


def test_runtime_scenario_surfaces_fault_validation_errors():
    runtime = _load_runtime_module()
    scenario = {
        "id": "T44",
        "world_scenario": "S08",
        "seed": 1777,
        "robot": {"initial_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}},
        "localization": {"mode": "global"},
        "faults": [_kidnap(start_time_s=-1.0)],
    }
    with pytest.raises(runtime.ScenarioRuntimeError, match="invalid scenario fault configuration"):
        runtime.validate_scenario(scenario, {"S08"})
