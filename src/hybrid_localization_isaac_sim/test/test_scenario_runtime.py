from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_ROOT / "scripts" / "scenario_runtime.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("scenario_runtime", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


runtime = _load_module()


def _base_scenario():
    return {
        "id": "T00",
        "world_scenario": "S00",
        "seed": 123,
        "robot": {"initial_pose": {"x": 1.0, "y": -2.0, "yaw": 0.5}},
        "localization": {
            "mode": "known_pose",
            "initial_pose": {"x": 0.8, "y": -1.9, "yaw": 0.4},
            "xy_stddev": 0.25,
            "yaw_stddev": 0.1,
        },
    }


def test_committed_runtime_catalog_preserves_one_fault_free_baseline_per_world():
    catalog = runtime.load_catalog()
    world_ids = runtime._load_world_ids(runtime.DEFAULT_WORLD_CATALOG)

    assert world_ids.issubset(catalog)
    for world_id in world_ids:
        baseline = catalog[world_id]
        assert baseline["world_scenario"] == world_id
        assert runtime.scenario_faults(baseline) == ()


def test_committed_stage2_kidnapped_scenario_is_deterministic_and_separate_from_baseline():
    catalog = runtime.load_catalog()
    scenario = catalog["S3_KIDNAPPED"]

    assert scenario["world_scenario"] == "S08"
    assert catalog["S08"].get("faults") is None
    scheduled = runtime.scenario_faults(scenario)
    assert len(scheduled) == 1
    assert scheduled[0].fault_id == "kidnap_1"
    assert scheduled[0].fault_type.value == "kidnapped_robot"
    assert scheduled[0].start_time_s == pytest.approx(10.0)
    assert dict(scheduled[0].parameters) == {
        "x_offset_m": 2.0,
        "y_offset_m": 1.0,
        "yaw_offset_rad": 0.0,
    }


def test_physical_pose_and_known_pose_prior_are_independent():
    scenario = _base_scenario()
    runtime.validate_scenario(scenario, {"S00"})
    assert scenario["robot"]["initial_pose"] != scenario["localization"]["initial_pose"]


def test_global_and_random_prior_modes_are_valid():
    global_scenario = _base_scenario()
    global_scenario["localization"] = {"mode": "global"}
    runtime.validate_scenario(global_scenario, {"S00"})

    random_scenario = _base_scenario()
    random_scenario["localization"] = {
        "mode": "random_prior",
        "seed": 77,
        "xy_stddev": 2.0,
        "yaw_stddev": 1.0,
    }
    runtime.validate_scenario(random_scenario, {"S00"})


def test_unknown_world_reference_is_rejected():
    scenario = _base_scenario()
    with pytest.raises(runtime.ScenarioRuntimeError, match="unknown world"):
        runtime.validate_scenario(scenario, {"S01"})


def test_invalid_pose_and_invalid_seed_are_rejected():
    scenario = _base_scenario()
    scenario["seed"] = -1
    with pytest.raises(runtime.ScenarioRuntimeError, match="non-negative integer"):
        runtime.validate_scenario(scenario, {"S00"})

    scenario = _base_scenario()
    scenario["robot"]["initial_pose"]["yaw"] = math.pi
    with pytest.raises(runtime.ScenarioRuntimeError, match=r"\[-pi, pi\)"):
        runtime.validate_scenario(scenario, {"S00"})


def test_arbitrary_robot_pose_override_does_not_move_localization_prior_by_default():
    scenario = _base_scenario()
    original_prior = dict(scenario["localization"]["initial_pose"])
    updated = runtime.scenario_with_pose(scenario, x=3.0, y=4.0, yaw=-0.75)

    assert updated["robot"]["initial_pose"] == {"x": 3.0, "y": 4.0, "yaw": -0.75}
    assert updated["localization"]["initial_pose"] == original_prior
    assert scenario["robot"]["initial_pose"] == {"x": 1.0, "y": -2.0, "yaw": 0.5}


def test_arbitrary_pose_can_explicitly_update_known_pose_prior():
    scenario = _base_scenario()
    updated = runtime.scenario_with_pose(
        scenario,
        x=-3.0,
        y=2.0,
        yaw=1.25,
        localization_follows_robot=True,
    )
    assert updated["robot"]["initial_pose"] == updated["localization"]["initial_pose"]


def test_localization_follows_robot_rejects_non_known_pose_mode():
    scenario = _base_scenario()
    scenario["localization"] = {"mode": "global"}
    with pytest.raises(runtime.ScenarioRuntimeError, match="requires known_pose"):
        runtime.scenario_with_pose(
            scenario,
            x=0.0,
            y=0.0,
            yaw=0.0,
            localization_follows_robot=True,
        )


def test_committed_stage3_odometry_scenarios_cover_drift_bias_noise_and_freeze():
    catalog = runtime.load_catalog()
    expected = {
        "S4_ODOMETRY_DRIFT": {"linear_scale": 1.08, "angular_scale": 1.05},
        "S4_ODOMETRY_BIAS": {"linear_bias_m": 0.15, "angular_bias_rad": 0.05},
        "S4_ODOMETRY_NOISE": {
            "linear_noise_stddev_m": 0.02,
            "angular_noise_stddev_rad": 0.01,
        },
        "S4_ODOMETRY_FREEZE": {"freeze": True},
    }

    for scenario_id, parameters in expected.items():
        scenario = catalog[scenario_id]
        assert scenario["world_scenario"] == "S08"
        scheduled = runtime.scenario_faults(scenario)
        assert len(scheduled) == 1
        assert scheduled[0].fault_type.value == "odometry_degradation"
        assert scheduled[0].start_time_s == pytest.approx(10.0)
        assert dict(scheduled[0].parameters) == parameters

    assert runtime.scenario_faults(catalog["S08"]) == ()


def test_committed_stage4_lidar_scenarios_cover_supported_corruptions():
    catalog = runtime.load_catalog()
    expected = {
        "S5_LIDAR_NOISE": {"gaussian_noise_stddev_m": 0.05},
        "S5_LIDAR_DROPOUT": {"dropout_fraction": 0.30},
        "S5_LIDAR_OCCLUSION": {
            "sector_start_rad": -0.60,
            "sector_end_rad": 0.60,
        },
        "S5_LIDAR_REDUCED_RANGE": {"max_range_m": 3.0},
        "S5_LIDAR_OUTLIERS": {
            "outlier_fraction": 0.10,
            "outlier_min_m": 0.25,
            "outlier_max_m": 6.0,
        },
        "S5_LIDAR_SCAN_LOSS": {"complete_scan_loss": True},
    }

    for scenario_id, parameters in expected.items():
        scenario = catalog[scenario_id]
        assert scenario["world_scenario"] == "S08"
        scheduled = runtime.scenario_faults(scenario)
        assert len(scheduled) == 1
        assert scheduled[0].fault_type.value == "lidar_degradation"
        assert scheduled[0].start_time_s == pytest.approx(10.0)
        assert dict(scheduled[0].parameters) == parameters

    assert runtime.scenario_faults(catalog["S08"]) == ()


def test_committed_stage5_environment_scenarios_cover_map_mismatch_and_dynamic_obstruction():
    catalog = runtime.load_catalog()
    expected = {
        "S6_PARTIAL_MAP_MISMATCH": (
            "map_mismatch",
            0.0,
            {"severity": "partial", "variant": "added_box"},
            None,
        ),
        "S7_SEVERE_MAP_MISMATCH": (
            "map_mismatch",
            0.0,
            {"severity": "severe", "variant": "added_wall"},
            None,
        ),
        "S8_DYNAMIC_OBSTRUCTION": (
            "dynamic_obstruction",
            10.0,
            {"profile": "front_box"},
            20.0,
        ),
    }

    for scenario_id, (fault_type, start_s, parameters, end_s) in expected.items():
        scenario = catalog[scenario_id]
        assert scenario["world_scenario"] == "S08"
        scheduled = runtime.scenario_faults(scenario)
        assert len(scheduled) == 1
        fault = scheduled[0]
        assert fault.fault_type.value == fault_type
        assert fault.start_time_s == pytest.approx(start_s)
        assert dict(fault.parameters) == parameters
        if end_s is None:
            assert fault.end_time_s is None
        else:
            assert fault.end_time_s == pytest.approx(end_s)

    assert runtime.scenario_faults(catalog["S08"]) == ()


def test_committed_stage6_representatives_cover_s0_s1_s2_and_combined_s9():
    catalog = runtime.load_catalog()

    assert catalog["S0_BASELINE"]["world_scenario"] == "S08"
    assert catalog["S0_BASELINE"]["localization"]["mode"] == "known_pose"
    assert runtime.scenario_faults(catalog["S0_BASELINE"]) == ()

    assert catalog["S1_GLOBAL_INITIALIZATION"]["world_scenario"] == "S08"
    assert catalog["S1_GLOBAL_INITIALIZATION"]["localization"]["mode"] == "global"
    assert runtime.scenario_faults(catalog["S1_GLOBAL_INITIALIZATION"]) == ()

    assert catalog["S2_MULTIMODAL_SYMMETRIC"]["world_scenario"] == "S07"
    assert catalog["S2_MULTIMODAL_SYMMETRIC"]["localization"]["mode"] == "global"
    assert runtime.scenario_faults(catalog["S2_MULTIMODAL_SYMMETRIC"]) == ()

    combined = runtime.scenario_faults(catalog["S9_COMBINED"])
    assert len(combined) == 2
    assert {fault.fault_type.value for fault in combined} == {
        "odometry_degradation",
        "lidar_degradation",
    }
    assert {fault.fault_id for fault in combined} == {
        "combined_odom_1",
        "combined_lidar_1",
    }
    assert all(fault.start_time_s == pytest.approx(10.0) for fault in combined)
    assert all(fault.end_time_s == pytest.approx(30.0) for fault in combined)
