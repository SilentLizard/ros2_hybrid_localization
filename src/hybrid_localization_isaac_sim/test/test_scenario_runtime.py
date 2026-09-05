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


def test_committed_runtime_catalog_matches_all_world_scenario_ids():
    catalog = runtime.load_catalog()
    world_ids = runtime._load_world_ids(runtime.DEFAULT_WORLD_CATALOG)
    assert len(catalog) == 20
    assert set(catalog) == world_ids


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
