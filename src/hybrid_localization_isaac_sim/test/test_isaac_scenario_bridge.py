import importlib.util
import json
import math
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "isaac_scenario_bridge", ROOT / "scripts" / "isaac_scenario_bridge.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_parse_command_without_override():
    command = MODULE.parse_command('{"request_id":"abc","scenario_id":"S07"}')
    assert command.request_id == "abc"
    assert command.scenario_id == "S07"
    assert command.robot_pose_override is None
    assert command.action == "apply"


def test_parse_command_with_override():
    command = MODULE.parse_command(
        '{"request_id":"abc","scenario_id":"S07","robot_pose_override":[1.0,2.0,-0.5]}'
    )
    assert command.robot_pose_override == (1.0, 2.0, -0.5)
    assert command.action == "apply"


def test_parse_activate_command():
    command = MODULE.parse_command(
        '{"request_id":"abc","scenario_id":"S07","action":"activate"}'
    )
    assert command.action == "activate"
    assert command.robot_pose_override is None


@pytest.mark.parametrize(
    "payload",
    [
        "not-json",
        "[]",
        '{"scenario_id":"S07"}',
        '{"request_id":"abc"}',
        '{"request_id":"abc","scenario_id":"S07","robot_pose_override":[1,2]}',
        '{"request_id":"abc","scenario_id":"S07","robot_pose_override":[1,2,3.141592653589793]}',
        '{"request_id":"abc","scenario_id":"S07","action":"invalid"}',
    ],
)
def test_parse_command_rejects_invalid_payloads(payload):
    with pytest.raises(MODULE.IsaacScenarioBridgeError):
        MODULE.parse_command(payload)


def test_status_payload_is_correlated_and_contains_observed_pose():
    command = MODULE.ScenarioCommand("req-1", "S03")
    value = json.loads(
        MODULE.status_payload(
            command,
            state="ready",
            detail="ok",
            observed_pose=(1.0, 2.0, -0.2),
        )
    )
    assert value["request_id"] == "req-1"
    assert value["scenario_id"] == "S03"
    assert value["state"] == "ready"
    assert value["action"] == "apply"
    assert value["observed_pose"] == {"x": 1.0, "y": 2.0, "yaw": -0.2}


def test_dynamic_loader_registers_dataclass_module(tmp_path):
    helper = tmp_path / "helper.py"
    helper.write_text(
        "from dataclasses import dataclass\n"
        "@dataclass(frozen=True)\n"
        "class Value:\n"
        "    number: int\n",
        encoding="utf-8",
    )
    loaded = MODULE._load_module("stage3_test_dataclass_helper", helper)
    assert loaded.Value(7).number == 7


def test_kidnapped_target_pose_applies_map_frame_offset_and_wraps_yaw():
    target = MODULE.kidnapped_target_pose(
        (1.0, -2.0, math.pi - 0.1),
        {
            "x_offset_m": 2.0,
            "y_offset_m": 1.0,
            "yaw_offset_rad": 0.2,
        },
    )

    assert target[0] == pytest.approx(3.0)
    assert target[1] == pytest.approx(-1.0)
    assert target[2] == pytest.approx(-math.pi + 0.1)


def test_fault_status_payload_contains_reproducibility_and_sim_time_fields():
    fault_spec = type(
        "FaultSpecification",
        (),
        {
            "fault_id": "kidnap_1",
            "fault_type": type("FaultType", (), {"value": "kidnapped_robot"})(),
            "start_time_s": 10.0,
            "seed": 1777,
            "parameters": {
                "x_offset_m": 2.0,
                "y_offset_m": 1.0,
                "yaw_offset_rad": 0.0,
            },
        },
    )()
    scheduled = type(
        "ScheduledFault",
        (),
        {
            "specification": fault_spec,
            "state": type("FaultState", (), {"value": "completed"})(),
            "activation_sim_time_s": 52.0,
            "completion_sim_time_s": 52.15,
            "error_detail": None,
        },
    )()

    value = json.loads(
        MODULE.fault_status_payload(
            scenario_id="S3_KIDNAPPED",
            scheduled_fault=scheduled,
            scenario_activation_sim_time_s=42.0,
            detail="stable",
        )
    )

    assert value["scenario_id"] == "S3_KIDNAPPED"
    assert value["fault_id"] == "kidnap_1"
    assert value["fault_type"] == "kidnapped_robot"
    assert value["state"] == "completed"
    assert value["scheduled_elapsed_time_s"] == pytest.approx(10.0)
    assert value["scheduled_sim_time_s"] == pytest.approx(52.0)
    assert value["activation_sim_time_s"] == pytest.approx(52.0)
    assert value["completion_sim_time_s"] == pytest.approx(52.15)
    assert value["seed"] == 1777
    assert value["detail"] == "stable"


def test_runtime_active_payload_includes_simulation_activation_time():
    value = json.loads(
        MODULE.active_scenario_payload(
            {"scenario_id": "S4_ODOMETRY_DRIFT", "seed": 1777},
            42.125,
        )
    )
    assert value["scenario_id"] == "S4_ODOMETRY_DRIFT"
    assert value["state"] == "active"
    assert value["activation_sim_time_s"] == pytest.approx(42.125)

    with pytest.raises(MODULE.IsaacScenarioBridgeError):
        MODULE.active_scenario_payload({"scenario_id": "S04"}, -1.0)
