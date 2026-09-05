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
