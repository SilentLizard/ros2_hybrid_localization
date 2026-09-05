import importlib.util
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "scenario_controller", ROOT / "scripts" / "scenario_controller.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_command_payload_keeps_physical_override_explicit():
    payload = json.loads(
        MODULE.command_payload(
            "S07",
            request_id="req",
            robot_pose_override=(3.0, 4.0, -0.75),
        )
    )
    assert payload == {
        "request_id": "req",
        "scenario_id": "S07",
        "robot_pose_override": [3.0, 4.0, -0.75],
    }


def test_command_payload_omits_override_for_canonical_pose():
    payload = json.loads(
        MODULE.command_payload("S03", request_id="req", robot_pose_override=None)
    )
    assert payload == {"request_id": "req", "scenario_id": "S03"}


def test_matching_status_filters_unrelated_requests_and_invalid_json():
    assert MODULE.matching_status("not-json", "req") is None
    assert MODULE.matching_status('{"request_id":"other","state":"ready"}', "req") is None
    result = MODULE.matching_status('{"request_id":"req","state":"ready"}', "req")
    assert result == {"request_id": "req", "state": "ready"}


def test_map_yaml_is_derived_from_referenced_world_identity():
    assert MODULE.map_yaml_for_scenario(Path("/tmp/pkg"), "S14") == Path(
        "/tmp/pkg/generated/scenarios/S14/map.yaml"
    )


def test_activation_payload_is_explicit_and_keeps_request_correlation():
    payload = json.loads(
        MODULE.command_payload(
            "S07",
            request_id="req",
            robot_pose_override=None,
            action="activate",
        )
    )
    assert payload == {
        "request_id": "req",
        "scenario_id": "S07",
        "action": "activate",
    }
