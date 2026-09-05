import importlib.util
import json
import math
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "scenario_acceptance", ROOT / "scripts" / "scenario_acceptance.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def active_payload():
    return json.dumps(
        {
            "state": "active",
            "scenario_id": "S07",
            "world_scenario": "S07",
            "localization_mode": "known_pose",
            "isaac_observed_pose": {"x": 3.0, "y": 4.0, "yaw": -0.75},
        }
    )


def test_parses_correlated_active_scenario_and_expected_pose():
    active = MODULE.parse_active_scenario(active_payload(), "S07")
    assert MODULE.expected_pose_from_active(active) == MODULE.Pose2(3.0, 4.0, -0.75)


def test_rejects_wrong_or_nonactive_scenario():
    with pytest.raises(MODULE.ScenarioAcceptanceError):
        MODULE.parse_active_scenario(active_payload(), "S03")
    value = json.loads(active_payload())
    value["state"] = "ready"
    with pytest.raises(MODULE.ScenarioAcceptanceError):
        MODULE.parse_active_scenario(json.dumps(value), "S07")


def test_ground_truth_accepts_small_error_and_wraparound():
    sample = MODULE.GroundTruthSample(
        "map", 42, MODULE.Pose2(3.005, 3.995, math.pi - 0.001)
    )
    expected = MODULE.Pose2(3.0, 4.0, -math.pi + 0.001)
    position_error, yaw_error = MODULE.validate_ground_truth(
        sample, expected, position_tolerance=0.01, yaw_tolerance=0.01
    )
    assert position_error < 0.01
    assert yaw_error == pytest.approx(0.002)


def test_ground_truth_rejects_wrong_frame_or_large_error():
    bad_frame = MODULE.GroundTruthSample("odom", 0, MODULE.Pose2(0.0, 0.0, 0.0))
    with pytest.raises(MODULE.ScenarioAcceptanceError):
        MODULE.validate_ground_truth(
            bad_frame, MODULE.Pose2(0.0, 0.0, 0.0), position_tolerance=0.1, yaw_tolerance=0.1
        )
    large = MODULE.GroundTruthSample("map", 0, MODULE.Pose2(1.0, 0.0, 0.0))
    with pytest.raises(MODULE.ScenarioAcceptanceError):
        MODULE.validate_ground_truth(
            large, MODULE.Pose2(0.0, 0.0, 0.0), position_tolerance=0.1, yaw_tolerance=0.1
        )


def test_required_topic_check_is_deterministic():
    missing = MODULE.missing_required_topics(MODULE.REQUIRED_TOPICS[:-2])
    assert missing == [MODULE.GROUND_TRUTH_TOPIC, MODULE.ACTIVE_SCENARIO_TOPIC]


def test_tf_authority_requires_amcl_and_rejects_hybrid_publishers():
    MODULE.validate_tf_publishers(["amcl", "ros2_publish_transform_tree"])
    with pytest.raises(MODULE.ScenarioAcceptanceError):
        MODULE.validate_tf_publishers(["ros2_publish_transform_tree"])
    with pytest.raises(MODULE.ScenarioAcceptanceError):
        MODULE.validate_tf_publishers(["amcl", "hybrid_localization_isaac_scenario_bridge"])


def test_lifecycle_states_must_be_active():
    MODULE.validate_lifecycle_states({"/map_server": "active", "/amcl": "active"})
    with pytest.raises(MODULE.ScenarioAcceptanceError):
        MODULE.validate_lifecycle_states({"/map_server": "active", "/amcl": "inactive"})
