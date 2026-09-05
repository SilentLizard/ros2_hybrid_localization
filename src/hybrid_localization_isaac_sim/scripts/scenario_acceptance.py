#!/usr/bin/env python3
"""Live #43 acceptance validator for the synchronized Isaac/ROS scenario state.

The module keeps validation helpers ROS-independent so normal pytest can cover
payload, pose, topic, lifecycle, and TF-authority rules without Isaac Sim.
ROS imports are delayed until ``LiveScenarioAcceptance`` is constructed.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from typing import Any, Iterable


ACTIVE_SCENARIO_TOPIC = "/hybrid_localization/active_scenario"
GROUND_TRUTH_TOPIC = "/hybrid_localization/ground_truth/pose"
GROUND_TRUTH_FRAME = "map"

REQUIRED_TOPICS = (
    "/clock",
    "/scan",
    "/odom",
    "/tf",
    "/tf_static",
    "/particle_cloud",
    "/hybrid_localization/particle_analysis",
    "/hybrid_localization/rviz_markers",
    GROUND_TRUTH_TOPIC,
    ACTIVE_SCENARIO_TOPIC,
)

FORBIDDEN_TF_NODES = {
    "hybrid_localization_isaac_scenario_bridge",
    "hybrid_localization_scenario_controller",
    "hybrid_localization_scenario_acceptance",
    "particle_analysis_observer",
    "particle_analysis_visualization",
}


class ScenarioAcceptanceError(RuntimeError):
    pass


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class GroundTruthSample:
    frame_id: str
    stamp_ns: int
    pose: Pose2


def normalize_angle(angle: float) -> float:
    value = math.fmod(angle + math.pi, 2.0 * math.pi)
    if value < 0.0:
        value += 2.0 * math.pi
    return value - math.pi


def yaw_error(a: float, b: float) -> float:
    return abs(normalize_angle(a - b))


def parse_active_scenario(payload: str, expected_scenario: str) -> dict[str, Any]:
    try:
        value = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise ScenarioAcceptanceError("active_scenario is not valid JSON") from exc
    if not isinstance(value, dict):
        raise ScenarioAcceptanceError("active_scenario must be a JSON object")
    if value.get("scenario_id") != expected_scenario:
        raise ScenarioAcceptanceError(
            f"active scenario mismatch: expected {expected_scenario!r}, got {value.get('scenario_id')!r}"
        )
    if value.get("state") != "active":
        raise ScenarioAcceptanceError(
            f"scenario is not active: state={value.get('state')!r}"
        )
    return value


def expected_pose_from_active(active: dict[str, Any]) -> Pose2:
    value = active.get("isaac_observed_pose")
    if not isinstance(value, dict):
        raise ScenarioAcceptanceError("active scenario has no Isaac observed pose")
    try:
        pose = Pose2(float(value["x"]), float(value["y"]), float(value["yaw"]))
    except (KeyError, TypeError, ValueError) as exc:
        raise ScenarioAcceptanceError("invalid Isaac observed pose in active scenario") from exc
    if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw)):
        raise ScenarioAcceptanceError("active scenario pose must be finite")
    return pose


def validate_ground_truth(
    sample: GroundTruthSample,
    expected: Pose2,
    *,
    position_tolerance: float,
    yaw_tolerance: float,
) -> tuple[float, float]:
    if position_tolerance < 0.0 or yaw_tolerance < 0.0:
        raise ScenarioAcceptanceError("ground-truth tolerances must be non-negative")
    if sample.frame_id != GROUND_TRUTH_FRAME:
        raise ScenarioAcceptanceError(
            f"ground truth frame must be {GROUND_TRUTH_FRAME!r}, got {sample.frame_id!r}"
        )
    if sample.stamp_ns < 0:
        raise ScenarioAcceptanceError("ground truth timestamp must be non-negative")
    if not all(math.isfinite(v) for v in (sample.pose.x, sample.pose.y, sample.pose.yaw)):
        raise ScenarioAcceptanceError("ground truth pose must be finite")

    position_error = math.hypot(sample.pose.x - expected.x, sample.pose.y - expected.y)
    angle_error = yaw_error(sample.pose.yaw, expected.yaw)
    if position_error > position_tolerance:
        raise ScenarioAcceptanceError(
            f"ground truth position error {position_error:.6g} m exceeds {position_tolerance:.6g} m"
        )
    if angle_error > yaw_tolerance:
        raise ScenarioAcceptanceError(
            f"ground truth yaw error {angle_error:.6g} rad exceeds {yaw_tolerance:.6g} rad"
        )
    return position_error, angle_error


def missing_required_topics(available: Iterable[str]) -> list[str]:
    present = set(available)
    return [topic for topic in REQUIRED_TOPICS if topic not in present]


def validate_tf_publishers(publishers: Iterable[str]) -> None:
    names = {name.lstrip("/") for name in publishers}
    forbidden = sorted(names & FORBIDDEN_TF_NODES)
    if forbidden:
        raise ScenarioAcceptanceError(
            "hybrid scenario/observation nodes must not publish /tf: " + ", ".join(forbidden)
        )
    if "amcl" not in names:
        raise ScenarioAcceptanceError("/tf publisher set does not include AMCL")


def validate_lifecycle_states(states: dict[str, str]) -> None:
    for node in ("/map_server", "/amcl"):
        state = states.get(node)
        if state != "active":
            raise ScenarioAcceptanceError(f"{node} lifecycle state must be active, got {state!r}")


class LiveScenarioAcceptance:
    def __init__(self, expected_scenario: str) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from lifecycle_msgs.srv import GetState
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self.rclpy = rclpy
        self.GetState = GetState
        self.node = Node("hybrid_localization_scenario_acceptance")
        self.expected_scenario = expected_scenario
        self.active_payload: str | None = None
        self.ground_truth: GroundTruthSample | None = None

        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.active_sub = self.node.create_subscription(
            String, ACTIVE_SCENARIO_TOPIC, self._active_cb, transient
        )
        self.gt_sub = self.node.create_subscription(
            PoseStamped, GROUND_TRUTH_TOPIC, self._gt_cb, 10
        )

    def _active_cb(self, msg) -> None:
        self.active_payload = msg.data

    def _gt_cb(self, msg) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        self.ground_truth = GroundTruthSample(
            frame_id=msg.header.frame_id,
            stamp_ns=stamp_ns,
            pose=Pose2(float(msg.pose.position.x), float(msg.pose.position.y), yaw),
        )

    def _wait_for_samples(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.active_payload is not None and self.ground_truth is not None:
                return
        raise ScenarioAcceptanceError("timed out waiting for active scenario and ground truth")

    def _lifecycle_state(self, node_name: str, timeout: float) -> str:
        client = self.node.create_client(self.GetState, f"{node_name}/get_state")
        if not client.wait_for_service(timeout_sec=timeout):
            raise ScenarioAcceptanceError(f"timed out waiting for {node_name}/get_state")
        future = client.call_async(self.GetState.Request())
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done() or future.exception() is not None:
            raise ScenarioAcceptanceError(f"failed to read lifecycle state for {node_name}")
        return str(future.result().current_state.label)

    def validate(self, *, timeout: float, position_tolerance: float, yaw_tolerance: float) -> dict[str, Any]:
        self._wait_for_samples(timeout)
        active = parse_active_scenario(self.active_payload or "", self.expected_scenario)
        expected = expected_pose_from_active(active)
        position_error, angle_error = validate_ground_truth(
            self.ground_truth,
            expected,
            position_tolerance=position_tolerance,
            yaw_tolerance=yaw_tolerance,
        )

        topics = self.node.get_topic_names_and_types()
        missing = missing_required_topics(name for name, _types in topics)
        if missing:
            raise ScenarioAcceptanceError("missing required topics: " + ", ".join(missing))

        tf_publishers = [info.node_name for info in self.node.get_publishers_info_by_topic("/tf")]
        validate_tf_publishers(tf_publishers)

        states = {
            "/map_server": self._lifecycle_state("/map_server", timeout),
            "/amcl": self._lifecycle_state("/amcl", timeout),
        }
        validate_lifecycle_states(states)

        return {
            "scenario_id": self.expected_scenario,
            "world_scenario": active.get("world_scenario"),
            "localization_mode": active.get("localization_mode"),
            "ground_truth_position_error_m": position_error,
            "ground_truth_yaw_error_rad": angle_error,
            "map_server_state": states["/map_server"],
            "amcl_state": states["/amcl"],
            "tf_publishers": sorted(tf_publishers),
            "result": "PASS",
        }

    def close(self) -> None:
        self.node.destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--position-tolerance", type=float, default=0.02)
    parser.add_argument("--yaw-tolerance", type=float, default=0.02)
    args = parser.parse_args()

    import rclpy

    rclpy.init()
    validator = LiveScenarioAcceptance(args.scenario)
    try:
        result = validator.validate(
            timeout=args.timeout,
            position_tolerance=args.position_tolerance,
            yaw_tolerance=args.yaw_tolerance,
        )
        print(json.dumps(result, indent=2, sort_keys=True))
    except ScenarioAcceptanceError as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(2) from exc
    finally:
        validator.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
