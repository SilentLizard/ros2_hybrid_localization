#!/usr/bin/env python3
"""Host-side synchronized scenario controller for #43 Stage 3.

This command coordinates the already-running observation stack and the persistent
Isaac Script Editor bridge:

1. load/validate the Stage-1 runtime scenario;
2. materialize the referenced world if needed;
3. request Isaac to switch the collision world and reset HEROS;
4. load the matching occupancy map into Nav2 map_server;
5. reset AMCL according to the independent localization policy;
6. commit scenario activation only after host-side map and AMCL reset succeed.

The controller does not publish TF.  AMCL remains the sole ``map -> odom``
authority; simulator truth is exposed separately by the Isaac bridge as
``/hybrid_localization/ground_truth/pose``.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import random
import sys
import time
import uuid
from pathlib import Path
from typing import Any

import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import LoadMap
from std_msgs.msg import String
from std_srvs.srv import Empty


COMMAND_TOPIC = "/hybrid_localization/scenario_command"
STATUS_TOPIC = "/hybrid_localization/scenario_status"
ACTIVE_SCENARIO_TOPIC = "/hybrid_localization/active_scenario"


class ScenarioControllerError(RuntimeError):
    pass


def _package_root() -> Path:
    """Return the source package root shared with the Isaac container.

    The host executable is installed under ``install/<pkg>/lib/<pkg>`` while
    Isaac sees the repository source tree through the workspace bind mount.
    Derive that source path from the ament package prefix, with an explicit
    environment override for non-standard layouts.
    """

    configured = os.environ.get("HYBRID_LOCALIZATION_ISAAC_SIM_ROOT")
    if configured:
        candidate = Path(configured).expanduser().resolve()
        if candidate.is_dir():
            return candidate
        raise ScenarioControllerError(
            f"HYBRID_LOCALIZATION_ISAAC_SIM_ROOT does not exist: {candidate}"
        )

    prefix = Path(get_package_prefix("hybrid_localization_isaac_sim")).resolve()
    workspace_root = prefix.parents[1]
    candidate = workspace_root / "src" / "hybrid_localization_isaac_sim"
    if candidate.is_dir():
        return candidate

    raise ScenarioControllerError(
        "Could not locate the hybrid_localization_isaac_sim source tree shared "
        "with Isaac. Set HYBRID_LOCALIZATION_ISAAC_SIM_ROOT explicitly."
    )


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ScenarioControllerError(f"Could not load helper: {path}")

    module = importlib.util.module_from_spec(spec)
    previous = sys.modules.get(name)
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        if previous is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = previous
        raise
    return module


def command_payload(
    scenario_id: str,
    *,
    request_id: str,
    robot_pose_override: tuple[float, float, float] | None,
    action: str = "apply",
) -> str:
    value: dict[str, Any] = {"request_id": request_id, "scenario_id": scenario_id}
    if action != "apply":
        value["action"] = action
    if robot_pose_override is not None:
        value["robot_pose_override"] = list(robot_pose_override)
    return json.dumps(value, sort_keys=True)


def matching_status(payload: str, request_id: str) -> dict[str, Any] | None:
    try:
        value = json.loads(payload)
    except json.JSONDecodeError:
        return None
    if not isinstance(value, dict) or value.get("request_id") != request_id:
        return None
    return value


def map_yaml_for_scenario(root: Path, world_id: str) -> Path:
    return root / "generated" / "scenarios" / world_id / "map.yaml"


class ScenarioController(Node):
    def __init__(self) -> None:
        super().__init__("hybrid_localization_scenario_controller")
        self.command_pub = self.create_publisher(String, COMMAND_TOPIC, 10)
        self.status_sub = self.create_subscription(String, STATUS_TOPIC, self._status_cb, 10)
        self.map_client = self.create_client(LoadMap, "/map_server/load_map")
        self.global_client = self.create_client(Empty, "/reinitialize_global_localization")
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self._statuses: list[str] = []

    def _status_cb(self, msg: String) -> None:
        self._statuses.append(msg.data)

    def _wait_for_service(self, client, name: str, timeout: float) -> None:
        if not client.wait_for_service(timeout_sec=timeout):
            raise ScenarioControllerError(f"Timed out waiting for {name}")

    def _call(self, client, request, name: str, timeout: float):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.exception() is not None:
            raise ScenarioControllerError(f"Service call failed or timed out: {name}")
        return future.result()

    def request_isaac(
        self,
        scenario_id: str,
        robot_pose_override: tuple[float, float, float] | None,
        timeout: float,
    ) -> dict[str, Any]:
        request_id = uuid.uuid4().hex
        deadline = time.monotonic() + timeout

        # Wait briefly for the persistent Isaac bridge subscriber.
        while self.command_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.command_pub.get_subscription_count() == 0:
            raise ScenarioControllerError(
                "Isaac scenario bridge is not connected. Run isaac_scenario_bridge.py "
                "once in Isaac Sim's Script Editor."
            )

        msg = String()
        msg.data = command_payload(
            scenario_id,
            request_id=request_id,
            robot_pose_override=robot_pose_override,
        )
        self.command_pub.publish(msg)

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            for payload in self._statuses:
                status = matching_status(payload, request_id)
                if status is None:
                    continue
                state = status.get("state")
                if state == "ready":
                    status["request_id"] = request_id
                    return status
                if state == "error":
                    raise ScenarioControllerError(
                        f"Isaac scenario application failed: {status.get('detail', 'unknown error')}"
                    )
        raise ScenarioControllerError("Timed out waiting for Isaac scenario READY status")

    def load_map(self, map_yaml: Path, timeout: float) -> None:
        self._wait_for_service(self.map_client, "/map_server/load_map", timeout)
        request = LoadMap.Request()
        request.map_url = str(map_yaml)
        result = self._call(self.map_client, request, "/map_server/load_map", timeout)
        # nav2_msgs/LoadMap result constants: RESULT_SUCCESS == 0.
        if int(result.result) != 0:
            raise ScenarioControllerError(
                f"map_server rejected map {map_yaml}: result={int(result.result)}"
            )

    def _publish_initial_pose(self, pose: dict[str, Any], xy_stddev: float, yaw_stddev: float) -> None:
        deadline = time.monotonic() + 2.0
        while self.initial_pose_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(pose["x"])
        msg.pose.pose.position.y = float(pose["y"])
        yaw = float(pose["yaw"])
        msg.pose.pose.orientation.z = math.sin(0.5 * yaw)
        msg.pose.pose.orientation.w = math.cos(0.5 * yaw)
        msg.pose.covariance[0] = xy_stddev * xy_stddev
        msg.pose.covariance[7] = xy_stddev * xy_stddev
        msg.pose.covariance[35] = yaw_stddev * yaw_stddev
        self.initial_pose_pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.2)

    def reset_localization(self, scenario: dict[str, Any], layout_path: Path, timeout: float) -> None:
        policy = scenario["localization"]
        mode = policy["mode"]
        if mode == "known_pose":
            self._publish_initial_pose(
                policy["initial_pose"],
                float(policy["xy_stddev"]),
                float(policy["yaw_stddev"]),
            )
            return
        if mode == "global":
            self._wait_for_service(self.global_client, "/reinitialize_global_localization", timeout)
            self._call(
                self.global_client,
                Empty.Request(),
                "/reinitialize_global_localization",
                timeout,
            )
            return
        if mode == "random_prior":
            world_layout = _load_module("world_layout", _package_root() / "scripts" / "world_layout.py")
            reset_amcl = _load_module(
                "reset_amcl_localization", _package_root() / "scripts" / "reset_amcl_localization.py"
            )
            layout = world_layout.load_layout(layout_path)
            rng = random.Random(int(policy["seed"]))
            x, y, yaw = reset_amcl._sample_free_pose(layout, rng)
            self._publish_initial_pose(
                {"x": x, "y": y, "yaw": yaw},
                float(policy["xy_stddev"]),
                float(policy["yaw_stddev"]),
            )
            return
        raise ScenarioControllerError(f"Unsupported localization mode: {mode}")

    def activate_scenario(self, scenario: dict[str, Any], isaac_status: dict[str, Any], timeout: float) -> None:
        request_id = str(isaac_status.get("request_id", ""))
        if not request_id:
            raise ScenarioControllerError("Isaac READY status did not contain request_id")

        msg = String()
        msg.data = command_payload(
            scenario["id"],
            request_id=request_id,
            robot_pose_override=None,
            action="activate",
        )
        self.command_pub.publish(msg)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            for payload in self._statuses:
                status = matching_status(payload, request_id)
                if status is None:
                    continue
                state = status.get("state")
                if state == "active":
                    return
                if state == "error":
                    raise ScenarioControllerError(
                        f"Isaac scenario activation failed: {status.get('detail', 'unknown error')}"
                    )
        raise ScenarioControllerError("Timed out waiting for Isaac scenario ACTIVE status")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--robot-pose", nargs=3, type=float, metavar=("X", "Y", "YAW"))
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    root = _package_root()
    runtime = _load_module("scenario_runtime", root / "scripts" / "scenario_runtime.py")
    materializer = _load_module(
        "materialize_world_scenario", root / "scripts" / "materialize_world_scenario.py"
    )

    runtime_catalog = runtime.load_catalog()
    if args.scenario not in runtime_catalog:
        parser.error(f"Unknown runtime scenario {args.scenario!r}")
    scenario = runtime_catalog[args.scenario]
    world_id = scenario["world_scenario"]

    world_catalog = materializer.load_catalog(root / "config" / "world_scenarios.json")
    output_dir = materializer.materialize(world_catalog[world_id], root / "generated" / "scenarios")
    map_yaml = output_dir / "map.yaml"
    layout_path = output_dir / "layout.json"

    override = tuple(args.robot_pose) if args.robot_pose is not None else None

    rclpy.init()
    node = ScenarioController()
    try:
        isaac_status = node.request_isaac(args.scenario, override, args.timeout)
        node.load_map(map_yaml, args.timeout)
        node.reset_localization(scenario, layout_path, args.timeout)
        node.activate_scenario(scenario, isaac_status, args.timeout)
        print(
            f"Scenario {args.scenario} ACTIVE: world={world_id}, "
            f"map={map_yaml}, localization={scenario['localization']['mode']}, "
            f"Isaac pose={isaac_status.get('observed_pose')}"
        )
    except ScenarioControllerError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
