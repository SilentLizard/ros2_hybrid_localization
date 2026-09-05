#!/usr/bin/env python3
"""Persistent Isaac-side scenario bridge and ground-truth publisher for #43.

Run this file once from Isaac Sim's Script Editor after the HEROS stage and ROS
bridge graphs exist.  It deliberately publishes simulator truth as a pose topic,
not TF, so AMCL remains the only owner of ``map -> odom``.

ROS interfaces:

* subscribes ``/hybrid_localization/scenario_command`` (std_msgs/String JSON);
* publishes ``/hybrid_localization/scenario_status`` (std_msgs/String JSON);
* publishes ``/hybrid_localization/ground_truth/pose``
  (geometry_msgs/PoseStamped, frame ``map``).

The command payload is produced by ``scenario_controller.py`` and contains a
validated runtime scenario ID plus an optional physical robot-pose override.
World switching and HEROS reset happen in Isaac from the same command.
"""

from __future__ import annotations

import importlib
import importlib.util
import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


COMMAND_TOPIC = "/hybrid_localization/scenario_command"
STATUS_TOPIC = "/hybrid_localization/scenario_status"
GROUND_TRUTH_TOPIC = "/hybrid_localization/ground_truth/pose"
ACTIVE_SCENARIO_TOPIC = "/hybrid_localization/active_scenario"
GROUND_TRUTH_FRAME = "map"
GROUND_TRUTH_RATE_HZ = 20.0
WORLD_SYNC_UPDATES = 2
RESET_STABLE_UPDATES = 3

_BRIDGE_INSTANCE = None


class IsaacScenarioBridgeError(RuntimeError):
    """Raised when Isaac-side scenario application cannot be completed."""


@dataclass(frozen=True)
class ScenarioCommand:
    request_id: str
    scenario_id: str
    robot_pose_override: tuple[float, float, float] | None = None
    action: str = "apply"


def _package_root() -> Path:
    configured = os.environ.get("HYBRID_LOCALIZATION_ISAAC_SIM_ROOT")
    if configured:
        return Path(configured).expanduser()
    return Path("/workspace/ros2_hybrid_localization/src/hybrid_localization_isaac_sim")


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise IsaacScenarioBridgeError(f"Could not load helper module: {path}")

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


def parse_command(payload: str) -> ScenarioCommand:
    try:
        value = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise IsaacScenarioBridgeError("Scenario command must be valid JSON") from exc
    if not isinstance(value, dict):
        raise IsaacScenarioBridgeError("Scenario command must be a JSON object")

    request_id = value.get("request_id")
    scenario_id = value.get("scenario_id")
    if not isinstance(request_id, str) or not request_id:
        raise IsaacScenarioBridgeError("request_id must be a non-empty string")
    if not isinstance(scenario_id, str) or not scenario_id:
        raise IsaacScenarioBridgeError("scenario_id must be a non-empty string")

    action = value.get("action", "apply")
    if action not in {"apply", "activate"}:
        raise IsaacScenarioBridgeError("action must be apply or activate")

    override = value.get("robot_pose_override")
    if override is None:
        parsed_override = None
    else:
        if not isinstance(override, list) or len(override) != 3:
            raise IsaacScenarioBridgeError("robot_pose_override must be [x, y, yaw]")
        parsed = tuple(float(v) for v in override)
        if not all(math.isfinite(v) for v in parsed):
            raise IsaacScenarioBridgeError("robot_pose_override values must be finite")
        if not -math.pi <= parsed[2] < math.pi:
            raise IsaacScenarioBridgeError("robot_pose_override yaw must be in [-pi, pi)")
        parsed_override = parsed

    return ScenarioCommand(request_id, scenario_id, parsed_override, action)


def status_payload(
    command: ScenarioCommand,
    *,
    state: str,
    detail: str,
    observed_pose: tuple[float, float, float] | None = None,
) -> str:
    value: dict[str, Any] = {
        "request_id": command.request_id,
        "scenario_id": command.scenario_id,
        "state": state,
        "action": command.action,
        "detail": detail,
    }
    if observed_pose is not None:
        value["observed_pose"] = {
            "x": observed_pose[0],
            "y": observed_pose[1],
            "yaw": observed_pose[2],
        }
    return json.dumps(value, sort_keys=True)


class IsaacScenarioBridge:
    def __init__(self) -> None:
        # Dynamic imports keep ordinary ROS package tests independent of Isaac's
        # Python environment and keep this file classified as an Isaac helper.
        self._rclpy = importlib.import_module("rclpy")
        geometry_msgs = importlib.import_module("geometry_msgs.msg")
        std_msgs = importlib.import_module("std_msgs.msg")
        carb = importlib.import_module("carb")
        omni_kit_app = importlib.import_module("omni.kit.app")

        if not self._rclpy.ok():
            self._rclpy.init()
            self._owns_rclpy = True
        else:
            self._owns_rclpy = False

        self._node = self._rclpy.create_node("hybrid_localization_isaac_scenario_bridge")
        self._node.set_parameters(
            [self._rclpy.parameter.Parameter("use_sim_time", value=True)]
        )
        self._PoseStamped = geometry_msgs.PoseStamped
        self._String = std_msgs.String

        self._status_pub = self._node.create_publisher(self._String, STATUS_TOPIC, 10)

        qos = importlib.import_module("rclpy.qos")
        active_qos = qos.QoSProfile(depth=1)
        active_qos.reliability = qos.ReliabilityPolicy.RELIABLE
        active_qos.durability = qos.DurabilityPolicy.TRANSIENT_LOCAL
        self._active_pub = self._node.create_publisher(
            self._String, ACTIVE_SCENARIO_TOPIC, active_qos
        )
        self._ground_truth_pub = self._node.create_publisher(
            self._PoseStamped, GROUND_TRUTH_TOPIC, 10
        )
        self._command_sub = self._node.create_subscription(
            self._String, COMMAND_TOPIC, self._on_command, 10
        )

        root = _package_root()
        self._reset = _load_module("reset_heros_scenario", root / "scripts" / "reset_heros_scenario.py")
        self._world = _load_module("build_localization_world", root / "scripts" / "build_localization_world.py")
        self._runtime = _load_module("scenario_runtime", root / "scripts" / "scenario_runtime.py")
        self._robot_backend = self._reset.IsaacArticulationBackend()
        self._pending: dict[str, Any] | None = None
        self._apply_work: dict[str, Any] | None = None

        self._last_ground_truth_time = -1.0
        self._period = 1.0 / GROUND_TRUTH_RATE_HZ
        self._update_subscription = (
            omni_kit_app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(self._on_update, name="hybrid_localization_scenario_bridge")
        )
        carb.log_info(
            f"Hybrid localization Isaac scenario bridge active; command={COMMAND_TOPIC}, "
            f"ground_truth={GROUND_TRUTH_TOPIC}"
        )

    def _publish_status(self, payload: str) -> None:
        msg = self._String()
        msg.data = payload
        self._status_pub.publish(msg)

    def _on_command(self, msg) -> None:
        try:
            command = parse_command(msg.data)
        except Exception as exc:
            # No request ID is trustworthy for malformed commands.
            dummy = ScenarioCommand("invalid", "invalid")
            self._publish_status(status_payload(dummy, state="error", detail=str(exc)))
            return

        try:
            if command.action == "activate":
                pending = self._pending
                if pending is None or pending["request_id"] != command.request_id or pending["scenario_id"] != command.scenario_id:
                    raise IsaacScenarioBridgeError(
                        "activate command does not match the pending Isaac READY scenario"
                    )
                active = dict(pending)
                active["state"] = "active"
                msg = self._String()
                msg.data = json.dumps(active, sort_keys=True)
                self._active_pub.publish(msg)
                self._publish_status(
                    status_payload(command, state="active", detail="scenario activation committed")
                )
                return

            if self._apply_work is not None:
                raise IsaacScenarioBridgeError("another Isaac scenario apply is still in progress")

            self._publish_status(status_payload(command, state="applying", detail="Isaac apply started"))
            scenario = self._runtime.load_catalog()[command.scenario_id]
            world_id = scenario["world_scenario"]
            layout = _package_root() / "generated" / "scenarios" / world_id / "layout.json"
            if not layout.is_file():
                raise IsaacScenarioBridgeError(
                    f"World scenario {world_id!r} is not materialized in Isaac mount: {layout}"
                )

            # Stage edits are synchronized into PhysX at update boundaries.  Do
            # not teleport the articulation in the same callback as the world
            # edit: an immediate read-back can look correct and then be replaced
            # by the pre-edit physics state on the next simulation update.
            self._world.apply_layout(str(layout))
            reset_scenario = self._reset.scenario_for_reset(
                command.scenario_id,
                robot_pose_override=command.robot_pose_override,
            )
            requested = self._reset.pose2d_from_scenario(reset_scenario)
            self._pending = None
            self._apply_work = {
                "command": command,
                "scenario": scenario,
                "world_id": world_id,
                "requested": requested,
                "phase": "world_sync",
                "updates_remaining": WORLD_SYNC_UPDATES,
            }
        except Exception as exc:
            self._publish_status(status_payload(command, state="error", detail=str(exc)))

    def _fail_apply(self, command: ScenarioCommand, exc: Exception) -> None:
        self._apply_work = None
        self._pending = None
        self._publish_status(status_payload(command, state="error", detail=str(exc)))

    def _advance_apply(self) -> None:
        work = self._apply_work
        if work is None:
            return

        command = work["command"]
        try:
            if work["updates_remaining"] > 0:
                work["updates_remaining"] -= 1
                return

            if work["phase"] == "world_sync":
                # Re-bind the articulation after the stage edit has reached
                # PhysX, then perform the physical reset.
                self._robot_backend.refresh()
                result = self._reset.execute_reset(
                    self._robot_backend,
                    scenario_id=command.scenario_id,
                    pose=work["requested"],
                )
                work["initial_result"] = result
                work["phase"] = "reset_stable"
                work["updates_remaining"] = RESET_STABLE_UPDATES
                return

            if work["phase"] != "reset_stable":
                raise IsaacScenarioBridgeError(f"unknown apply phase {work['phase']!r}")

            # This is the critical live check: validate the articulation again
            # only after several simulation updates, not just immediately after
            # set_world_pose().
            observed_pose = self._robot_backend.read_pose()
            self._robot_backend.clear_motion()
            self._reset.validate_pose_match(work["requested"], observed_pose)
            observed = (observed_pose.x, observed_pose.y, observed_pose.yaw)
            scenario = work["scenario"]
            world_id = work["world_id"]
            self._pending = {
                "request_id": command.request_id,
                "scenario_id": scenario["id"],
                "world_scenario": world_id,
                "seed": scenario["seed"],
                "localization_mode": scenario["localization"]["mode"],
                "isaac_observed_pose": {"x": observed[0], "y": observed[1], "yaw": observed[2]},
            }
            self._apply_work = None
            self._publish_status(
                status_payload(
                    command,
                    state="ready",
                    detail=(
                        f"world={world_id}; robot reset stable across "
                        f"{RESET_STABLE_UPDATES} Isaac updates; waiting for host activation"
                    ),
                    observed_pose=observed,
                )
            )
        except Exception as exc:
            self._fail_apply(command, exc)

    def _on_update(self, _event) -> None:
        self._rclpy.spin_once(self._node, timeout_sec=0.0)
        self._advance_apply()
        now = self._node.get_clock().now()
        time_seconds = now.nanoseconds * 1.0e-9
        if self._last_ground_truth_time >= 0.0 and time_seconds - self._last_ground_truth_time < self._period:
            return
        self._last_ground_truth_time = time_seconds

        try:
            pose = self._robot_backend.read_pose()
        except Exception:
            return

        msg = self._PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = GROUND_TRUTH_FRAME
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        msg.pose.position.z = pose.z
        msg.pose.orientation.z = math.sin(0.5 * pose.yaw)
        msg.pose.orientation.w = math.cos(0.5 * pose.yaw)
        self._ground_truth_pub.publish(msg)

    def shutdown(self) -> None:
        self._update_subscription = None
        self._node.destroy_node()
        if self._owns_rclpy and self._rclpy.ok():
            self._rclpy.shutdown()


def start_bridge() -> IsaacScenarioBridge:
    global _BRIDGE_INSTANCE
    if _BRIDGE_INSTANCE is not None:
        _BRIDGE_INSTANCE.shutdown()
    _BRIDGE_INSTANCE = IsaacScenarioBridge()
    return _BRIDGE_INSTANCE


def stop_bridge() -> None:
    global _BRIDGE_INSTANCE
    if _BRIDGE_INSTANCE is not None:
        _BRIDGE_INSTANCE.shutdown()
        _BRIDGE_INSTANCE = None


if __name__ == "__main__":
    start_bridge()
    print("Isaac scenario bridge started. Keep this Script Editor module loaded while Isaac runs.")
