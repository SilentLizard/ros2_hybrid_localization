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
from typing import Any, Mapping


COMMAND_TOPIC = "/hybrid_localization/scenario_command"
STATUS_TOPIC = "/hybrid_localization/scenario_status"
GROUND_TRUTH_TOPIC = "/hybrid_localization/ground_truth/pose"
ACTIVE_SCENARIO_TOPIC = "/hybrid_localization/active_scenario"
FAULT_STATUS_TOPIC = "/hybrid_localization/fault_status"
GROUND_TRUTH_FRAME = "map"
GROUND_TRUTH_RATE_HZ = 20.0
WORLD_SYNC_UPDATES = 2
RESET_STABLE_UPDATES = 3
FAULT_STABLE_UPDATES = 3

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



def _normalize_yaw(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


def kidnapped_target_pose(
    current_pose: tuple[float, float, float],
    parameters: Mapping[str, Any],
) -> tuple[float, float, float]:
    """Apply a kidnapped-robot SE(2) offset in the map/world frame."""

    return (
        current_pose[0] + float(parameters["x_offset_m"]),
        current_pose[1] + float(parameters["y_offset_m"]),
        _normalize_yaw(current_pose[2] + float(parameters["yaw_offset_rad"])),
    )


def fault_status_payload(
    *,
    scenario_id: str,
    scheduled_fault,
    scenario_activation_sim_time_s: float,
    detail: str = "",
) -> str:
    """Serialize one deterministic fault state for observers and #45."""

    specification = scheduled_fault.specification
    value: dict[str, Any] = {
        "scenario_id": scenario_id,
        "fault_id": specification.fault_id,
        "fault_type": specification.fault_type.value,
        "state": scheduled_fault.state.value,
        "scenario_activation_sim_time_s": scenario_activation_sim_time_s,
        "scheduled_elapsed_time_s": specification.start_time_s,
        "scheduled_sim_time_s": scenario_activation_sim_time_s + specification.start_time_s,
        "seed": specification.seed,
        "parameters": dict(specification.parameters),
    }
    if scheduled_fault.activation_sim_time_s is not None:
        value["activation_sim_time_s"] = scheduled_fault.activation_sim_time_s
    if scheduled_fault.completion_sim_time_s is not None:
        value["completion_sim_time_s"] = scheduled_fault.completion_sim_time_s
    if scheduled_fault.error_detail is not None:
        value["error"] = scheduled_fault.error_detail
    if detail:
        value["detail"] = detail
    return json.dumps(value, sort_keys=True)



def active_scenario_payload(pending: Mapping[str, Any], activation_sim_time_s: float) -> str:
    """Serialize the retained active-scenario contract used by ROS-side injectors."""

    if not math.isfinite(activation_sim_time_s) or activation_sim_time_s < 0.0:
        raise IsaacScenarioBridgeError("activation_sim_time_s must be finite and >= 0")
    active = dict(pending)
    active["state"] = "active"
    active["activation_sim_time_s"] = float(activation_sim_time_s)
    return json.dumps(active, sort_keys=True)

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
        self._fault_status_pub = self._node.create_publisher(
            self._String, FAULT_STATUS_TOPIC, active_qos
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
        self._fault_scheduler_module = _load_module(
            "fault_scheduler", root / "scripts" / "fault_scheduler.py"
        )
        self._environment_geometry = _load_module(
            "environment_fault_geometry", root / "scripts" / "environment_fault_geometry.py"
        )
        self._environment_backend_module = _load_module(
            "isaac_environment_faults", root / "scripts" / "isaac_environment_faults.py"
        )
        self._robot_backend = self._reset.IsaacArticulationBackend()
        self._environment_backend = (
            self._environment_backend_module.IsaacEnvironmentFaultController()
        )
        self._fault_scheduler = self._fault_scheduler_module.DeterministicFaultScheduler()
        self._pending: dict[str, Any] | None = None
        self._pending_faults = ()
        self._apply_work: dict[str, Any] | None = None
        self._fault_work: dict[str, Any] | None = None
        self._active_dynamic_faults: dict[str, dict[str, Any]] = {}

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

    def _publish_fault_status(self, scheduled_fault, *, detail: str = "") -> None:
        activation_time = self._fault_scheduler.scenario_activation_sim_time_s
        scenario_id = self._fault_scheduler.scenario_id
        if activation_time is None or scenario_id is None:
            return
        msg = self._String()
        msg.data = fault_status_payload(
            scenario_id=scenario_id,
            scheduled_fault=scheduled_fault,
            scenario_activation_sim_time_s=activation_time,
            detail=detail,
        )
        self._fault_status_pub.publish(msg)

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
                activation_sim_time_s = self._node.get_clock().now().nanoseconds * 1.0e-9
                msg = self._String()
                msg.data = active_scenario_payload(pending, activation_sim_time_s)
                self._active_pub.publish(msg)

                # Isaac owns physical simulator-side faults. ROS-side
                # measurement injectors consume the same retained active-scenario
                # event and schedule odometry/LiDAR channel faults against this
                # exact activation timestamp.
                isaac_faults = tuple(
                    fault
                    for fault in self._pending_faults
                    if fault.fault_type.value
                    in {"kidnapped_robot", "map_mismatch", "dynamic_obstruction"}
                )
                scheduled = self._fault_scheduler.configure(
                    scenario_id=command.scenario_id,
                    faults=isaac_faults,
                    activation_sim_time_s=activation_sim_time_s,
                )
                self._fault_work = None
                for fault in scheduled:
                    self._publish_fault_status(fault, detail="fault scheduled")

                self._publish_status(
                    status_payload(command, state="active", detail="scenario activation committed")
                )
                return

            if self._apply_work is not None:
                raise IsaacScenarioBridgeError("another Isaac scenario apply is still in progress")

            # Applying a new canonical/runtime scenario immediately disables any
            # prior fault schedule so fault state cannot leak across experiments.
            self._fault_scheduler.clear()
            self._fault_work = None
            self._pending_faults = ()
            self._active_dynamic_faults.clear()
            self._environment_backend.clear_all()

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
            faults = self._runtime.scenario_faults(scenario)
            self._pending = {
                "request_id": command.request_id,
                "scenario_id": scenario["id"],
                "world_scenario": world_id,
                "seed": scenario["seed"],
                "localization_mode": scenario["localization"]["mode"],
                "fault_count": len(faults),
                "fault_ids": [fault.fault_id for fault in faults],
                "isaac_observed_pose": {"x": observed[0], "y": observed[1], "yaw": observed[2]},
            }
            self._pending_faults = faults
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

    def _begin_kidnapped_robot_fault(self, scheduled_fault, sim_time_s: float) -> None:
        current = self._robot_backend.read_pose()
        target_values = kidnapped_target_pose(
            (current.x, current.y, current.yaw),
            scheduled_fault.specification.parameters,
        )
        target = self._reset.Pose2d(*target_values)
        self._reset.execute_reset(
            self._robot_backend,
            scenario_id=self._fault_scheduler.scenario_id or "unknown",
            pose=target,
        )
        self._fault_work = {
            "scheduled_fault": scheduled_fault,
            "target": target,
            "updates_remaining": FAULT_STABLE_UPDATES,
        }
        self._publish_fault_status(
            scheduled_fault,
            detail=(
                f"kidnapped robot teleported at sim_time={sim_time_s:.9f}; "
                f"waiting {FAULT_STABLE_UPDATES} updates for stable verification"
            ),
        )


    def _begin_map_mismatch_fault(self, scheduled_fault, sim_time_s: float) -> None:
        current = self._robot_backend.read_pose()
        parameters = scheduled_fault.specification.parameters
        boxes = self._environment_geometry.map_mismatch_geometry(
            str(parameters["severity"]),
            str(parameters["variant"]),
            (current.x, current.y, current.yaw),
        )
        self._environment_backend.apply_boxes(
            scheduled_fault.specification.fault_id, boxes
        )
        self._publish_fault_status(
            scheduled_fault,
            detail=(
                f"persistent map mismatch geometry applied at sim_time={sim_time_s:.9f}; "
                "Nav2 map intentionally unchanged"
            ),
        )

    def _begin_dynamic_obstruction_fault(self, scheduled_fault, sim_time_s: float) -> None:
        current = self._robot_backend.read_pose()
        parameters = scheduled_fault.specification.parameters
        boxes = self._environment_geometry.dynamic_obstruction_geometry(
            str(parameters["profile"]),
            (current.x, current.y, current.yaw),
        )
        fault_id = scheduled_fault.specification.fault_id
        self._environment_backend.apply_boxes(fault_id, boxes)
        end_time_s = scheduled_fault.specification.end_time_s
        if end_time_s is None:
            raise IsaacScenarioBridgeError(
                "dynamic obstruction requires a finite end time"
            )
        activation_anchor = self._fault_scheduler.scenario_activation_sim_time_s
        if activation_anchor is None:
            raise IsaacScenarioBridgeError("fault scheduler has no activation anchor")
        absolute_end_time_s = activation_anchor + float(end_time_s)
        self._active_dynamic_faults[fault_id] = {
            "scheduled_fault": scheduled_fault,
            "end_sim_time_s": absolute_end_time_s,
        }
        self._publish_fault_status(
            scheduled_fault,
            detail=(
                f"dynamic obstruction geometry applied at sim_time={sim_time_s:.9f}; "
                f"scheduled removal at sim_time={absolute_end_time_s:.9f}"
            ),
        )

    def _complete_due_dynamic_faults(self, sim_time_s: float) -> None:
        for fault_id, work in tuple(self._active_dynamic_faults.items()):
            if sim_time_s + 1.0e-12 < work["end_sim_time_s"]:
                continue
            self._environment_backend.remove_fault(fault_id)
            completed = self._fault_scheduler.mark_completed(
                fault_id, sim_time_s=sim_time_s
            )
            del self._active_dynamic_faults[fault_id]
            self._publish_fault_status(
                completed, detail="dynamic obstruction removed after scheduled duration"
            )

    def _advance_faults(self, sim_time_s: float) -> None:
        self._complete_due_dynamic_faults(sim_time_s)
        work = self._fault_work
        if work is not None:
            scheduled_fault = work["scheduled_fault"]
            try:
                if work["updates_remaining"] > 0:
                    work["updates_remaining"] -= 1
                    return
                observed = self._robot_backend.read_pose()
                self._robot_backend.clear_motion()
                self._reset.validate_pose_match(work["target"], observed)
                completed = self._fault_scheduler.mark_completed(
                    scheduled_fault.specification.fault_id,
                    sim_time_s=sim_time_s,
                )
                self._fault_work = None
                self._publish_fault_status(
                    completed, detail="kidnapped robot stable pose verified"
                )
            except Exception as exc:
                self._fault_work = None
                failed = self._fault_scheduler.mark_error(
                    scheduled_fault.specification.fault_id,
                    sim_time_s=sim_time_s,
                    detail=str(exc),
                )
                self._publish_fault_status(failed, detail="fault execution failed")
            return

        try:
            due = self._fault_scheduler.activate_due(sim_time_s)
        except self._fault_scheduler_module.FaultSchedulerError:
            # A backwards/reset simulation clock invalidates relative scheduling.
            # A fresh scenario activation is required to establish a new anchor.
            self._fault_work = None
            self._active_dynamic_faults.clear()
            self._environment_backend.clear_all()
            self._fault_scheduler.clear()
            return

        for scheduled_fault in due:
            fault_type = scheduled_fault.specification.fault_type.value
            if fault_type == "map_mismatch":
                try:
                    self._begin_map_mismatch_fault(scheduled_fault, sim_time_s)
                except Exception as exc:
                    failed = self._fault_scheduler.mark_error(
                        scheduled_fault.specification.fault_id,
                        sim_time_s=sim_time_s,
                        detail=str(exc),
                    )
                    self._publish_fault_status(failed, detail="fault execution failed")
                continue
            if fault_type == "dynamic_obstruction":
                try:
                    self._begin_dynamic_obstruction_fault(scheduled_fault, sim_time_s)
                except Exception as exc:
                    failed = self._fault_scheduler.mark_error(
                        scheduled_fault.specification.fault_id,
                        sim_time_s=sim_time_s,
                        detail=str(exc),
                    )
                    self._publish_fault_status(failed, detail="fault execution failed")
                continue
            if fault_type != "kidnapped_robot":
                failed = self._fault_scheduler.mark_error(
                    scheduled_fault.specification.fault_id,
                    sim_time_s=sim_time_s,
                    detail="fault type is owned by a ROS-side measurement injector",
                )
                self._publish_fault_status(failed, detail="unsupported Isaac-side fault type")
                continue
            if self._fault_work is not None:
                failed = self._fault_scheduler.mark_error(
                    scheduled_fault.specification.fault_id,
                    sim_time_s=sim_time_s,
                    detail="another physical fault is already being stabilized",
                )
                self._publish_fault_status(failed, detail="fault execution conflict")
                continue
            try:
                self._begin_kidnapped_robot_fault(scheduled_fault, sim_time_s)
            except Exception as exc:
                failed = self._fault_scheduler.mark_error(
                    scheduled_fault.specification.fault_id,
                    sim_time_s=sim_time_s,
                    detail=str(exc),
                )
                self._publish_fault_status(failed, detail="fault execution failed")

    def _on_update(self, _event) -> None:
        self._rclpy.spin_once(self._node, timeout_sec=0.0)
        self._advance_apply()
        now = self._node.get_clock().now()
        time_seconds = now.nanoseconds * 1.0e-9
        if self._fault_scheduler.scenario_id is not None:
            self._advance_faults(time_seconds)
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
