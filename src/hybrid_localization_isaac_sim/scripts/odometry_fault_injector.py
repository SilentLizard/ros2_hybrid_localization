#!/usr/bin/env python3
"""ROS-side deterministic odometry gateway/fault injector for #44.

Isaac publishes measurement-only odometry on
``/hybrid_localization/raw_odom`` and no dynamic odometry TF.  This node is the
single source of the localization-facing ``/odom`` message and
``odom -> base_link`` TF.

Two transformations are applied, in this order:

1. Scheduled kidnapped-robot teleports are absorbed by an odometry continuity
   compensator.  Physical ground truth still jumps, but localization-facing
   odometry remains continuous and continues integrating later motion from its
   pre-kidnap belief.
2. If the active runtime scenario schedules ``odometry_degradation``, the
   deterministic Stage-3 fault model is applied to that continuous stream.
"""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


RAW_ODOM_TOPIC = "/hybrid_localization/raw_odom"
OUTPUT_ODOM_TOPIC = "/odom"
ACTIVE_SCENARIO_TOPIC = "/hybrid_localization/active_scenario"
FAULT_STATUS_TOPIC = "/hybrid_localization/fault_status"
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"


def _load_sibling(name: str, filename: str):
    source_path = Path(__file__).resolve().parent / filename
    path = source_path
    if not path.is_file():
        try:
            from ament_index_python.packages import get_package_share_directory
        except ImportError as exc:
            raise RuntimeError(
                f"Could not locate helper module {filename!r} beside {__file__} and "
                "ament_index_python is unavailable"
            ) from exc
        path = (
            Path(get_package_share_directory("hybrid_localization_isaac_sim"))
            / "scripts"
            / filename
        )
    if not path.is_file():
        raise RuntimeError(f"Could not locate helper module: {path}")

    existing = sys.modules.get(name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load helper module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(name, None)
        raise
    return module


_MODEL = _load_sibling("hybrid_localization_odometry_fault_model", "odometry_fault_model.py")
_RUNTIME = _load_sibling("hybrid_localization_scenario_runtime_for_odom", "scenario_runtime.py")
_SCHEDULER = _load_sibling("hybrid_localization_fault_scheduler_for_odom", "fault_scheduler.py")


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _sample_from_message(msg: Odometry):
    stamp = msg.header.stamp
    sim_time_s = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    return _MODEL.OdometrySample(
        sim_time_s=sim_time_s,
        x=float(msg.pose.pose.position.x),
        y=float(msg.pose.pose.position.y),
        yaw=_yaw_from_quaternion(msg.pose.pose.orientation),
        linear_velocity_x=float(msg.twist.twist.linear.x),
        linear_velocity_y=float(msg.twist.twist.linear.y),
        angular_velocity_z=float(msg.twist.twist.angular.z),
    )


def _active_scenario(value: str) -> tuple[str, float]:
    payload = json.loads(value)
    if not isinstance(payload, dict) or payload.get("state") != "active":
        raise ValueError("active scenario payload must contain state=active")
    scenario_id = payload.get("scenario_id")
    activation = payload.get("activation_sim_time_s")
    if not isinstance(scenario_id, str) or not scenario_id:
        raise ValueError("active scenario payload requires scenario_id")
    if isinstance(activation, bool) or not isinstance(activation, (int, float)):
        raise ValueError("active scenario payload requires activation_sim_time_s")
    activation = float(activation)
    if not math.isfinite(activation) or activation < 0.0:
        raise ValueError("activation_sim_time_s must be finite and >= 0")
    return scenario_id, activation


class OdometryFaultInjector(Node):
    def __init__(self) -> None:
        super().__init__("hybrid_localization_odometry_fault_injector")

        retained = QoSProfile(depth=1)
        retained.reliability = ReliabilityPolicy.RELIABLE
        retained.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._odom_pub = self.create_publisher(Odometry, OUTPUT_ODOM_TOPIC, 20)
        self._status_pub = self.create_publisher(String, FAULT_STATUS_TOPIC, retained)
        self._tf = TransformBroadcaster(self)
        self._raw_sub = self.create_subscription(Odometry, RAW_ODOM_TOPIC, self._on_raw_odom, 20)
        self._active_sub = self.create_subscription(
            String, ACTIVE_SCENARIO_TOPIC, self._on_active_scenario, retained
        )

        self._catalog = _RUNTIME.load_catalog()
        self._scheduler = _SCHEDULER.DeterministicFaultScheduler()
        self._continuity = _MODEL.OdometryContinuityCompensator()
        self._model = _MODEL.DeterministicOdometryFaultModel()
        self._last_raw = None
        self._active_fault = None
        self._reported_absorbed_fault_id: str | None = None

        self.get_logger().info(
            f"Odometry gateway active: {RAW_ODOM_TOPIC} -> {OUTPUT_ODOM_TOPIC} + "
            f"{ODOM_FRAME}->{BASE_FRAME}"
        )

    def _publish_status(self, scheduled_fault, *, detail: str) -> None:
        activation = self._scheduler.scenario_activation_sim_time_s
        scenario_id = self._scheduler.scenario_id
        if activation is None or scenario_id is None:
            return
        spec = scheduled_fault.specification
        value = {
            "scenario_id": scenario_id,
            "fault_id": spec.fault_id,
            "fault_type": spec.fault_type.value,
            "state": scheduled_fault.state.value,
            "scenario_activation_sim_time_s": activation,
            "scheduled_elapsed_time_s": spec.start_time_s,
            "scheduled_sim_time_s": activation + spec.start_time_s,
            "seed": spec.seed,
            "parameters": dict(spec.parameters),
            "detail": detail,
        }
        if scheduled_fault.activation_sim_time_s is not None:
            value["activation_sim_time_s"] = scheduled_fault.activation_sim_time_s
        if scheduled_fault.completion_sim_time_s is not None:
            value["completion_sim_time_s"] = scheduled_fault.completion_sim_time_s
        if scheduled_fault.error_detail is not None:
            value["error"] = scheduled_fault.error_detail
        msg = String()
        msg.data = json.dumps(value, sort_keys=True)
        self._status_pub.publish(msg)

    def _on_active_scenario(self, msg: String) -> None:
        try:
            scenario_id, activation_time = _active_scenario(msg.data)
            scenario = self._catalog[scenario_id]
            scenario_faults = _RUNTIME.scenario_faults(scenario)

            odometry_faults = tuple(
                fault
                for fault in scenario_faults
                if fault.fault_type.value == "odometry_degradation"
            )
            scheduled = self._scheduler.configure(
                scenario_id=scenario_id,
                faults=odometry_faults,
                activation_sim_time_s=activation_time,
            )

            self._continuity.reset()
            self._reported_absorbed_fault_id = None
            for fault in scenario_faults:
                if fault.fault_type.value != "kidnapped_robot":
                    continue
                parameters = fault.parameters
                self._continuity.arm(
                    fault_id=fault.fault_id,
                    scheduled_sim_time_s=activation_time + fault.start_time_s,
                    expected_dx_m=float(parameters["x_offset_m"]),
                    expected_dy_m=float(parameters["y_offset_m"]),
                    expected_dyaw_rad=float(parameters["yaw_offset_rad"]),
                )
                self.get_logger().info(
                    "Armed odometry continuity compensation for "
                    f"{fault.fault_id} at sim_time="
                    f"{activation_time + fault.start_time_s:.9f}"
                )

            self._model.deactivate()
            self._active_fault = None
            for fault in scheduled:
                self._publish_status(fault, detail="odometry fault scheduled")
        except Exception as exc:
            self.get_logger().error(f"Could not configure odometry scenario: {exc}")

    def _finish_if_due(self, sim_time_s: float) -> None:
        fault = self._active_fault
        if fault is None:
            return
        spec = fault.specification
        if spec.end_time_s is None:
            return
        activation = self._scheduler.scenario_activation_sim_time_s
        if activation is None:
            return
        if sim_time_s + 1.0e-12 < activation + spec.end_time_s:
            return
        completed = self._scheduler.mark_completed(spec.fault_id, sim_time_s=sim_time_s)
        self._model.deactivate()
        self._active_fault = None
        self._publish_status(completed, detail="odometry fault interval completed")

    def _activate_due(self, continuous_sample) -> None:
        if self._scheduler.scenario_id is None:
            return
        due = self._scheduler.activate_due(continuous_sample.sim_time_s)
        for fault in due:
            if self._active_fault is not None:
                failed = self._scheduler.mark_error(
                    fault.specification.fault_id,
                    sim_time_s=continuous_sample.sim_time_s,
                    detail="another odometry fault is already active",
                )
                self._publish_status(failed, detail="odometry fault conflict")
                continue
            self._model.activate(
                seed=fault.specification.seed,
                parameters=fault.specification.parameters,
                anchor=continuous_sample,
            )
            self._active_fault = fault
            self._publish_status(fault, detail="odometry degradation active")

    def _on_raw_odom(self, msg: Odometry) -> None:
        try:
            raw = _sample_from_message(msg)
            self._last_raw = raw

            continuous = self._continuity.process(raw)
            absorbed = self._continuity.last_absorbed
            if (
                absorbed is not None
                and absorbed.fault_id != self._reported_absorbed_fault_id
            ):
                self._reported_absorbed_fault_id = absorbed.fault_id
                self.get_logger().info(
                    "Absorbed kidnapped-robot raw odometry discontinuity "
                    f"{absorbed.fault_id} at sim_time={absorbed.sim_time_s:.9f}: "
                    f"dx={absorbed.raw_dx_m:.6f} m, "
                    f"dy={absorbed.raw_dy_m:.6f} m, "
                    f"dyaw={absorbed.raw_dyaw_rad:.6f} rad"
                )

            self._finish_if_due(continuous.sim_time_s)
            self._activate_due(continuous)
            output = self._model.process(continuous)
            self._publish_odometry(msg, output)
        except Exception as exc:
            self.get_logger().error(f"Odometry gateway rejected sample: {exc}")

    def _publish_odometry(self, source: Odometry, sample) -> None:
        out = Odometry()
        out.header = source.header
        out.header.frame_id = ODOM_FRAME
        out.child_frame_id = BASE_FRAME
        out.pose = source.pose
        out.twist = source.twist
        out.pose.pose.position.x = sample.x
        out.pose.pose.position.y = sample.y
        out.pose.pose.orientation.x = 0.0
        out.pose.pose.orientation.y = 0.0
        out.pose.pose.orientation.z = math.sin(0.5 * sample.yaw)
        out.pose.pose.orientation.w = math.cos(0.5 * sample.yaw)
        out.twist.twist.linear.x = sample.linear_velocity_x
        out.twist.twist.linear.y = sample.linear_velocity_y
        out.twist.twist.angular.z = sample.angular_velocity_z
        self._odom_pub.publish(out)

        transform = TransformStamped()
        transform.header = out.header
        transform.child_frame_id = BASE_FRAME
        transform.transform.translation.x = sample.x
        transform.transform.translation.y = sample.y
        transform.transform.translation.z = float(source.pose.pose.position.z)
        transform.transform.rotation = out.pose.pose.orientation
        self._tf.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdometryFaultInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
