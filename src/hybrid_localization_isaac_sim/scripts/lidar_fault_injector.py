#!/usr/bin/env python3
"""ROS-side deterministic LaserScan gateway/fault injector for #44 Stage 4.

Isaac publishes nominal scans on ``/hybrid_localization/raw_scan``. This node
is the single localization-facing publisher of ``/scan``. Canonical scenarios
pass scans through unchanged; runtime scenarios containing ``lidar_degradation``
activate the ROS-independent deterministic model using the same scenario
activation simulation timestamp and retained fault-status contract as Stages
2/3.
"""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


RAW_SCAN_TOPIC = "/hybrid_localization/raw_scan"
OUTPUT_SCAN_TOPIC = "/scan"
ACTIVE_SCENARIO_TOPIC = "/hybrid_localization/active_scenario"
FAULT_STATUS_TOPIC = "/hybrid_localization/fault_status"


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


_MODEL = _load_sibling("hybrid_localization_lidar_fault_model", "lidar_fault_model.py")
_RUNTIME = _load_sibling("hybrid_localization_scenario_runtime_for_lidar", "scenario_runtime.py")
_SCHEDULER = _load_sibling("hybrid_localization_fault_scheduler_for_lidar", "fault_scheduler.py")


def _sample_from_message(msg: LaserScan):
    stamp = msg.header.stamp
    sim_time_s = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    return _MODEL.LaserScanSample(
        sim_time_s=sim_time_s,
        angle_min=float(msg.angle_min),
        angle_increment=float(msg.angle_increment),
        range_min=float(msg.range_min),
        range_max=float(msg.range_max),
        ranges=tuple(float(value) for value in msg.ranges),
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


class LidarFaultInjector(Node):
    def __init__(self) -> None:
        super().__init__("hybrid_localization_lidar_fault_injector")

        retained = QoSProfile(depth=1)
        retained.reliability = ReliabilityPolicy.RELIABLE
        retained.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Sensor-data-style depth without forcing RELIABLE against the Isaac
        # publisher. AMCL already consumes LaserScan using sensor-data QoS.
        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self._scan_pub = self.create_publisher(LaserScan, OUTPUT_SCAN_TOPIC, scan_qos)
        self._status_pub = self.create_publisher(String, FAULT_STATUS_TOPIC, retained)
        self._raw_sub = self.create_subscription(
            LaserScan, RAW_SCAN_TOPIC, self._on_raw_scan, scan_qos
        )
        self._active_sub = self.create_subscription(
            String, ACTIVE_SCENARIO_TOPIC, self._on_active_scenario, retained
        )

        self._catalog = _RUNTIME.load_catalog()
        self._scheduler = _SCHEDULER.DeterministicFaultScheduler()
        self._model = _MODEL.DeterministicLidarFaultModel()
        self._active_fault = None

        self.get_logger().info(
            f"LiDAR gateway active: {RAW_SCAN_TOPIC} -> {OUTPUT_SCAN_TOPIC}"
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
            faults = tuple(
                fault
                for fault in _RUNTIME.scenario_faults(scenario)
                if fault.fault_type.value == "lidar_degradation"
            )
            scheduled = self._scheduler.configure(
                scenario_id=scenario_id,
                faults=faults,
                activation_sim_time_s=activation_time,
            )
            self._model.deactivate()
            self._active_fault = None
            for fault in scheduled:
                self._publish_status(fault, detail="LiDAR fault scheduled")
        except Exception as exc:
            self.get_logger().error(f"Could not configure LiDAR scenario: {exc}")

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
        self._publish_status(completed, detail="LiDAR fault interval completed")

    def _activate_due(self, sim_time_s: float) -> None:
        if self._scheduler.scenario_id is None:
            return
        due = self._scheduler.activate_due(sim_time_s)
        for fault in due:
            if self._active_fault is not None:
                failed = self._scheduler.mark_error(
                    fault.specification.fault_id,
                    sim_time_s=sim_time_s,
                    detail="another LiDAR fault is already active",
                )
                self._publish_status(failed, detail="LiDAR fault conflict")
                continue
            self._model.activate(
                seed=fault.specification.seed,
                parameters=fault.specification.parameters,
            )
            self._active_fault = fault
            self._publish_status(fault, detail="LiDAR degradation active")

    def _on_raw_scan(self, msg: LaserScan) -> None:
        try:
            raw = _sample_from_message(msg)
            self._finish_if_due(raw.sim_time_s)
            self._activate_due(raw.sim_time_s)
            output = self._model.process(raw)
            self._publish_scan(msg, output)
        except Exception as exc:
            self.get_logger().error(f"LiDAR gateway rejected scan: {exc}")

    def _publish_scan(self, source: LaserScan, sample) -> None:
        out = LaserScan()
        out.header = source.header
        out.angle_min = source.angle_min
        out.angle_max = source.angle_max
        out.angle_increment = source.angle_increment
        out.time_increment = source.time_increment
        out.scan_time = source.scan_time
        out.range_min = source.range_min
        out.range_max = float(sample.range_max)
        out.ranges = list(sample.ranges)
        out.intensities = list(source.intensities)
        self._scan_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarFaultInjector()
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
