#!/usr/bin/env python3
"""Live one-scenario acceptance helper for #44 Stage 6.

This intentionally validates one already-activated scenario at a time. It is
not the unattended matrix runner planned for #45.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


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


_RUNTIME = _load_sibling(
    "hybrid_localization_stage6_runtime",
    "scenario_runtime.py",
)
_MATRIX = _load_sibling(
    "hybrid_localization_stage6_matrix",
    "fault_validation_matrix.py",
)

ACTIVE_TOPIC = "/hybrid_localization/active_scenario"
FAULT_STATUS_TOPIC = "/hybrid_localization/fault_status"
GROUND_TRUTH_TOPIC = "/hybrid_localization/ground_truth/pose"

REQUIRED_TOPICS = (
    "/clock",
    "/hybrid_localization/raw_odom",
    "/odom",
    "/hybrid_localization/raw_scan",
    "/scan",
    "/tf",
    "/tf_static",
    "/particle_cloud",
    "/hybrid_localization/particle_analysis",
    ACTIVE_TOPIC,
    FAULT_STATUS_TOPIC,
    GROUND_TRUTH_TOPIC,
)


class FaultScenarioAcceptance(Node):
    def __init__(self, scenario_id: str) -> None:
        super().__init__("hybrid_localization_fault_scenario_acceptance")

        self.scenario_id = scenario_id
        self.active_payload: dict | None = None
        self.observed_statuses: dict[str, dict] = {}
        self.ground_truth_stamps: list[int] = []

        retained = QoSProfile(depth=20)
        retained.reliability = ReliabilityPolicy.RELIABLE
        retained.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            String,
            ACTIVE_TOPIC,
            self._active_cb,
            retained,
        )

        self.create_subscription(
            String,
            FAULT_STATUS_TOPIC,
            self._status_cb,
            retained,
        )

        self.create_subscription(
            PoseStamped,
            GROUND_TRUTH_TOPIC,
            self._gt_cb,
            20,
        )

    def _active_cb(self, msg: String) -> None:
        try:
            value = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if (
            isinstance(value, dict)
            and value.get("scenario_id") == self.scenario_id
        ):
            self.active_payload = value

    def _status_cb(self, msg: String) -> None:
        try:
            _MATRIX.update_observed_statuses(
                self.observed_statuses,
                msg.data,
                self.scenario_id,
            )
        except _MATRIX.FaultValidationError:
            # Multiple TRANSIENT_LOCAL fault-status publishers can replay
            # retained messages from previously active scenarios during DDS
            # discovery. Only current-scenario payloads are relevant.
            return

    def _gt_cb(self, msg: PoseStamped) -> None:
        stamp = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )

        if (
            not self.ground_truth_stamps
            or stamp > self.ground_truth_stamps[-1]
        ):
            self.ground_truth_stamps.append(stamp)

    def publisher_names(self, topic: str) -> list[str]:
        return [
            info.node_name
            for info in self.get_publishers_info_by_topic(topic)
        ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate one active #44 Stage-6 scenario."
    )

    parser.add_argument(
        "--scenario",
        required=True,
    )

    parser.add_argument(
        "--timeout-s",
        type=float,
        help=(
            "Maximum wall-clock time to wait for the expected scenario/fault "
            "acceptance state. Defaults to the matrix benchmark duration + 5 s."
        ),
    )

    parser.add_argument(
        "--discovery-timeout-s",
        type=float,
        default=10.0,
        help=(
            "Maximum wall-clock time to wait for required ROS topics and "
            "gateway publishers to become visible through DDS discovery "
            "after the scenario acceptance state has been reached."
        ),
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    catalog = _RUNTIME.load_catalog()
    _MATRIX.validate_matrix(
        catalog,
        _RUNTIME.scenario_faults,
    )

    case = _MATRIX.case_for_scenario(args.scenario)

    timeout_s = (
        args.timeout_s
        if args.timeout_s is not None
        else case.benchmark_duration_s + 5.0
    )

    if timeout_s <= 0.0:
        raise SystemExit("--timeout-s must be > 0")

    if args.discovery_timeout_s <= 0.0:
        raise SystemExit("--discovery-timeout-s must be > 0")

    rclpy.init()
    node = FaultScenarioAcceptance(args.scenario)

    try:
        #
        # Phase 1:
        #
        # Wait for:
        #   - the requested scenario to be ACTIVE,
        #   - the expected fault state(s),
        #   - multiple strictly increasing ground-truth samples.
        #
        acceptance_deadline = time.monotonic() + timeout_s

        while time.monotonic() < acceptance_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

            if (
                node.active_payload is None
                or node.active_payload.get("state") != "active"
            ):
                continue

            try:
                _MATRIX.validate_observed_statuses(
                    case,
                    node.observed_statuses,
                )

                _MATRIX.validate_ground_truth_stamps(
                    node.ground_truth_stamps
                )

            except _MATRIX.FaultValidationError:
                continue

            break

        else:
            observed_text = ", ".join(
                f"{fault_id}:{value.get('state')}"
                for fault_id, value
                in sorted(node.observed_statuses.items())
            )

            raise _MATRIX.FaultValidationError(
                f"timed out waiting for {args.scenario} "
                "Stage-6 acceptance state; "
                f"observed={{{observed_text}}}"
            )

        #
        # Phase 2:
        #
        # ROS graph discovery is asynchronous. A freshly started validator
        # can receive retained scenario/fault messages before all existing
        # topic endpoints are visible through get_topic_names_and_types().
        #
        # Do not fail based on a single graph snapshot. Give DDS discovery a
        # separate bounded grace period.
        #
        discovery_deadline = (
            time.monotonic() + args.discovery_timeout_s
        )

        missing = list(REQUIRED_TOPICS)
        gateway_error: _MATRIX.FaultValidationError | None = None

        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

            available = {
                name
                for name, _types
                in node.get_topic_names_and_types()
            }

            missing = [
                topic
                for topic in REQUIRED_TOPICS
                if topic not in available
            ]

            if missing:
                continue

            try:
                _MATRIX.validate_gateway_publishers(
                    {
                        "/odom": node.publisher_names("/odom"),
                        "/scan": node.publisher_names("/scan"),
                    }
                )

            except _MATRIX.FaultValidationError as exc:
                # Topic discovery and endpoint-info discovery can converge at
                # slightly different times. Keep waiting within the same
                # bounded discovery window.
                gateway_error = exc
                continue

            gateway_error = None
            break

        else:
            if missing:
                raise _MATRIX.FaultValidationError(
                    "timed out waiting for required topic discovery: "
                    + ", ".join(missing)
                )

            if gateway_error is not None:
                raise _MATRIX.FaultValidationError(
                    "timed out waiting for gateway publisher discovery: "
                    f"{gateway_error}"
                )

            raise _MATRIX.FaultValidationError(
                "timed out waiting for ROS graph discovery"
            )

        #
        # Re-check the scenario invariants after the DDS-discovery phase.
        #
        _MATRIX.validate_observed_statuses(
            case,
            node.observed_statuses,
        )

        _MATRIX.validate_ground_truth_stamps(
            node.ground_truth_stamps
        )

        result = {
            "result": "PASS",
            "family": case.family,
            "scenario_id": case.scenario_id,
            "benchmark_duration_s": case.benchmark_duration_s,
            "requires_motion": case.requires_motion,
            "require_multimodal_particles":
                case.require_multimodal_particles,
            "fault_states": {
                fault_id: value["state"]
                for fault_id, value
                in sorted(node.observed_statuses.items())
            },
            "ground_truth_samples":
                len(node.ground_truth_stamps),
        }

        print(
            json.dumps(
                result,
                indent=2,
                sort_keys=True,
            )
        )

        return 0

    except _MATRIX.FaultValidationError as exc:
        print(
            f"ERROR: {exc}",
            file=sys.stderr,
        )
        return 2

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())