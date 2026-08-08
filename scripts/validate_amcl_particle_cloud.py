#!/usr/bin/env python3
"""Validate the ROS 2 Jazzy Nav2 AMCL particle-cloud runtime contract.

Run this while AMCL is active and has a known initial pose. For publication-rate
measurements, move the robot enough to exceed AMCL's update_min_d/update_min_a
thresholds, or request a no-motion update.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from dataclasses import asdict, dataclass

import rclpy
from nav2_msgs.msg import ParticleCloud
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

EXPECTED_TYPE = "nav2_msgs/msg/ParticleCloud"


@dataclass
class SampleSummary:
    particle_count: int
    weight_sum: float
    minimum_weight: float
    maximum_weight: float
    frame_id: str
    stamp_seconds: float
    maximum_abs_pose_z: float
    maximum_abs_quaternion_xy: float
    maximum_quaternion_norm_error: float


class ParticleCloudValidator(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("validate_amcl_particle_cloud")
        self.topic = topic
        self.samples: list[SampleSummary] = []
        self.arrival_times: list[float] = []
        self.create_subscription(
            ParticleCloud,
            topic,
            self._on_cloud,
            qos_profile_sensor_data,
        )

    def _on_cloud(self, msg: ParticleCloud) -> None:
        if not msg.particles:
            minimum_weight = math.nan
            maximum_weight = math.nan
            weight_sum = 0.0
            max_pose_z = 0.0
            max_quaternion_xy = 0.0
            max_quaternion_norm_error = 0.0
        else:
            weights = [float(p.weight) for p in msg.particles]
            weight_sum = math.fsum(weights)
            minimum_weight = min(weights)
            maximum_weight = max(weights)
            max_pose_z = max(abs(float(p.pose.position.z)) for p in msg.particles)
            max_quaternion_xy = max(
                max(abs(float(p.pose.orientation.x)), abs(float(p.pose.orientation.y)))
                for p in msg.particles
            )
            max_quaternion_norm_error = max(
                abs(
                    math.sqrt(
                        float(p.pose.orientation.x) ** 2
                        + float(p.pose.orientation.y) ** 2
                        + float(p.pose.orientation.z) ** 2
                        + float(p.pose.orientation.w) ** 2
                    )
                    - 1.0
                )
                for p in msg.particles
            )

        stamp_seconds = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self.samples.append(
            SampleSummary(
                particle_count=len(msg.particles),
                weight_sum=weight_sum,
                minimum_weight=minimum_weight,
                maximum_weight=maximum_weight,
                frame_id=msg.header.frame_id,
                stamp_seconds=stamp_seconds,
                maximum_abs_pose_z=max_pose_z,
                maximum_abs_quaternion_xy=max_quaternion_xy,
                maximum_quaternion_norm_error=max_quaternion_norm_error,
            )
        )
        self.arrival_times.append(time.monotonic())


def qos_to_dict(info) -> dict:
    qos = info.qos_profile
    return {
        "node_name": info.node_name,
        "node_namespace": info.node_namespace,
        "topic_type": info.topic_type,
        "reliability": str(qos.reliability),
        "durability": str(qos.durability),
        "history": str(qos.history),
        "depth": int(qos.depth),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/particle_cloud")
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--expected-frame", default="map")
    parser.add_argument("--weight-tolerance", type=float, default=1e-6)
    parser.add_argument("--planar-tolerance", type=float, default=1e-9)
    parser.add_argument("--quaternion-tolerance", type=float, default=1e-6)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.samples <= 0 or args.timeout <= 0.0:
        print("--samples and --timeout must be positive", file=sys.stderr)
        return 2

    rclpy.init()
    node = ParticleCloudValidator(args.topic)
    failures: list[str] = []

    try:
        deadline = time.monotonic() + args.timeout

        # Give discovery a brief opportunity before evaluating the graph.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            names_and_types = dict(node.get_topic_names_and_types())
            if args.topic in names_and_types:
                break

        names_and_types = dict(node.get_topic_names_and_types())
        actual_types = names_and_types.get(args.topic, [])
        if EXPECTED_TYPE not in actual_types:
            failures.append(
                f"topic {args.topic!r} does not advertise expected type {EXPECTED_TYPE}; "
                f"observed={actual_types}"
            )

        # Endpoint discovery can lag topic/message discovery on some DDS setups.
        # Collect the requested samples first, then query publisher endpoints again.
        publisher_info = node.get_publishers_info_by_topic(args.topic)

        while len(node.samples) < args.samples and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)

        if not publisher_info:
            discovery_deadline = min(deadline, time.monotonic() + 2.0)
            while time.monotonic() < discovery_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                publisher_info = node.get_publishers_info_by_topic(args.topic)
                if publisher_info:
                    break

        if not node.samples:
            failures.append(
                "no ParticleCloud message received; ensure AMCL is ACTIVE and has a known "
                "initial pose. Move the robot or call /request_nomotion_update if needed."
            )
        elif len(node.samples) < args.samples:
            failures.append(
                f"received only {len(node.samples)} of {args.samples} requested samples before timeout"
            )

        frames = sorted({sample.frame_id for sample in node.samples})
        for i, sample in enumerate(node.samples):
            if sample.particle_count == 0:
                failures.append(f"sample {i}: particle array is empty")
                continue
            if not math.isfinite(sample.weight_sum):
                failures.append(f"sample {i}: weight sum is non-finite")
            if not math.isfinite(sample.minimum_weight) or sample.minimum_weight < 0.0:
                failures.append(f"sample {i}: negative or non-finite particle weight")
            if abs(sample.weight_sum - 1.0) > args.weight_tolerance:
                failures.append(
                    f"sample {i}: weights sum to {sample.weight_sum:.17g}, expected 1 within "
                    f"{args.weight_tolerance:g}"
                )
            if sample.frame_id != args.expected_frame:
                failures.append(
                    f"sample {i}: frame_id={sample.frame_id!r}, expected {args.expected_frame!r}"
                )
            if sample.maximum_abs_pose_z > args.planar_tolerance:
                failures.append(
                    f"sample {i}: non-planar pose z up to {sample.maximum_abs_pose_z:g}"
                )
            if sample.maximum_abs_quaternion_xy > args.planar_tolerance:
                failures.append(
                    f"sample {i}: quaternion x/y exceed planar tolerance: "
                    f"{sample.maximum_abs_quaternion_xy:g}"
                )
            if sample.maximum_quaternion_norm_error > args.quaternion_tolerance:
                failures.append(
                    f"sample {i}: quaternion norm error up to "
                    f"{sample.maximum_quaternion_norm_error:g}"
                )

        intervals = [
            later - earlier
            for earlier, later in zip(node.arrival_times, node.arrival_times[1:])
            if later > earlier
        ]
        measured_hz = None
        if intervals:
            measured_hz = 1.0 / statistics.mean(intervals)

        notes: list[str] = []
        if not publisher_info and node.samples:
            notes.append(
                "publisher endpoint metadata was not returned by rclpy graph discovery; "
                "message reception proves that a publisher exists. Use "
                "'ros2 topic info <topic> --verbose' to record endpoint QoS."
            )
        elif not publisher_info:
            failures.append(f"no publisher discovered for {args.topic!r}")

        report = {
            "topic": args.topic,
            "expected_type": EXPECTED_TYPE,
            "advertised_types": actual_types,
            "publishers": [qos_to_dict(info) for info in publisher_info],
            "received_samples": len(node.samples),
            "frames": frames,
            "measured_receive_rate_hz": measured_hz,
            "samples": [asdict(sample) for sample in node.samples],
            "result": "PASS" if not failures else "FAIL",
            "failures": failures,
            "notes": notes,
        }
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0 if not failures else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
