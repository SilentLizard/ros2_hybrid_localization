#!/usr/bin/env python3
"""ROS 2 Stage-2 recorder for AMCL localization accuracy."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from hybrid_localization_benchmark.localization_recording import (
    LocalizationCsvWriter,
    PoseSample,
    PoseSynchronizer,
)


def stamp_to_ns(stamp: object) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion values must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 0.0:
        raise ValueError("quaternion norm must be positive")
    x, y, z, w = (value / norm for value in values)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def pose_sample_from_message(message: PoseStamped | PoseWithCovarianceStamped) -> PoseSample:
    pose = message.pose.pose if isinstance(message, PoseWithCovarianceStamped) else message.pose
    return PoseSample(
        sim_time_ns=stamp_to_ns(message.header.stamp),
        x_m=float(pose.position.x),
        y_m=float(pose.position.y),
        yaw_rad=quaternion_to_yaw(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ),
    )


class LocalizationRecorderNode(Node):
    def __init__(
        self,
        run_directory: Path,
        ground_truth_topic: str,
        estimate_topic: str,
        max_sync_delta_ns: int,
        expected_frame: str,
    ) -> None:
        super().__init__("hybrid_localization_benchmark_localization_recorder")
        self._expected_frame = expected_frame
        self._synchronizer = PoseSynchronizer(max_sync_delta_ns)
        self._writer = LocalizationCsvWriter(run_directory)
        self._ground_truth_subscription = self.create_subscription(
            PoseStamped,
            ground_truth_topic,
            self._ground_truth_callback,
            50,
        )
        self._estimate_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            estimate_topic,
            self._estimate_callback,
            20,
        )
        self.get_logger().info(
            f"recording localization accuracy to {self._writer.path}; "
            f"ground_truth={ground_truth_topic}, estimate={estimate_topic}, "
            f"max_sync_delta_ns={max_sync_delta_ns}"
        )

    def _validate_frame(self, frame_id: str, source: str) -> None:
        if frame_id != self._expected_frame:
            raise ValueError(
                f"{source} frame must be {self._expected_frame!r}, got {frame_id!r}"
            )

    def _ground_truth_callback(self, message: PoseStamped) -> None:
        try:
            self._validate_frame(message.header.frame_id, "ground truth")
            self._write(self._synchronizer.add_ground_truth(pose_sample_from_message(message)))
        except ValueError as exc:
            self.get_logger().error(f"rejecting ground-truth sample: {exc}")

    def _estimate_callback(self, message: PoseWithCovarianceStamped) -> None:
        try:
            self._validate_frame(message.header.frame_id, "estimate")
            self._write(self._synchronizer.add_estimate(pose_sample_from_message(message)))
        except ValueError as exc:
            self.get_logger().error(f"rejecting localization estimate: {exc}")

    def _write(self, samples: list[object]) -> None:
        for sample in samples:
            self._writer.append(sample)  # type: ignore[arg-type]

    def close(self) -> None:
        self._write(self._synchronizer.finalize())
        self._writer.close()
        if rclpy.ok():
            self.get_logger().info(
                f"localization recorder stopped: matched={self._synchronizer.matched_count}, "
                f"dropped_estimates={self._synchronizer.dropped_estimate_count}"
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-directory", type=Path, required=True)
    parser.add_argument(
        "--ground-truth-topic",
        default="/hybrid_localization/ground_truth/pose",
    )
    parser.add_argument("--estimate-topic", default="/amcl_pose")
    parser.add_argument("--max-sync-delta-ms", type=float, default=50.0)
    parser.add_argument("--expected-frame", default="map")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.max_sync_delta_ms) or args.max_sync_delta_ms < 0.0:
        raise SystemExit("--max-sync-delta-ms must be finite and non-negative")
    max_sync_delta_ns = int(round(args.max_sync_delta_ms * 1_000_000.0))

    rclpy.init()
    node = LocalizationRecorderNode(
        run_directory=args.run_directory,
        ground_truth_topic=args.ground_truth_topic,
        estimate_topic=args.estimate_topic,
        max_sync_delta_ns=max_sync_delta_ns,
        expected_frame=args.expected_frame,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
