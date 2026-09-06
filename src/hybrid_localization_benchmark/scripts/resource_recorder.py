#!/usr/bin/env python3
"""Record per-update process resources and particle-analysis timing."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from hybrid_localization_msgs.msg import ParticleAnalysis
from hybrid_localization_benchmark.resource_recording import (
    ProcessResourceTracker,
    ResourceCsvWriter,
    ResourceSample,
    find_unique_process,
    read_process_snapshot,
)


def stamp_ns(message: ParticleAnalysis) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class ResourceRecorder(Node):
    def __init__(self, run_directory: Path, pid: int, topic: str, expected_frame: str) -> None:
        super().__init__("hybrid_localization_resource_recorder")
        self._writer = ResourceCsvWriter(run_directory)
        self._pid = pid
        self._expected_frame = expected_frame
        self._tracker = ProcessResourceTracker()
        self._last_sim_time_ns: int | None = None

        # Establish the CPU baseline before accepting the first analysis update.
        baseline_time_ns = time.monotonic_ns()
        baseline_ticks, _ = read_process_snapshot(self._pid)
        self._tracker.update(baseline_time_ns, baseline_ticks)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscription = self.create_subscription(
            ParticleAnalysis,
            topic,
            self._on_analysis,
            qos,
        )
        self.get_logger().info(
            f"Recording process resources for PID {pid} from {topic} into {self._writer.path}"
        )

    def _on_analysis(self, message: ParticleAnalysis) -> None:
        if message.header.frame_id != self._expected_frame:
            self.get_logger().error(
                f"Rejected particle analysis in frame {message.header.frame_id!r}; "
                f"expected {self._expected_frame!r}"
            )
            return

        try:
            monotonic_time_ns = time.monotonic_ns()
            total_ticks, rss_bytes = read_process_snapshot(self._pid)
            cpu_percent = self._tracker.update(monotonic_time_ns, total_ticks)
            if cpu_percent is None:
                return

            sim_time_ns = stamp_ns(message)
            update_period_ns = (
                0 if self._last_sim_time_ns is None else sim_time_ns - self._last_sim_time_ns
            )
            sample = ResourceSample(
                monotonic_time_ns=monotonic_time_ns,
                sim_time_ns=sim_time_ns,
                process_cpu_percent=cpu_percent,
                process_rss_bytes=rss_bytes,
                update_duration_ns=int(message.processing_duration_ns),
                update_period_ns=update_period_ns,
            )
            self._writer.write(sample)
            self._last_sim_time_ns = sim_time_ns
        except Exception as exc:  # keep recorder alive but make bad samples visible
            self.get_logger().error(f"Resource sample rejected: {exc}")

    def destroy_node(self) -> bool:
        self._writer.close()
        return super().destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-directory", required=True, type=Path)
    parser.add_argument("--pid", type=int, default=0)
    parser.add_argument("--process-match", default="particle_analysis_observer")
    parser.add_argument("--topic", default="/hybrid_localization/particle_analysis")
    parser.add_argument("--expected-frame", default="map")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    pid = args.pid if args.pid > 0 else find_unique_process(args.process_match)

    rclpy.init()
    node = ResourceRecorder(args.run_directory, pid, args.topic, args.expected_frame)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
