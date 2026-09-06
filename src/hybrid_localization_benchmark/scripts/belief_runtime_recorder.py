#!/usr/bin/env python3
"""Record ParticleAnalysis belief metrics into a Stage-3 benchmark stream."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from hybrid_localization_benchmark.belief_runtime_recording import (
    BeliefRuntimeCsvWriter,
    BeliefRuntimeSample,
)
from hybrid_localization_msgs.msg import ParticleAnalysis


DEFAULT_TOPIC = "/hybrid_localization/particle_analysis"
DEFAULT_FRAME = "map"


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class BeliefRuntimeRecorderNode(Node):
    def __init__(self, run_directory: Path, topic: str, expected_frame: str) -> None:
        super().__init__("hybrid_localization_belief_runtime_recorder")
        self._writer = BeliefRuntimeCsvWriter(run_directory)
        self._expected_frame = expected_frame
        self._accepted = 0
        self._rejected = 0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
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
            f"recording {topic} -> {self._writer.path} (frame={expected_frame})"
        )

    def _on_analysis(self, message: ParticleAnalysis) -> None:
        try:
            if message.header.frame_id != self._expected_frame:
                raise ValueError(
                    f"unexpected frame {message.header.frame_id!r}; "
                    f"expected {self._expected_frame!r}"
                )

            sample = BeliefRuntimeSample(
                sim_time_ns=stamp_to_ns(message.header.stamp),
                particle_count=int(message.particle_count),
                effective_sample_size=float(message.effective_sample_size),
                gmm_component_count=int(message.health.component_count),
                gmm_represented_weight=float(message.health.represented_weight),
                gmm_discarded_weight=float(message.health.discarded_weight),
                gmm_entropy=float(message.health.normalized_mixture_entropy),
                dominant_component_weight=float(message.health.dominant_component_weight),
                analysis_sequence=int(message.analysis_sequence),
            )
            self._writer.write(sample)
            self._accepted += 1
        except (ValueError, OSError) as exc:
            self._rejected += 1
            self.get_logger().error(f"rejected particle-analysis sample: {exc}")

    def destroy_node(self) -> bool:
        self._writer.close()
        if rclpy.ok():
            self.get_logger().info(
                f"belief-runtime recorder stopped: accepted={self._accepted}, "
                f"rejected={self._rejected}"
            )
        return super().destroy_node()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record particle/GMM belief metrics into belief_runtime.csv."
    )
    parser.add_argument("--run-directory", type=Path, required=True)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--expected-frame", default=DEFAULT_FRAME)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init(args=None)
    node = BeliefRuntimeRecorderNode(args.run_directory, args.topic, args.expected_frame)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
