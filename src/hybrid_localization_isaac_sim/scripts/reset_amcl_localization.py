#!/usr/bin/env python3
"""Reset AMCL without assuming the robot starts at map pose (0, 0, 0).

Modes:

  global
      Calls AMCL's reinitialize_global_localization service. AMCL uniformly
      samples free map cells and yaw. This is the preferred unknown-start test.

  random-prior
      Publishes a deterministic pseudo-random free-space initial pose with
      configurable covariance. This tests convergence from a repeatable,
      potentially incorrect prior while the simulated robot itself remains at
      its current physical pose.

This utility resets only the localization belief. It does not teleport the
Isaac robot; synchronized robot reset/teleport belongs to the scenario
controller and ground-truth milestone.
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import random
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_srvs.srv import Empty


def _load_world_layout(path: Path):
    spec = importlib.util.spec_from_file_location("world_layout", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _is_free(x_cell: float, y_cell: float, layout: dict) -> bool:
    for x0, y0, x1, y1 in layout.get("occupied_rectangles", []):
        if x0 <= x_cell <= x1 and y0 <= y_cell <= y1:
            return False
    for circle in layout.get("occupied_circles", []):
        cx, cy = (float(v) for v in circle["center_cells"])
        radius = float(circle["radius_cells"])
        if math.hypot(x_cell - cx, y_cell - cy) <= radius:
            return False
    return True


def _sample_free_pose(layout: dict, rng: random.Random) -> tuple[float, float, float]:
    width = int(layout["map"]["width_pixels"])
    height = int(layout["map"]["height_pixels"])
    resolution = float(layout["map"]["resolution"])
    origin_x, origin_y, _ = (float(v) for v in layout["map"]["origin"])
    margin = max(2, int(0.75 / resolution))

    for _ in range(10000):
        x_cell = rng.uniform(margin, width - margin - 1)
        y_cell = rng.uniform(margin, height - margin - 1)
        if not _is_free(x_cell, y_cell, layout):
            continue
        x = origin_x + (x_cell + 0.5) * resolution
        y = origin_y + (y_cell + 0.5) * resolution
        yaw = rng.uniform(-math.pi, math.pi)
        return x, y, yaw
    raise RuntimeError("Could not sample a free initial pose")


class AmclReset(Node):
    def __init__(self) -> None:
        super().__init__("hybrid_localization_amcl_reset")

    def global_reset(self, timeout: float) -> None:
        client = self.create_client(Empty, "/reinitialize_global_localization")
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                "AMCL global-localization service is unavailable: "
                "/reinitialize_global_localization"
            )
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.exception() is not None:
            raise RuntimeError("AMCL global-localization service call failed")
        self.get_logger().info("AMCL reset to a uniform global particle belief")

    def random_prior(
        self,
        layout: dict,
        seed: int,
        xy_stddev: float,
        yaw_stddev: float,
    ) -> None:
        rng = random.Random(seed)
        x, y, yaw = _sample_free_pose(layout, rng)

        publisher = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = xy_stddev * xy_stddev
        msg.pose.covariance[7] = xy_stddev * xy_stddev
        msg.pose.covariance[35] = yaw_stddev * yaw_stddev

        # Give DDS endpoint discovery a short bounded window before publishing.
        deadline = self.get_clock().now().nanoseconds + int(2.0e9)
        while (
            publisher.get_subscription_count() == 0
            and self.get_clock().now().nanoseconds < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.05)

        publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info(
            f"Published seeded AMCL prior seed={seed}: "
            f"x={x:.3f} y={y:.3f} yaw={yaw:.3f} rad, "
            f"xy_stddev={xy_stddev:.3f} yaw_stddev={yaw_stddev:.3f}"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("global", "random-prior"), required=True)
    parser.add_argument("--layout", type=Path)
    parser.add_argument("--seed", type=int, default=41)
    parser.add_argument("--xy-stddev", type=float, default=2.0)
    parser.add_argument("--yaw-stddev", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    if args.mode == "random-prior" and args.layout is None:
        parser.error("--layout is required for random-prior mode")

    rclpy.init()
    node = AmclReset()
    try:
        if args.mode == "global":
            node.global_reset(args.timeout)
        else:
            root = Path(__file__).resolve().parents[1]
            helper = _load_world_layout(root / "scripts" / "world_layout.py")
            layout = helper.load_layout(args.layout)
            node.random_prior(
                layout,
                args.seed,
                args.xy_stddev,
                args.yaw_stddev,
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
