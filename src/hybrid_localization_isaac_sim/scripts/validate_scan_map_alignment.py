#!/usr/bin/env python3
"""Estimate scan-to-map endpoint agreement for the active localization fixture.

This is a diagnostic, not a localization benchmark. It checks whether transformed
finite LaserScan endpoints fall near occupied map cells, which is enough to catch
large fixture/map alignment errors before interpreting AMCL or GMM quality.
"""
from __future__ import annotations

import argparse
import math
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


class AlignmentValidator(Node):
    def __init__(self, tolerance_m: float) -> None:
        super().__init__("scan_map_alignment_validator")
        self.tolerance_m = tolerance_m
        self.map_msg = None
        self.scan_msg = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, map_qos)
        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        scan_qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(LaserScan, "/scan", self._scan_cb, scan_qos)

    def _map_cb(self, msg):
        self.map_msg = msg

    def _scan_cb(self, msg):
        self.scan_msg = msg

    def evaluate(self):
        if self.map_msg is None or self.scan_msg is None:
            return None
        scan = self.scan_msg
        grid = self.map_msg
        try:
            tf = self.tf_buffer.lookup_transform(
                grid.header.frame_id or "map",
                scan.header.frame_id,
                rclpy.time.Time.from_msg(scan.header.stamp),
            )
        except TransformException:
            return None

        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

        resolution = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        width, height = grid.info.width, grid.info.height
        radius_cells = max(1, int(math.ceil(self.tolerance_m / resolution)))

        considered = hits = 0
        angle = scan.angle_min
        for distance in scan.ranges:
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                sx = distance * math.cos(angle)
                sy = distance * math.sin(angle)
                mx = tx + cos_yaw * sx - sin_yaw * sy
                my = ty + sin_yaw * sx + cos_yaw * sy
                ix = int(math.floor((mx - ox) / resolution))
                iy = int(math.floor((my - oy) / resolution))
                if 0 <= ix < width and 0 <= iy < height:
                    considered += 1
                    found = False
                    for dy in range(-radius_cells, radius_cells + 1):
                        for dx in range(-radius_cells, radius_cells + 1):
                            x, y = ix + dx, iy + dy
                            if 0 <= x < width and 0 <= y < height and grid.data[y * width + x] >= 65:
                                found = True
                                break
                        if found:
                            break
                    hits += int(found)
            angle += scan.angle_increment

        if considered == 0:
            return 0, 0, 0.0
        return considered, hits, hits / considered


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--tolerance", type=float, default=0.20)
    parser.add_argument("--minimum-hit-fraction", type=float, default=0.50)
    args = parser.parse_args()

    rclpy.init()
    node = AlignmentValidator(args.tolerance)
    deadline = time.monotonic() + args.timeout
    result = None
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            result = node.evaluate()
            if result is not None and result[0] >= 20:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if result is None or result[0] < 20:
        print("FAIL: insufficient synchronized map/scan/TF data")
        return 2

    considered, hits, fraction = result
    print(f"scan endpoints considered: {considered}")
    print(f"endpoints near occupied map cells: {hits}")
    print(f"hit fraction: {fraction:.3f}")
    if fraction < args.minimum_hit_fraction:
        print("FAIL: scan/map alignment is too poor for localization validation")
        return 1
    print("PASS: scan/map geometry is sufficiently aligned for observation testing")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
