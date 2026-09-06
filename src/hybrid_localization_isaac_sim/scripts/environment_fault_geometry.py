#!/usr/bin/env python3
"""ROS-/Isaac-independent geometry plans for #44 Stage 5 environment faults.

The plans are expressed relative to the robot pose at fault activation.  They
contain only deterministic metric primitive descriptions; Isaac/USD ownership
is isolated in ``isaac_environment_faults.py``.
"""

from __future__ import annotations

import math
from typing import NamedTuple


class EnvironmentFaultGeometryError(ValueError):
    """Raised when an unsupported Stage-5 environment profile is requested."""


class BoxPrimitive(NamedTuple):
    x_m: float
    y_m: float
    yaw_rad: float
    size_x_m: float
    size_y_m: float
    height_m: float


def _normalize_yaw(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


def _world_box(
    anchor_pose: tuple[float, float, float],
    *,
    forward_m: float,
    lateral_m: float,
    local_yaw_rad: float,
    size_x_m: float,
    size_y_m: float,
    height_m: float,
) -> BoxPrimitive:
    x, y, yaw = anchor_pose
    if not all(math.isfinite(value) for value in anchor_pose):
        raise EnvironmentFaultGeometryError("anchor pose must contain finite values")
    if not -math.pi <= yaw < math.pi:
        raise EnvironmentFaultGeometryError("anchor yaw must be in [-pi, pi)")

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return BoxPrimitive(
        x_m=x + cos_yaw * forward_m - sin_yaw * lateral_m,
        y_m=y + sin_yaw * forward_m + cos_yaw * lateral_m,
        yaw_rad=_normalize_yaw(yaw + local_yaw_rad),
        size_x_m=size_x_m,
        size_y_m=size_y_m,
        height_m=height_m,
    )


def map_mismatch_geometry(
    severity: str,
    variant: str,
    anchor_pose: tuple[float, float, float],
) -> tuple[BoxPrimitive, ...]:
    """Return deterministic world-only geometry for persistent map mismatch.

    The Nav2 map is deliberately left unchanged.  ``partial/added_box`` inserts
    one local obstacle.  ``severe/added_wall`` inserts a wide barrier plus two
    offset blocks so a much larger fraction of the expected scan disagrees with
    the map.
    """

    if severity == "partial" and variant == "added_box":
        return (
            _world_box(
                anchor_pose,
                forward_m=2.5,
                lateral_m=0.0,
                local_yaw_rad=0.0,
                size_x_m=0.8,
                size_y_m=0.8,
                height_m=2.0,
            ),
        )

    if severity == "severe" and variant == "added_wall":
        return (
            _world_box(
                anchor_pose,
                forward_m=3.0,
                lateral_m=0.0,
                local_yaw_rad=0.0,
                size_x_m=0.35,
                size_y_m=5.0,
                height_m=2.2,
            ),
            _world_box(
                anchor_pose,
                forward_m=1.8,
                lateral_m=2.2,
                local_yaw_rad=0.0,
                size_x_m=1.0,
                size_y_m=1.0,
                height_m=2.2,
            ),
            _world_box(
                anchor_pose,
                forward_m=1.8,
                lateral_m=-2.2,
                local_yaw_rad=0.0,
                size_x_m=1.0,
                size_y_m=1.0,
                height_m=2.2,
            ),
        )

    raise EnvironmentFaultGeometryError(
        "unsupported map mismatch profile; expected partial/added_box or "
        "severe/added_wall"
    )


def dynamic_obstruction_geometry(
    profile: str,
    anchor_pose: tuple[float, float, float],
) -> tuple[BoxPrimitive, ...]:
    """Return temporary physical geometry used to obstruct the live scan."""

    if profile != "front_box":
        raise EnvironmentFaultGeometryError(
            "unsupported dynamic obstruction profile; expected front_box"
        )

    return (
        _world_box(
            anchor_pose,
            forward_m=1.8,
            lateral_m=0.0,
            local_yaw_rad=0.0,
            size_x_m=0.7,
            size_y_m=0.7,
            height_m=2.0,
        ),
    )
