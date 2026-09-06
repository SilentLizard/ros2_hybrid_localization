#!/usr/bin/env python3
"""Shared localization-world layout parsing and coordinate conversion.

Schema v2 stores obstacle geometry in ROS map-cell coordinates:

* cell (0, 0) is the lower-left cell of the occupancy grid;
* +x points right;
* +y points up;
* rectangle bounds are inclusive integer cell indices.

This deliberately avoids using PNG/image coordinates as an interchange format.
PNG row coordinates point downward, while ROS OccupancyGrid/map coordinates
point upward. Keeping the authoritative layout in ROS coordinates makes the
Isaac and occupancy-map conversions explicit and testable.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any


SCHEMA_VERSION = 2
COORDINATE_SYSTEM = "ros_map_cells"


class LayoutError(ValueError):
    """Raised when a localization-world layout violates the schema contract."""


def load_layout(path: str | Path) -> dict[str, Any]:
    """Load and validate one schema-v2 localization-world JSON file."""
    path = Path(path)
    layout = json.loads(path.read_text(encoding="utf-8"))
    validate_layout(layout)
    return layout


def validate_layout(layout: dict[str, Any]) -> None:
    """Validate map metadata and supported obstacle primitives."""
    if int(layout.get("schema_version", -1)) != SCHEMA_VERSION:
        raise LayoutError(
            f"Unsupported schema_version {layout.get('schema_version')!r}; "
            f"expected {SCHEMA_VERSION}"
        )
    if layout.get("coordinate_system") != COORDINATE_SYSTEM:
        raise LayoutError(
            f"Unsupported coordinate_system {layout.get('coordinate_system')!r}; "
            f"expected {COORDINATE_SYSTEM!r}"
        )

    map_cfg = layout.get("map")
    world_cfg = layout.get("world")
    if not isinstance(map_cfg, dict) or not isinstance(world_cfg, dict):
        raise LayoutError("layout must contain 'map' and 'world' objects")

    width = _positive_int(map_cfg.get("width_pixels"), "map.width_pixels")
    height = _positive_int(map_cfg.get("height_pixels"), "map.height_pixels")
    resolution = _positive_float(map_cfg.get("resolution"), "map.resolution")

    origin = map_cfg.get("origin")
    if (
        not isinstance(origin, list)
        or len(origin) != 3
        or any(not _finite_number(value) for value in origin)
    ):
        raise LayoutError("map.origin must contain three finite numbers")

    _positive_float(world_cfg.get("wall_height"), "world.wall_height")
    floor_thickness = world_cfg.get("floor_thickness", 0.0)
    if not _finite_number(floor_thickness) or float(floor_thickness) < 0.0:
        raise LayoutError("world.floor_thickness must be finite and non-negative")

    for index, rectangle in enumerate(layout.get("occupied_rectangles", [])):
        if not isinstance(rectangle, list) or len(rectangle) != 4:
            raise LayoutError(f"occupied_rectangles[{index}] must be [x0,y0,x1,y1]")
        x0, y0, x1, y1 = rectangle
        if any(not isinstance(value, int) or isinstance(value, bool) for value in rectangle):
            raise LayoutError(f"occupied_rectangles[{index}] must use integer cell indices")
        if not (0 <= x0 <= x1 < width and 0 <= y0 <= y1 < height):
            raise LayoutError(
                f"occupied_rectangles[{index}] lies outside {width}x{height} map"
            )

    for index, circle in enumerate(layout.get("occupied_circles", [])):
        if not isinstance(circle, dict):
            raise LayoutError(f"occupied_circles[{index}] must be an object")
        center = circle.get("center_cells")
        radius = circle.get("radius_cells")
        if (
            not isinstance(center, list)
            or len(center) != 2
            or any(not _finite_number(value) for value in center)
        ):
            raise LayoutError(
                f"occupied_circles[{index}].center_cells must contain two finite numbers"
            )
        if not _finite_number(radius) or float(radius) <= 0.0:
            raise LayoutError(
                f"occupied_circles[{index}].radius_cells must be positive"
            )
        cx, cy = (float(value) for value in center)
        radius = float(radius)
        if cx - radius < 0.0 or cy - radius < 0.0 or cx + radius > width or cy + radius > height:
            raise LayoutError(f"occupied_circles[{index}] lies outside map bounds")


def map_dimensions(layout: dict[str, Any]) -> tuple[int, int, float]:
    map_cfg = layout["map"]
    return (
        int(map_cfg["width_pixels"]),
        int(map_cfg["height_pixels"]),
        float(map_cfg["resolution"]),
    )


def map_origin(layout: dict[str, Any]) -> tuple[float, float, float]:
    return tuple(float(value) for value in layout["map"]["origin"])  # type: ignore[return-value]


def cell_rect_to_image(
    rectangle: list[int], layout: dict[str, Any]
) -> tuple[int, int, int, int]:
    """Convert an inclusive ROS-cell rectangle into an inclusive PNG rectangle."""
    _, height, _ = map_dimensions(layout)
    x0, y0, x1, y1 = rectangle
    return x0, height - 1 - y1, x1, height - 1 - y0


def cell_rect_to_world(
    rectangle: list[int], layout: dict[str, Any]
) -> tuple[float, float, float, float]:
    """Convert an inclusive ROS-cell rectangle to metric world bounds."""
    _, _, resolution = map_dimensions(layout)
    origin_x, origin_y, _ = map_origin(layout)
    x0, y0, x1, y1 = rectangle
    return (
        origin_x + x0 * resolution,
        origin_y + y0 * resolution,
        origin_x + (x1 + 1) * resolution,
        origin_y + (y1 + 1) * resolution,
    )


def circle_to_image(
    circle: dict[str, Any], layout: dict[str, Any]
) -> tuple[float, float, float, float]:
    """Convert a ROS-cell circle to a PNG ellipse bounding box."""
    _, height, _ = map_dimensions(layout)
    cx, cy = (float(value) for value in circle["center_cells"])
    radius = float(circle["radius_cells"])
    image_cy = (height - 1) - cy
    return (
        cx - radius,
        image_cy - radius,
        cx + radius,
        image_cy + radius,
    )


def circle_to_world(
    circle: dict[str, Any], layout: dict[str, Any]
) -> tuple[float, float, float]:
    """Return circle center x/y and radius in metric world coordinates."""
    _, _, resolution = map_dimensions(layout)
    origin_x, origin_y, _ = map_origin(layout)
    cx, cy = (float(value) for value in circle["center_cells"])
    radius = float(circle["radius_cells"])
    return (
        origin_x + cx * resolution,
        origin_y + cy * resolution,
        radius * resolution,
    )


def _finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _positive_int(value: Any, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise LayoutError(f"{name} must be a positive integer")
    return value


def _positive_float(value: Any, name: str) -> float:
    if not _finite_number(value) or float(value) <= 0.0:
        raise LayoutError(f"{name} must be a positive finite number")
    return float(value)
