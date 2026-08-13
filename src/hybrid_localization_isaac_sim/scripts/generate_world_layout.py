#!/usr/bin/env python3
"""Generate deterministic schema-v2 localization layouts.

Every generated world is fully determined by its explicit parameters and seed.
Geometry is expressed in ROS map-cell coordinates, never PNG coordinates.

The generator deliberately keeps a configurable obstacle-free area around the
map origin so the HEROS fixture can remain physically spawned at (0, 0, 0)
while AMCL is tested with known, random-prior, or fully global initialization.
"""

from __future__ import annotations

import argparse
import json
import math
import random
from pathlib import Path


PRESETS = ("warehouse", "rooms", "mixed", "corridors")


def base_layout(name: str, width: int, height: int, resolution: float) -> dict:
    return {
        "schema_version": 2,
        "name": name,
        "coordinate_system": "ros_map_cells",
        "map": {
            "width_pixels": width,
            "height_pixels": height,
            "resolution": resolution,
            "origin": [
                -0.5 * width * resolution,
                -0.5 * height * resolution,
                0.0,
            ],
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
            "mode": "trinary",
        },
        "world": {
            "wall_height": 1.5,
            "floor_thickness": 0.10,
        },
        "occupied_rectangles": [],
        "occupied_circles": [],
    }


def add_border(layout: dict, thickness: int | None = None) -> None:
    width = layout["map"]["width_pixels"]
    height = layout["map"]["height_pixels"]
    thickness = thickness or max(5, min(width, height) // 120)
    layout["occupied_rectangles"].extend(
        [
            [0, 0, width - 1, thickness - 1],
            [0, height - thickness, width - 1, height - 1],
            [0, thickness, thickness - 1, height - thickness - 1],
            [width - thickness, thickness, width - 1, height - thickness - 1],
        ]
    )


def _spawn_box(layout: dict, clearance_m: float) -> tuple[float, float, float, float]:
    width = layout["map"]["width_pixels"]
    height = layout["map"]["height_pixels"]
    resolution = float(layout["map"]["resolution"])
    half = clearance_m / resolution
    cx = width / 2.0
    cy = height / 2.0
    return cx - half, cy - half, cx + half, cy + half


def _rect_intersects_box(rect: list[int], box: tuple[float, float, float, float]) -> bool:
    x0, y0, x1, y1 = rect
    bx0, by0, bx1, by1 = box
    return not (x1 < bx0 or x0 > bx1 or y1 < by0 or y0 > by1)


def _circle_intersects_box(circle: dict, box: tuple[float, float, float, float]) -> bool:
    cx, cy = (float(v) for v in circle["center_cells"])
    radius = float(circle["radius_cells"])
    bx0, by0, bx1, by1 = box
    nearest_x = min(max(cx, bx0), bx1)
    nearest_y = min(max(cy, by0), by1)
    return math.hypot(cx - nearest_x, cy - nearest_y) <= radius


def clear_spawn_area(layout: dict, clearance_m: float) -> None:
    box = _spawn_box(layout, clearance_m)
    layout["occupied_rectangles"] = [
        rect for rect in layout["occupied_rectangles"]
        if not _rect_intersects_box(rect, box)
    ]
    layout["occupied_circles"] = [
        circle for circle in layout["occupied_circles"]
        if not _circle_intersects_box(circle, box)
    ]


def _add_random_boxes(
    layout: dict,
    rng: random.Random,
    count: int,
    min_size_cells: int,
    max_size_cells: int,
) -> None:
    width = layout["map"]["width_pixels"]
    height = layout["map"]["height_pixels"]
    margin = max(12, min(width, height) // 40)
    for _ in range(count):
        w = rng.randint(min_size_cells, max_size_cells)
        h = rng.randint(min_size_cells, max_size_cells)
        x = rng.randint(margin, max(margin, width - margin - w - 1))
        y = rng.randint(margin, max(margin, height - margin - h - 1))
        layout["occupied_rectangles"].append([x, y, x + w, y + h])


def _add_random_pillars(layout: dict, rng: random.Random, count: int) -> None:
    width = layout["map"]["width_pixels"]
    height = layout["map"]["height_pixels"]
    margin = max(20, min(width, height) // 30)
    for _ in range(count):
        radius = rng.uniform(2.0, max(3.0, min(width, height) / 100.0))
        layout["occupied_circles"].append(
            {
                "center_cells": [
                    rng.uniform(margin, width - margin),
                    rng.uniform(margin, height - margin),
                ],
                "radius_cells": radius,
            }
        )


def _warehouse(layout: dict, rng: random.Random) -> None:
    width = layout["map"]["width_pixels"]
    height = layout["map"]["height_pixels"]
    shelf_width = max(5, width // 70)
    y0 = height // 8
    y1 = 7 * height // 8
    spacing = max(shelf_width * 5, width // 10)

    for x in range(width // 6, 5 * width // 6, spacing):
        gap_count = rng.choice((1, 2))
        cuts = sorted(rng.randint(height // 4, 3 * height // 4) for _ in range(gap_count))
        segments = [y0]
        gap_half = max(10, height // 35)
        for cut in cuts:
            segments.extend((cut - gap_half, cut + gap_half))
        segments.append(y1)
        for index in range(0, len(segments) - 1, 2):
            lo, hi = segments[index], segments[index + 1]
            if hi - lo > 5:
                layout["occupied_rectangles"].append(
                    [x, lo, min(x + shelf_width, width - 1), hi]
                )


def _rooms(layout: dict, rng: random.Random) -> None:
    width = layout["map"]["width_pixels"]
    height = layout["map"]["height_pixels"]
    wall = max(4, min(width, height) // 120)
    door = max(18, min(width, height) // 22)
    mid_x = width // 2
    mid_y = height // 2
    offsets = [
        (mid_x, "vertical"),
        (width // 3, "vertical"),
        (2 * width // 3, "vertical"),
        (mid_y, "horizontal"),
    ]
    for position, orientation in offsets:
        if rng.random() < 0.25:
            continue
        if orientation == "vertical":
            gap = rng.randint(height // 4, 3 * height // 4)
            layout["occupied_rectangles"].append(
                [position, 15, position + wall, max(15, gap - door)]
            )
            layout["occupied_rectangles"].append(
                [position, min(height - 16, gap + door), position + wall, height - 16]
            )
        else:
            gap = rng.randint(width // 4, 3 * width // 4)
            layout["occupied_rectangles"].append(
                [15, position, max(15, gap - door), position + wall]
            )
            layout["occupied_rectangles"].append(
                [min(width - 16, gap + door), position, width - 16, position + wall]
            )


def _corridors(layout: dict, rng: random.Random) -> None:
    width = layout["map"]["width_pixels"]
    height = layout["map"]["height_pixels"]
    wall = max(4, min(width, height) // 140)
    for i in range(5):
        horizontal = i % 2 == 0
        if horizontal:
            y = rng.randint(height // 7, 6 * height // 7)
            x0 = rng.randint(20, width // 3)
            x1 = rng.randint(2 * width // 3, width - 20)
            gap = rng.randint(x0 + 30, x1 - 30)
            gap_half = rng.randint(12, 30)
            layout["occupied_rectangles"].extend(
                [
                    [x0, y, gap - gap_half, y + wall],
                    [gap + gap_half, y, x1, y + wall],
                ]
            )
        else:
            x = rng.randint(width // 7, 6 * width // 7)
            y0 = rng.randint(20, height // 3)
            y1 = rng.randint(2 * height // 3, height - 20)
            gap = rng.randint(y0 + 30, y1 - 30)
            gap_half = rng.randint(12, 30)
            layout["occupied_rectangles"].extend(
                [
                    [x, y0, x + wall, gap - gap_half],
                    [x, gap + gap_half, x + wall, y1],
                ]
            )


def generate_layout(
    *,
    preset: str,
    seed: int,
    width: int,
    height: int,
    resolution: float,
    small_obstacles: int = 16,
    large_obstacles: int = 4,
    pillars: int = 6,
    spawn_clearance_m: float = 3.0,
    name: str | None = None,
) -> dict:
    if preset not in PRESETS:
        raise ValueError(f"unknown preset {preset!r}; expected one of {PRESETS}")
    if width < 100 or height < 100:
        raise ValueError("width and height must be at least 100 cells")
    if resolution <= 0.0:
        raise ValueError("resolution must be positive")
    if spawn_clearance_m < 0.0:
        raise ValueError("spawn_clearance_m must be non-negative")

    rng = random.Random(seed)
    layout = base_layout(
        name or f"{preset}_seed_{seed}",
        width,
        height,
        resolution,
    )
    add_border(layout)

    if preset == "warehouse":
        _warehouse(layout, rng)
    elif preset == "rooms":
        _rooms(layout, rng)
    elif preset == "corridors":
        _corridors(layout, rng)
    elif preset == "mixed":
        _warehouse(layout, rng)
        _corridors(layout, rng)

    min_dim = min(width, height)
    _add_random_boxes(
        layout,
        rng,
        large_obstacles,
        max(15, min_dim // 35),
        max(30, min_dim // 14),
    )
    _add_random_boxes(
        layout,
        rng,
        small_obstacles,
        max(3, min_dim // 180),
        max(10, min_dim // 65),
    )
    _add_random_pillars(layout, rng, pillars)
    clear_spawn_area(layout, spawn_clearance_m)

    layout["generator"] = {
        "preset": preset,
        "seed": seed,
        "small_obstacles": small_obstacles,
        "large_obstacles": large_obstacles,
        "pillars": pillars,
        "spawn_clearance_m": spawn_clearance_m,
    }
    return layout


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--preset", choices=PRESETS, default="warehouse")
    parser.add_argument("--width", type=int, default=800)
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=41)
    parser.add_argument("--small-obstacles", type=int, default=16)
    parser.add_argument("--large-obstacles", type=int, default=4)
    parser.add_argument("--pillars", type=int, default=6)
    parser.add_argument("--spawn-clearance", type=float, default=3.0)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    layout = generate_layout(
        preset=args.preset,
        seed=args.seed,
        width=args.width,
        height=args.height,
        resolution=args.resolution,
        small_obstacles=args.small_obstacles,
        large_obstacles=args.large_obstacles,
        pillars=args.pillars,
        spawn_clearance_m=args.spawn_clearance,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(layout, indent=2) + "\n", encoding="utf-8")
    print(f"Generated deterministic layout: {args.output}")
    print(f"  preset: {args.preset}")
    print(f"  seed: {args.seed}")
    print(f"  size: {args.width} x {args.height} cells")
    print(f"  resolution: {args.resolution} m/cell")


if __name__ == "__main__":
    main()
