#!/usr/bin/env python3
"""Render a ROS occupancy map from a schema-v2 localization-world layout."""

from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image, ImageDraw

from world_layout import (
    cell_rect_to_image,
    circle_to_image,
    load_layout,
    map_dimensions,
    map_origin,
)


def render_map(layout_path: Path, image_path: Path, yaml_path: Path | None) -> None:
    layout = load_layout(layout_path)
    width, height, resolution = map_dimensions(layout)
    origin = map_origin(layout)

    image = Image.new("L", (width, height), 255)
    draw = ImageDraw.Draw(image)

    for rectangle in layout.get("occupied_rectangles", []):
        draw.rectangle(cell_rect_to_image(rectangle, layout), fill=0)

    for circle in layout.get("occupied_circles", []):
        draw.ellipse(circle_to_image(circle, layout), fill=0)

    image_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(image_path)

    if yaml_path is not None:
        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        map_cfg = layout["map"]
        yaml_path.write_text(
            "\n".join(
                [
                    f"image: {image_path.name}",
                    f"resolution: {resolution}",
                    f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]",
                    "negate: 0",
                    f"occupied_thresh: {float(map_cfg.get('occupied_thresh', 0.65))}",
                    f"free_thresh: {float(map_cfg.get('free_thresh', 0.196))}",
                    f"mode: {map_cfg.get('mode', 'trinary')}",
                    "",
                ]
            ),
            encoding="utf-8",
        )

    print(f"Layout: {layout_path}")
    print(f"Occupancy image: {image_path}")
    if yaml_path is not None:
        print(f"Map YAML: {yaml_path}")
    print(f"Size: {width} x {height} cells")
    print(f"Resolution: {resolution} m/cell")
    print(f"Origin: {origin}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--layout", required=True, type=Path)
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument(
        "--yaml",
        type=Path,
        help="Optional Nav2 map YAML to write beside the generated image.",
    )
    args = parser.parse_args()
    render_map(args.layout, args.image, args.yaml)


if __name__ == "__main__":
    main()
