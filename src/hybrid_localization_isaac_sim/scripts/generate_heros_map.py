#!/usr/bin/env python3
"""Generate the HEROS occupancy map from the shared simulator layout."""

from __future__ import annotations

import json
from pathlib import Path

from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[1]
LAYOUT_PATH = ROOT / "config" / "heros_world_layout.json"
OUTPUT_PATH = ROOT / "maps" / "heros_localization_map.png"


def main() -> None:
    layout = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))

    width = int(layout["width_pixels"])
    height = int(layout["height_pixels"])

    image = Image.new("L", (width, height), 255)
    draw = ImageDraw.Draw(image)

    # Layout coordinates are expressed in ROS/world orientation:
    # +x points right and +y points up.
    #
    # Image coordinates use:
    # +x points right and +y points down.
    #
    # Therefore Y must be flipped when rasterizing the layout into the PNG.
    for rectangle in layout["occupied_rectangles"]:
        x0, y0, x1, y1 = (int(value) for value in rectangle)

        image_x0 = width - 1 - x1
        image_x1 = width - 1 - x0

        image_y0 = height - 1 - y1
        image_y1 = height - 1 - y0

        draw.rectangle(
            (image_x0, image_y0, image_x1, image_y1),
            fill=0,
        )

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    image.save(OUTPUT_PATH)

    print(f"Generated occupancy map: {OUTPUT_PATH}")
    print(f"Size: {width} x {height} pixels")
    print(f"Resolution: {layout['resolution']} m/pixel")
    print(f"Origin: {layout['origin']}")


if __name__ == "__main__":
    main()