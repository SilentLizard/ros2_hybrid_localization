#!/usr/bin/env python3

from pathlib import Path

from PIL import Image, ImageDraw


OUTPUT = Path("localization_test_map.png")
WIDTH = 400
HEIGHT = 400

image = Image.new("L", (WIDTH, HEIGHT), 255)
draw = ImageDraw.Draw(image)

wall = 6

# Outer boundary
draw.rectangle(
    [10, 10, WIDTH - 11, HEIGHT - 11],
    outline=0,
    width=wall,
)

# Internal walls
draw.rectangle([100, 10, 106, 260], fill=0)
# draw.rectangle([200, 140, 206, 390], fill=0)
draw.rectangle([300, 10, 306, 260], fill=0)

draw.rectangle([100, 140, 200, 146], fill=0)
draw.rectangle([206, 260, 306, 266], fill=0)

# A few block obstacles
draw.rectangle([40, 300, 75, 335], fill=0)
draw.rectangle([325, 300, 360, 350], fill=0)

image.save(OUTPUT)
print(f"Saved {OUTPUT.resolve()}")