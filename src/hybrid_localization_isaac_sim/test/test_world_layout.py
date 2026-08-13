from __future__ import annotations

import importlib.util
import json
from pathlib import Path

from PIL import Image


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE_ROOT / "scripts"


def _load(name: str):
    path = SCRIPTS / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


world_layout = _load("world_layout")


def test_ros_cell_to_image_flips_y_only():
    layout = {
        "schema_version": 2,
        "name": "test",
        "coordinate_system": "ros_map_cells",
        "map": {
            "width_pixels": 10,
            "height_pixels": 8,
            "resolution": 0.5,
            "origin": [-2.5, -2.0, 0.0],
        },
        "world": {"wall_height": 1.0, "floor_thickness": 0.1},
        "occupied_rectangles": [[1, 2, 3, 4]],
        "occupied_circles": [],
    }
    world_layout.validate_layout(layout)
    assert world_layout.cell_rect_to_image([1, 2, 3, 4], layout) == (1, 3, 3, 5)


def test_ros_cell_to_world_does_not_flip_axes():
    layout = {
        "schema_version": 2,
        "name": "test",
        "coordinate_system": "ros_map_cells",
        "map": {
            "width_pixels": 10,
            "height_pixels": 8,
            "resolution": 0.5,
            "origin": [-2.5, -2.0, 0.0],
        },
        "world": {"wall_height": 1.0, "floor_thickness": 0.1},
        "occupied_rectangles": [[1, 2, 3, 4]],
        "occupied_circles": [],
    }
    assert world_layout.cell_rect_to_world([1, 2, 3, 4], layout) == (-2.0, -1.0, -0.5, 0.5)


def test_committed_heros_layout_is_valid():
    layout = world_layout.load_layout(PACKAGE_ROOT / "config" / "heros_world_layout.json")
    assert layout["name"] == "heros_nominal"
    assert len(layout["occupied_rectangles"]) == 11


def test_asymmetric_landmark_preserves_x_orientation():
    layout = world_layout.load_layout(PACKAGE_ROOT / "config" / "heros_world_layout.json")
    # Current top-right landmark in ROS coordinates must stay on the right side
    # of the map. This specifically guards against the historical extra X flip.
    x0, y0, x1, y1 = layout["occupied_rectangles"][8]
    assert x0 > layout["map"]["width_pixels"] // 2
    image_rect = world_layout.cell_rect_to_image([x0, y0, x1, y1], layout)
    assert image_rect[0] == x0
