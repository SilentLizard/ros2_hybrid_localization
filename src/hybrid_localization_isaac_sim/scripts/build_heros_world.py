#!/usr/bin/env python3
"""Build Isaac collision geometry from the same layout used by the ROS map.

Run from Isaac Sim's Script Editor. The generated static walls and floor live
under /World/HEROS_TestWorld.

Isaac Sim may execute Script Editor content from a temporary copy, so this
script must not rely solely on ``__file__`` to locate repository data.
Set HYBRID_LOCALIZATION_ISAAC_SIM_ROOT in the Isaac container, or mount the
repository at /workspace/ros2_hybrid_localization (the repository run script
does both).

The pixel-to-world conversion follows ROS OccupancyGrid image orientation:
image row 0 is the top of the map while the map origin is the lower-left corner.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics
from isaacsim.core.experimental.objects import GroundPlane

WORLD_ROOT = "/World/HEROS_TestWorld"
PACKAGE_ENV = "HYBRID_LOCALIZATION_ISAAC_SIM_ROOT"
DEFAULT_PACKAGE_ROOT = Path(
    "/workspace/ros2_hybrid_localization/src/hybrid_localization_isaac_sim"
)


def _resolve_package_root() -> Path:
    candidates: list[Path] = []

    configured = os.environ.get(PACKAGE_ENV)
    if configured:
        candidates.append(Path(configured).expanduser())

    candidates.append(DEFAULT_PACKAGE_ROOT)

    # Useful when the file is executed directly rather than through Isaac's
    # temporary Script Editor loader.
    try:
        candidates.append(Path(__file__).resolve().parents[1])
    except (NameError, IndexError):
        pass

    for candidate in candidates:
        layout_path = candidate / "config" / "heros_world_layout.json"
        if layout_path.is_file():
            return candidate

    checked = "\n  ".join(str(path) for path in candidates)
    raise FileNotFoundError(
        "Could not locate hybrid_localization_isaac_sim package root.\n"
        f"Checked:\n  {checked}\n"
        f"Set {PACKAGE_ENV} to the package directory inside the Isaac container."
    )


def _load_layout() -> dict:
    package_root = _resolve_package_root()
    layout_path = package_root / "config" / "heros_world_layout.json"
    print(f"Loading HEROS world layout from: {layout_path}")
    return json.loads(layout_path.read_text(encoding="utf-8"))


def _pixel_rect_to_world(rect, layout):
    x0, y0, x1, y1 = (int(value) for value in rect)
    resolution = float(layout["resolution"])
    height_pixels = int(layout["height_pixels"])
    origin_x, origin_y, _ = (float(value) for value in layout["origin"])

    x_min = origin_x + x0 * resolution
    x_max = origin_x + (x1 + 1) * resolution
    y_min = origin_y + (height_pixels - (y1 + 1)) * resolution
    y_max = origin_y + (height_pixels - y0) * resolution
    return x_min, y_min, x_max, y_max


def _create_box(stage, path, center, size):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*center))
    xform.AddScaleOp().Set(Gf.Vec3d(*size))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def main():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open")

    existing = stage.GetPrimAtPath(WORLD_ROOT)
    if existing and existing.IsValid():
        stage.RemovePrim(WORLD_ROOT)

    layout = _load_layout()
    UsdGeom.Xform.Define(stage, WORLD_ROOT)

    resolution = float(layout["resolution"])
    width_m = int(layout["width_pixels"]) * resolution
    height_m = int(layout["height_pixels"]) * resolution
    origin_x, origin_y, _ = (float(value) for value in layout["origin"])
    floor_thickness = float(layout["floor_thickness"])
    wall_height = float(layout["wall_height"])

    GroundPlane(
        f"{WORLD_ROOT}/GroundPlane",
        sizes=max(width_m, height_m),
        positions=[
            origin_x + width_m / 2.0,
            origin_y + height_m / 2.0,
            0.0,
        ],
    )

    for index, rectangle in enumerate(layout["occupied_rectangles"]):
        x_min, y_min, x_max, y_max = _pixel_rect_to_world(rectangle, layout)
        _create_box(
            stage,
            f"{WORLD_ROOT}/Wall_{index:02d}",
            ((x_min + x_max) / 2.0, (y_min + y_max) / 2.0, wall_height / 2.0),
            (x_max - x_min, y_max - y_min, wall_height),
        )

    print(
        f"Created {len(layout['occupied_rectangles'])} occupied rectangles "
        f"under {WORLD_ROOT}"
    )
    print(
        "The generated collision geometry and heros_localization_map.png "
        "share one layout source."
    )


main()
