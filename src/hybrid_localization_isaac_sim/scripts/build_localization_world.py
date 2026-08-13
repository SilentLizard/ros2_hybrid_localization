#!/usr/bin/env python3
"""Build/rebuild an Isaac localization collision world from a shared layout.

Run from Isaac Sim's Script Editor.  The layout is selected by:

  HYBRID_LOCALIZATION_WORLD_LAYOUT

The value may be an absolute path or a path relative to the
hybrid_localization_isaac_sim package root.  If unset, the committed HEROS
nominal layout is used.

The authoritative layout is stored in ROS map-cell coordinates.  Isaac geometry
is created directly in metric world coordinates; PNG/image-axis conventions do
not appear in this code path.
"""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path

import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics
from isaacsim.core.experimental.objects import GroundPlane


WORLD_ROOT_DEFAULT = "/World/HEROS_TestWorld"
PACKAGE_ENV = "HYBRID_LOCALIZATION_ISAAC_SIM_ROOT"
LAYOUT_ENV = "HYBRID_LOCALIZATION_WORLD_LAYOUT"
WORLD_ROOT_ENV = "HYBRID_LOCALIZATION_WORLD_ROOT"
DEFAULT_PACKAGE_ROOT = Path(
    "/workspace/ros2_hybrid_localization/src/hybrid_localization_isaac_sim"
)
DEFAULT_LAYOUT = "config/heros_world_layout.json"


def resolve_package_root() -> Path:
    candidates: list[Path] = []
    configured = os.environ.get(PACKAGE_ENV)
    if configured:
        candidates.append(Path(configured).expanduser())
    candidates.append(DEFAULT_PACKAGE_ROOT)
    try:
        candidates.append(Path(__file__).resolve().parents[1])
    except (NameError, IndexError):
        pass

    for candidate in candidates:
        if (candidate / "scripts" / "world_layout.py").is_file():
            return candidate

    checked = "\n  ".join(str(path) for path in candidates)
    raise FileNotFoundError(
        "Could not locate hybrid_localization_isaac_sim package root.\n"
        f"Checked:\n  {checked}\n"
        f"Set {PACKAGE_ENV} to the package directory inside the Isaac container."
    )


def _load_world_layout_module(package_root: Path):
    module_path = package_root / "scripts" / "world_layout.py"
    spec = importlib.util.spec_from_file_location("hybrid_world_layout", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load layout helper: {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def resolve_layout_path(package_root: Path, requested: str | None = None) -> Path:
    value = requested or os.environ.get(LAYOUT_ENV) or DEFAULT_LAYOUT
    candidate = Path(value).expanduser()
    if not candidate.is_absolute():
        candidate = package_root / candidate
    candidate = candidate.resolve()
    if not candidate.is_file():
        raise FileNotFoundError(f"Localization world layout not found: {candidate}")
    return candidate


def _create_box(stage, path: str, center, size, wall_height: float) -> None:
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(center[0], center[1], wall_height / 2.0))
    xform.AddScaleOp().Set(Gf.Vec3d(size[0], size[1], wall_height))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def _create_cylinder(stage, path: str, center, radius: float, wall_height: float) -> None:
    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateAxisAttr("Z")
    cylinder.CreateRadiusAttr(radius)
    cylinder.CreateHeightAttr(wall_height)
    xform = UsdGeom.Xformable(cylinder.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(center[0], center[1], wall_height / 2.0))
    UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())


def apply_layout(layout_path: str | None = None, world_root: str | None = None) -> Path:
    package_root = resolve_package_root()
    helpers = _load_world_layout_module(package_root)
    resolved_layout = resolve_layout_path(package_root, layout_path)
    layout = helpers.load_layout(resolved_layout)

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open")

    root = world_root or os.environ.get(WORLD_ROOT_ENV) or WORLD_ROOT_DEFAULT
    existing = stage.GetPrimAtPath(root)
    if existing and existing.IsValid():
        stage.RemovePrim(root)
    UsdGeom.Xform.Define(stage, root)

    width, height, resolution = helpers.map_dimensions(layout)
    origin_x, origin_y, _ = helpers.map_origin(layout)
    width_m = width * resolution
    height_m = height * resolution
    wall_height = float(layout["world"]["wall_height"])

    GroundPlane(
        f"{root}/GroundPlane",
        sizes=max(width_m, height_m),
        positions=[
            origin_x + width_m / 2.0,
            origin_y + height_m / 2.0,
            0.0,
        ],
    )

    for index, rectangle in enumerate(layout.get("occupied_rectangles", [])):
        x_min, y_min, x_max, y_max = helpers.cell_rect_to_world(rectangle, layout)
        _create_box(
            stage,
            f"{root}/Wall_{index:03d}",
            ((x_min + x_max) / 2.0, (y_min + y_max) / 2.0),
            (x_max - x_min, y_max - y_min),
            wall_height,
        )

    for index, circle in enumerate(layout.get("occupied_circles", [])):
        x, y, radius = helpers.circle_to_world(circle, layout)
        _create_cylinder(
            stage,
            f"{root}/Pillar_{index:03d}",
            (x, y),
            radius,
            wall_height,
        )

    print(f"Applied localization world layout: {resolved_layout}")
    print(f"World root: {root}")
    print(
        f"Created {len(layout.get('occupied_rectangles', []))} rectangles and "
        f"{len(layout.get('occupied_circles', []))} circles."
    )
    print(
        "Generate/load the matching ROS occupancy map from the same JSON before "
        "interpreting localization results."
    )
    return resolved_layout


if __name__ == "__main__":
    apply_layout()
