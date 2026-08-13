from __future__ import annotations

import importlib.util
import json
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE_ROOT / "scripts"


def _load(name: str):
    path = SCRIPTS / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


generator = _load("generate_world_layout")
world_layout = _load("world_layout")


def test_catalog_contains_twenty_unique_deterministic_scenarios():
    catalog = json.loads(
        (PACKAGE_ROOT / "config" / "world_scenarios.json").read_text()
    )
    scenarios = catalog["scenarios"]
    assert len(scenarios) == 20
    assert len({scenario["id"] for scenario in scenarios}) == 20


def test_all_catalog_scenarios_generate_valid_layouts():
    catalog = json.loads(
        (PACKAGE_ROOT / "config" / "world_scenarios.json").read_text()
    )
    for scenario in catalog["scenarios"]:
        layout = generator.generate_layout(
            preset=scenario["preset"],
            seed=scenario["seed"],
            width=scenario["width"],
            height=scenario["height"],
            resolution=scenario["resolution"],
            small_obstacles=scenario["small_obstacles"],
            large_obstacles=scenario["large_obstacles"],
            pillars=scenario["pillars"],
            spawn_clearance_m=scenario["spawn_clearance_m"],
            name=scenario["id"],
        )
        world_layout.validate_layout(layout)


def test_same_seed_and_configuration_generate_identical_layout():
    kwargs = dict(
        preset="mixed",
        seed=12345,
        width=900,
        height=800,
        resolution=0.05,
        small_obstacles=22,
        large_obstacles=5,
        pillars=8,
        spawn_clearance_m=3.0,
        name="determinism",
    )
    assert generator.generate_layout(**kwargs) == generator.generate_layout(**kwargs)


def test_different_seed_changes_generated_layout():
    kwargs = dict(
        preset="warehouse",
        width=800,
        height=800,
        resolution=0.05,
        small_obstacles=16,
        large_obstacles=4,
        pillars=6,
        spawn_clearance_m=3.0,
        name="seed-test",
    )
    a = generator.generate_layout(seed=1, **kwargs)
    b = generator.generate_layout(seed=2, **kwargs)
    assert a["occupied_rectangles"] != b["occupied_rectangles"] or (
        a["occupied_circles"] != b["occupied_circles"]
    )


def test_generated_world_keeps_origin_spawn_area_clear():
    layout = generator.generate_layout(
        preset="mixed",
        seed=99,
        width=1000,
        height=1000,
        resolution=0.05,
        small_obstacles=30,
        large_obstacles=8,
        pillars=12,
        spawn_clearance_m=3.0,
    )
    cx = layout["map"]["width_pixels"] / 2.0
    cy = layout["map"]["height_pixels"] / 2.0
    half = 3.0 / layout["map"]["resolution"]

    for x0, y0, x1, y1 in layout["occupied_rectangles"]:
        assert x1 < cx - half or x0 > cx + half or y1 < cy - half or y0 > cy + half

    for circle in layout["occupied_circles"]:
        x, y = circle["center_cells"]
        r = circle["radius_cells"]
        nearest_x = min(max(x, cx - half), cx + half)
        nearest_y = min(max(y, cy - half), cy + half)
        assert (x - nearest_x) ** 2 + (y - nearest_y) ** 2 > r ** 2
