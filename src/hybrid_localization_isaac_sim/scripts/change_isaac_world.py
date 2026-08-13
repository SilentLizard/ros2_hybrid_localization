#!/usr/bin/env python3
"""Replace the current Isaac localization collision world.

Run from Isaac Sim's Script Editor while a stage is open.

Selection priority:
1. TARGET_SCENARIO, e.g. "S07", materialized under generated/scenarios/S07;
2. TARGET_LAYOUT, a schema-v2 layout path;
3. HYBRID_LOCALIZATION_WORLD_SCENARIO environment variable;
4. HYBRID_LOCALIZATION_WORLD_LAYOUT environment variable.

This changes only the static Isaac collision world. It intentionally does not
reset the robot or AMCL.
"""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path


TARGET_SCENARIO: str | None = None
TARGET_LAYOUT: str | None = None
TARGET_SCENARIO = "S03"
TARGET_LAYOUT = None


def _package_root() -> Path:
    configured = os.environ.get("HYBRID_LOCALIZATION_ISAAC_SIM_ROOT")
    if configured:
        return Path(configured).expanduser()
    return Path(
        "/workspace/ros2_hybrid_localization/src/hybrid_localization_isaac_sim"
    )


def _load_builder():
    module_path = _package_root() / "scripts" / "build_localization_world.py"
    spec = importlib.util.spec_from_file_location("build_localization_world", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load generic world builder: {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _selected_layout(root: Path) -> str:
    scenario = TARGET_SCENARIO or os.environ.get("HYBRID_LOCALIZATION_WORLD_SCENARIO")
    if scenario:
        path = root / "generated" / "scenarios" / scenario / "layout.json"
        if not path.is_file():
            raise FileNotFoundError(
                f"Scenario {scenario!r} is not materialized: {path}\n"
                "Run materialize_world_scenario.py on the host first."
            )
        return str(path)

    layout = TARGET_LAYOUT or os.environ.get("HYBRID_LOCALIZATION_WORLD_LAYOUT")
    if layout:
        return layout

    raise RuntimeError(
        "Select TARGET_SCENARIO/TARGET_LAYOUT or set "
        "HYBRID_LOCALIZATION_WORLD_SCENARIO/HYBRID_LOCALIZATION_WORLD_LAYOUT."
    )


if __name__ == "__main__":
    root = _package_root()
    builder = _load_builder()
    selected = _selected_layout(root)
    print(f"Switching Isaac static world to: {selected}")
    builder.apply_layout(selected)
