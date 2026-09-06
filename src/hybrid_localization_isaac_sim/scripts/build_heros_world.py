#!/usr/bin/env python3
"""Compatibility wrapper that builds the committed HEROS nominal world.

This remains usable from Isaac Script Editor even when Isaac executes a
temporary copy of this file: the generic builder is loaded from the stable
package mount exposed through HYBRID_LOCALIZATION_ISAAC_SIM_ROOT.
"""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path


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


if __name__ == "__main__":
    _load_builder().apply_layout("config/heros_world_layout.json")
