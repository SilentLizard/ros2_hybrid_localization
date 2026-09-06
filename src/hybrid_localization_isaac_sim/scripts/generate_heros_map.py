#!/usr/bin/env python3
"""Generate the committed HEROS nominal occupancy map from its shared layout."""

from __future__ import annotations

from pathlib import Path

from generate_localization_map import render_map


ROOT = Path(__file__).resolve().parents[1]
LAYOUT_PATH = ROOT / "config" / "heros_world_layout.json"
IMAGE_PATH = ROOT / "maps" / "heros_localization_map.png"
YAML_PATH = ROOT / "maps" / "heros_localization_map.yaml"


if __name__ == "__main__":
    render_map(LAYOUT_PATH, IMAGE_PATH, YAML_PATH)
