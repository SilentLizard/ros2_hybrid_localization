#!/usr/bin/env python3
"""Materialize deterministic world scenarios from the committed catalog.

Generated artifacts are placed under:

  generated/scenarios/<ID>/
      layout.json
      map.png
      map.yaml
      scenario.json

The generated directory is intentionally not committed. The catalog plus the
generator version and seed are the reproducibility contract.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CATALOG = ROOT / "config" / "world_scenarios.json"
DEFAULT_OUTPUT_ROOT = ROOT / "generated" / "scenarios"


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


generator = _load_module(
    "generate_world_layout",
    ROOT / "scripts" / "generate_world_layout.py",
)
renderer = _load_module(
    "generate_localization_map",
    ROOT / "scripts" / "generate_localization_map.py",
)


def load_catalog(path: Path) -> dict[str, dict]:
    data = json.loads(path.read_text(encoding="utf-8"))
    scenarios = data.get("scenarios", [])
    by_id = {item["id"]: item for item in scenarios}
    if len(by_id) != len(scenarios):
        raise ValueError("scenario IDs must be unique")
    return by_id


def materialize(scenario: dict, output_root: Path) -> Path:
    scenario_id = scenario["id"]
    out = output_root / scenario_id
    out.mkdir(parents=True, exist_ok=True)

    layout = generator.generate_layout(
        preset=scenario["preset"],
        seed=int(scenario["seed"]),
        width=int(scenario["width"]),
        height=int(scenario["height"]),
        resolution=float(scenario["resolution"]),
        small_obstacles=int(scenario["small_obstacles"]),
        large_obstacles=int(scenario["large_obstacles"]),
        pillars=int(scenario["pillars"]),
        spawn_clearance_m=float(scenario["spawn_clearance_m"]),
        name=scenario_id,
    )

    layout_path = out / "layout.json"
    layout_path.write_text(json.dumps(layout, indent=2) + "\n", encoding="utf-8")
    renderer.render_map(layout_path, out / "map.png", out / "map.yaml")

    manifest = {
        "scenario_id": scenario_id,
        "layout": "layout.json",
        "map_yaml": "map.yaml",
        "generator": layout["generator"],
        "map": layout["map"],
    }
    (out / "scenario.json").write_text(
        json.dumps(manifest, indent=2) + "\n",
        encoding="utf-8",
    )
    return out


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--scenario")
    group.add_argument("--all", action="store_true")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    args = parser.parse_args()

    catalog = load_catalog(args.catalog)
    ids = sorted(catalog) if args.all else [args.scenario]
    for scenario_id in ids:
        if scenario_id not in catalog:
            parser.error(f"Unknown scenario {scenario_id!r}")
        out = materialize(catalog[scenario_id], args.output_root)
        print(f"{scenario_id}: {out}")


if __name__ == "__main__":
    main()
