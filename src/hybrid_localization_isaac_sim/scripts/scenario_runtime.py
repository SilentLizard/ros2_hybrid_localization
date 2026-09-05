#!/usr/bin/env python3
"""Runtime localization-scenario contract for deterministic Isaac experiments.

World geometry remains defined by ``config/world_scenarios.json``.  This module
adds the runtime state needed by localization experiments without coupling the
contract to Isaac Sim APIs:

* stable runtime scenario identity;
* referenced world-scenario identity;
* deterministic runtime seed;
* canonical physical robot pose in the map/world frame;
* independent AMCL initialization policy and, when applicable, initial pose.

Isaac robot teleportation, simulator ground-truth publication, and synchronized
reset execution intentionally belong to later #43 stages.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RUNTIME_CATALOG = ROOT / "config" / "localization_scenarios.json"
DEFAULT_WORLD_CATALOG = ROOT / "config" / "world_scenarios.json"

_ALLOWED_INITIALIZATION_MODES = {"known_pose", "global", "random_prior"}


class ScenarioRuntimeError(ValueError):
    """Raised when a runtime localization scenario violates the contract."""


def _finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _require_object(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ScenarioRuntimeError(f"{path} must be an object")
    return value


def _validate_pose(value: Any, path: str) -> None:
    pose = _require_object(value, path)
    if set(pose) != {"x", "y", "yaw"}:
        raise ScenarioRuntimeError(f"{path} must contain exactly x, y, and yaw")
    for key in ("x", "y", "yaw"):
        if not _finite_number(pose[key]):
            raise ScenarioRuntimeError(f"{path}.{key} must be finite")
    if not -math.pi <= float(pose["yaw"]) < math.pi:
        raise ScenarioRuntimeError(f"{path}.yaw must be in [-pi, pi)")


def _validate_positive_stddev(value: Any, path: str) -> None:
    if not _finite_number(value) or float(value) <= 0.0:
        raise ScenarioRuntimeError(f"{path} must be finite and > 0")


def validate_scenario(scenario: Any, world_ids: set[str] | None = None) -> None:
    scenario = _require_object(scenario, "scenario")
    required = {"id", "world_scenario", "seed", "robot", "localization"}
    missing = required.difference(scenario)
    if missing:
        raise ScenarioRuntimeError(
            "scenario is missing required field(s): " + ", ".join(sorted(missing))
        )

    scenario_id = scenario["id"]
    if not isinstance(scenario_id, str) or not scenario_id.strip():
        raise ScenarioRuntimeError("scenario.id must be a non-empty string")

    world_scenario = scenario["world_scenario"]
    if not isinstance(world_scenario, str) or not world_scenario.strip():
        raise ScenarioRuntimeError("scenario.world_scenario must be a non-empty string")
    if world_ids is not None and world_scenario not in world_ids:
        raise ScenarioRuntimeError(
            f"scenario.world_scenario references unknown world {world_scenario!r}"
        )

    seed = scenario["seed"]
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ScenarioRuntimeError("scenario.seed must be a non-negative integer")

    robot = _require_object(scenario["robot"], "scenario.robot")
    if set(robot) != {"initial_pose"}:
        raise ScenarioRuntimeError("scenario.robot must contain exactly initial_pose")
    _validate_pose(robot["initial_pose"], "scenario.robot.initial_pose")

    localization = _require_object(scenario["localization"], "scenario.localization")
    mode = localization.get("mode")
    if mode not in _ALLOWED_INITIALIZATION_MODES:
        raise ScenarioRuntimeError(
            "scenario.localization.mode must be one of: "
            + ", ".join(sorted(_ALLOWED_INITIALIZATION_MODES))
        )

    if mode == "known_pose":
        if set(localization) != {"mode", "initial_pose", "xy_stddev", "yaw_stddev"}:
            raise ScenarioRuntimeError(
                "known_pose localization must contain mode, initial_pose, xy_stddev, and yaw_stddev"
            )
        _validate_pose(localization["initial_pose"], "scenario.localization.initial_pose")
        _validate_positive_stddev(localization["xy_stddev"], "scenario.localization.xy_stddev")
        _validate_positive_stddev(localization["yaw_stddev"], "scenario.localization.yaw_stddev")
    elif mode == "random_prior":
        if set(localization) != {"mode", "seed", "xy_stddev", "yaw_stddev"}:
            raise ScenarioRuntimeError(
                "random_prior localization must contain mode, seed, xy_stddev, and yaw_stddev"
            )
        prior_seed = localization["seed"]
        if isinstance(prior_seed, bool) or not isinstance(prior_seed, int) or prior_seed < 0:
            raise ScenarioRuntimeError(
                "scenario.localization.seed must be a non-negative integer"
            )
        _validate_positive_stddev(localization["xy_stddev"], "scenario.localization.xy_stddev")
        _validate_positive_stddev(localization["yaw_stddev"], "scenario.localization.yaw_stddev")
    else:
        if set(localization) != {"mode"}:
            raise ScenarioRuntimeError("global localization must contain only mode")


def _load_world_ids(path: Path) -> set[str]:
    data = json.loads(path.read_text(encoding="utf-8"))
    scenarios = data.get("scenarios")
    if not isinstance(scenarios, list):
        raise ScenarioRuntimeError("world catalog scenarios must be an array")
    ids: list[str] = []
    for index, scenario in enumerate(scenarios):
        if not isinstance(scenario, dict) or not isinstance(scenario.get("id"), str):
            raise ScenarioRuntimeError(f"world catalog scenario {index} has no valid id")
        ids.append(scenario["id"])
    if len(ids) != len(set(ids)):
        raise ScenarioRuntimeError("world scenario IDs must be unique")
    return set(ids)


def load_catalog(
    runtime_path: Path = DEFAULT_RUNTIME_CATALOG,
    world_path: Path = DEFAULT_WORLD_CATALOG,
) -> dict[str, dict[str, Any]]:
    data = json.loads(runtime_path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise ScenarioRuntimeError("runtime scenario catalog schema_version must be 1")
    scenarios = data.get("scenarios")
    if not isinstance(scenarios, list):
        raise ScenarioRuntimeError("runtime scenario catalog scenarios must be an array")

    world_ids = _load_world_ids(world_path)
    by_id: dict[str, dict[str, Any]] = {}
    for scenario in scenarios:
        validate_scenario(scenario, world_ids)
        scenario_id = scenario["id"]
        if scenario_id in by_id:
            raise ScenarioRuntimeError(f"duplicate runtime scenario ID {scenario_id!r}")
        by_id[scenario_id] = copy.deepcopy(scenario)
    return by_id


def scenario_with_pose(
    scenario: dict[str, Any],
    *,
    x: float,
    y: float,
    yaw: float,
    localization_follows_robot: bool = False,
) -> dict[str, Any]:
    """Return a validated scenario copy with an arbitrary physical start pose.

    ``localization_follows_robot`` updates a known-pose AMCL prior as well.  It
    is deliberately explicit so physical truth and localization belief cannot
    be accidentally coupled during future error/recovery experiments.
    """

    result = copy.deepcopy(scenario)
    pose = {"x": x, "y": y, "yaw": yaw}
    _validate_pose(pose, "pose override")
    result["robot"]["initial_pose"] = pose

    if localization_follows_robot:
        localization = result["localization"]
        if localization["mode"] != "known_pose":
            raise ScenarioRuntimeError(
                "localization_follows_robot requires known_pose localization mode"
            )
        localization["initial_pose"] = copy.deepcopy(pose)

    validate_scenario(result)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect a deterministic runtime scenario")
    parser.add_argument("--catalog", type=Path, default=DEFAULT_RUNTIME_CATALOG)
    parser.add_argument("--world-catalog", type=Path, default=DEFAULT_WORLD_CATALOG)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--robot-pose", nargs=3, type=float, metavar=("X", "Y", "YAW"))
    parser.add_argument(
        "--localization-follows-robot",
        action="store_true",
        help="also update a known-pose localization prior when overriding robot pose",
    )
    args = parser.parse_args()

    catalog = load_catalog(args.catalog, args.world_catalog)
    if args.scenario not in catalog:
        parser.error(f"Unknown runtime scenario {args.scenario!r}")

    scenario = catalog[args.scenario]
    if args.robot_pose is not None:
        scenario = scenario_with_pose(
            scenario,
            x=args.robot_pose[0],
            y=args.robot_pose[1],
            yaw=args.robot_pose[2],
            localization_follows_robot=args.localization_follows_robot,
        )
    print(json.dumps(scenario, indent=2))


if __name__ == "__main__":
    main()
