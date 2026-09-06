#!/usr/bin/env python3
"""Deterministically reset the HEROS robot pose from a runtime scenario.

This is #43 Stage 2.  The scenario contract remains simulator-independent in
``scenario_runtime.py``; this module is the thin Isaac-specific consumer.

The reset deliberately affects only physical simulator state:

* select a validated runtime scenario;
* apply its robot SE(2) pose to the HEROS articulation root;
* preserve the articulation's current world Z coordinate;
* clear root and joint velocities;
* read the pose back and verify it within configured tolerances.

AMCL/localization-belief reset, ROS ground-truth publication, and synchronized
world/map/localization orchestration are intentionally deferred to the next
#43 stage.

Run this file from Isaac Sim's Script Editor after the HEROS stage has been
loaded and physics has been initialized at least once.  For the most repeatable
manual test, pause the timeline before executing the reset and make sure no
non-zero ``/cmd_vel`` source is active.
"""

from __future__ import annotations

import copy
import importlib.util
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol


ROBOT_PRIM = "/World/heros_3w"

# Script-Editor defaults.  These can be changed in the editor without changing
# the committed runtime scenario.  Stage 3 will replace this manual entry point
# with synchronized scenario control.
SCENARIO_ID = os.environ.get("HYBRID_LOCALIZATION_SCENARIO", "S07")
ROBOT_POSE_OVERRIDE: tuple[float, float, float] | None = None
POSITION_TOLERANCE_M = 0.02
YAW_TOLERANCE_RAD = 0.005


class IsaacScenarioResetError(RuntimeError):
    """Raised when a requested Isaac robot reset cannot be completed safely."""


@dataclass(frozen=True)
class Pose2d:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Pose3d:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class ResetResult:
    scenario_id: str
    requested: Pose2d
    observed: Pose3d
    position_error_m: float
    yaw_error_rad: float
    motion_cleared: bool


class RobotResetBackend(Protocol):
    """Minimal simulator surface needed by the deterministic reset logic."""

    def read_pose(self) -> Pose3d:
        ...

    def teleport_se2(self, pose: Pose2d, *, preserve_z: float) -> None:
        ...

    def clear_motion(self) -> None:
        ...


def _package_root() -> Path:
    configured = os.environ.get("HYBRID_LOCALIZATION_ISAAC_SIM_ROOT")
    if configured:
        return Path(configured).expanduser()
    return Path("/workspace/ros2_hybrid_localization/src/hybrid_localization_isaac_sim")


def _load_scenario_runtime():
    module_path = _package_root() / "scripts" / "scenario_runtime.py"
    name = "scenario_runtime"
    spec = importlib.util.spec_from_file_location(name, module_path)
    if spec is None or spec.loader is None:
        raise IsaacScenarioResetError(f"Could not load scenario runtime module: {module_path}")

    module = importlib.util.module_from_spec(spec)
    previous = sys.modules.get(name)
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        if previous is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = previous
        raise
    return module


def _angle_difference(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def pose2d_from_scenario(scenario: dict[str, Any]) -> Pose2d:
    """Extract the already-validated physical robot pose from a scenario."""

    pose = scenario["robot"]["initial_pose"]
    return Pose2d(float(pose["x"]), float(pose["y"]), float(pose["yaw"]))


def scenario_for_reset(
    scenario_id: str,
    *,
    robot_pose_override: tuple[float, float, float] | None = None,
) -> dict[str, Any]:
    """Load a validated Stage-1 scenario and optionally override physical pose.

    The localization prior is intentionally left untouched.  Stage 2 controls
    simulator truth only; coupling truth to localization belief belongs to the
    synchronized controller in a later stage.
    """

    runtime = _load_scenario_runtime()
    catalog = runtime.load_catalog()
    if scenario_id not in catalog:
        raise IsaacScenarioResetError(f"Unknown runtime scenario {scenario_id!r}")

    scenario = copy.deepcopy(catalog[scenario_id])
    if robot_pose_override is not None:
        scenario = runtime.scenario_with_pose(
            scenario,
            x=robot_pose_override[0],
            y=robot_pose_override[1],
            yaw=robot_pose_override[2],
            localization_follows_robot=False,
        )
    return scenario


def validate_pose_match(
    requested: Pose2d,
    observed: Pose3d,
    *,
    position_tolerance_m: float = POSITION_TOLERANCE_M,
    yaw_tolerance_rad: float = YAW_TOLERANCE_RAD,
) -> tuple[float, float]:
    """Validate a stable post-physics pose against the requested SE(2) pose."""

    if not math.isfinite(position_tolerance_m) or position_tolerance_m < 0.0:
        raise IsaacScenarioResetError("position_tolerance_m must be finite and >= 0")
    if not math.isfinite(yaw_tolerance_rad) or yaw_tolerance_rad < 0.0:
        raise IsaacScenarioResetError("yaw_tolerance_rad must be finite and >= 0")

    position_error = math.hypot(observed.x - requested.x, observed.y - requested.y)
    yaw_error = abs(_angle_difference(observed.yaw, requested.yaw))

    if position_error > position_tolerance_m or yaw_error > yaw_tolerance_rad:
        raise IsaacScenarioResetError(
            "Robot reset read-back mismatch: "
            f"position error={position_error:.6g} m "
            f"(limit {position_tolerance_m:.6g}), "
            f"yaw error={yaw_error:.6g} rad "
            f"(limit {yaw_tolerance_rad:.6g})"
        )

    return position_error, yaw_error


def execute_reset(
    backend: RobotResetBackend,
    *,
    scenario_id: str,
    pose: Pose2d,
    position_tolerance_m: float = POSITION_TOLERANCE_M,
    yaw_tolerance_rad: float = YAW_TOLERANCE_RAD,
) -> ResetResult:
    """Teleport, clear motion, read back, and validate one robot reset."""

    before = backend.read_pose()
    backend.teleport_se2(pose, preserve_z=before.z)
    backend.clear_motion()
    observed = backend.read_pose()

    position_error, yaw_error = validate_pose_match(
        pose,
        observed,
        position_tolerance_m=position_tolerance_m,
        yaw_tolerance_rad=yaw_tolerance_rad,
    )

    result = ResetResult(
        scenario_id=scenario_id,
        requested=pose,
        observed=observed,
        position_error_m=position_error,
        yaw_error_rad=yaw_error,
        motion_cleared=True,
    )

    return result


def _yaw_to_wxyz(yaw: float):
    import numpy as np

    half = 0.5 * yaw
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=float)


def _wxyz_to_yaw(quaternion) -> float:
    w, x, y, z = (float(value) for value in quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class IsaacArticulationBackend:
    """Isaac Sim 6.0 adapter for the imported HEROS articulation."""

    def __init__(self, robot_prim: str = ROBOT_PRIM) -> None:
        import omni.usd
        from isaacsim.core.prims import SingleArticulation
        from pxr import UsdPhysics

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise IsaacScenarioResetError("No USD stage is open")

        candidates = [
            prim
            for prim in stage.Traverse()
            if (str(prim.GetPath()) == robot_prim or str(prim.GetPath()).startswith(robot_prim + "/"))
            and prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        ]
        if len(candidates) != 1:
            raise IsaacScenarioResetError(
                f"Expected exactly one articulation root below {robot_prim}; "
                f"found {[str(prim.GetPath()) for prim in candidates]}"
            )

        self.articulation_path = str(candidates[0].GetPath())
        self._articulation = SingleArticulation(
            prim_path=self.articulation_path,
            name="hybrid_localization_heros_reset",
        )
        try:
            self._articulation.initialize()
        except Exception as exc:  # Isaac exposes backend-specific exceptions.
            raise IsaacScenarioResetError(
                "Could not initialize the HEROS articulation. Start/initialize "
                "physics once, then pause the timeline and retry."
            ) from exc

    def refresh(self) -> None:
        """Re-bind the PhysX articulation view after stage/physics changes.

        Isaac documents that articulation views must be initialized again after
        hard physics resets.  Scenario application also edits the stage before
        teleportation, so refreshing here avoids validating against a stale or
        pre-sync physics view.
        """
        try:
            self._articulation.initialize()
        except Exception as exc:
            raise IsaacScenarioResetError(
                "Could not refresh the HEROS articulation physics view"
            ) from exc

    def read_pose(self) -> Pose3d:
        position, orientation = self._articulation.get_world_pose()
        return Pose3d(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
            yaw=_wxyz_to_yaw(orientation),
        )

    def teleport_se2(self, pose: Pose2d, *, preserve_z: float) -> None:
        import numpy as np

        self._articulation.set_world_pose(
            position=np.array([pose.x, pose.y, preserve_z], dtype=float),
            orientation=_yaw_to_wxyz(pose.yaw),
        )

    def clear_motion(self) -> None:
        import numpy as np

        # Root velocity order is [vx, vy, vz, wx, wy, wz].
        self._articulation.set_world_velocity(np.zeros(6, dtype=float))

        joint_velocities = self._articulation.get_joint_velocities()
        if joint_velocities is not None and len(joint_velocities) > 0:
            self._articulation.set_joint_velocities(np.zeros_like(joint_velocities))


def reset_from_scenario(
    scenario_id: str,
    *,
    robot_pose_override: tuple[float, float, float] | None = None,
    backend: RobotResetBackend | None = None,
) -> ResetResult:
    scenario = scenario_for_reset(
        scenario_id,
        robot_pose_override=robot_pose_override,
    )
    pose = pose2d_from_scenario(scenario)
    active_backend = backend if backend is not None else IsaacArticulationBackend()
    return execute_reset(
        active_backend,
        scenario_id=scenario_id,
        pose=pose,
    )


def _format_result(result: ResetResult) -> str:
    return (
        f"Reset {result.scenario_id}: requested "
        f"x={result.requested.x:.6f}, y={result.requested.y:.6f}, "
        f"yaw={result.requested.yaw:.6f}; observed "
        f"x={result.observed.x:.6f}, y={result.observed.y:.6f}, "
        f"z={result.observed.z:.6f}, yaw={result.observed.yaw:.6f}; "
        f"position_error={result.position_error_m:.3e} m, "
        f"yaw_error={result.yaw_error_rad:.3e} rad; motion cleared"
    )


if __name__ == "__main__":
    result = reset_from_scenario(
        SCENARIO_ID,
        robot_pose_override=ROBOT_POSE_OVERRIDE,
    )
    print(_format_result(result))
