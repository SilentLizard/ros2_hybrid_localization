#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "reset_heros_scenario.py"
SPEC = importlib.util.spec_from_file_location("reset_heros_scenario", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class FakeBackend:
    def __init__(self, pose=None, readback_offset=(0.0, 0.0, 0.0)):
        self.pose = pose or MODULE.Pose3d(8.0, -2.0, 0.42, 1.2)
        self.readback_offset = readback_offset
        self.teleports = []
        self.clear_count = 0
        self._after_teleport = False

    def read_pose(self):
        if self._after_teleport:
            dx, dy, dyaw = self.readback_offset
            return MODULE.Pose3d(
                self.pose.x + dx,
                self.pose.y + dy,
                self.pose.z,
                self.pose.yaw + dyaw,
            )
        return self.pose

    def teleport_se2(self, pose, *, preserve_z):
        self.teleports.append((pose, preserve_z))
        self.pose = MODULE.Pose3d(pose.x, pose.y, preserve_z, pose.yaw)
        self._after_teleport = True

    def clear_motion(self):
        self.clear_count += 1


def test_execute_reset_preserves_z_and_clears_motion():
    backend = FakeBackend()
    requested = MODULE.Pose2d(3.0, 4.0, -0.75)

    result = MODULE.execute_reset(
        backend,
        scenario_id="S07",
        pose=requested,
    )

    assert backend.teleports == [(requested, 0.42)]
    assert backend.clear_count == 1
    assert result.requested == requested
    assert result.observed == MODULE.Pose3d(3.0, 4.0, 0.42, -0.75)
    assert result.position_error_m == pytest.approx(0.0)
    assert result.yaw_error_rad == pytest.approx(0.0)
    assert result.motion_cleared


def test_execute_reset_accepts_small_readback_error():
    backend = FakeBackend(
        readback_offset=(2.0e-5, -3.0e-5, 4.0e-5)
    )

    result = MODULE.execute_reset(
        backend,
        scenario_id="S07",
        pose=MODULE.Pose2d(1.0, 2.0, 0.3),
        position_tolerance_m=1.0e-4,
        yaw_tolerance_rad=1.0e-4,
    )

    assert result.position_error_m < 1.0e-4
    assert result.yaw_error_rad < 1.0e-4


def test_execute_reset_rejects_readback_mismatch():
    backend = FakeBackend(
        readback_offset=(0.03, 0.0, 0.0)
    )

    with pytest.raises(
        MODULE.IsaacScenarioResetError,
        match="read-back mismatch",
    ):
        MODULE.execute_reset(
            backend,
            scenario_id="S07",
            pose=MODULE.Pose2d(1.0, 2.0, 0.3),
        )


def test_execute_reset_uses_wrapped_yaw_error():
    requested = math.pi - 5.0e-5
    observed_offset = -2.0 * math.pi + 8.0e-5

    backend = FakeBackend(
        readback_offset=(0.0, 0.0, observed_offset)
    )

    result = MODULE.execute_reset(
        backend,
        scenario_id="S07",
        pose=MODULE.Pose2d(0.0, 0.0, requested),
        yaw_tolerance_rad=1.0e-4,
    )

    assert result.yaw_error_rad == pytest.approx(8.0e-5)


def test_execute_reset_accepts_small_physics_settling():
    backend = FakeBackend(
        readback_offset=(0.01, 0.0, 0.002)
    )

    result = MODULE.execute_reset(
        backend,
        scenario_id="S07",
        pose=MODULE.Pose2d(0.0, 0.0, 0.0),
    )

    assert result.observed.x == pytest.approx(0.01)
    assert result.position_error_m == pytest.approx(0.01)
    assert result.yaw_error_rad == pytest.approx(0.002)


def test_execute_reset_rejects_excessive_yaw_mismatch():
    backend = FakeBackend(
        readback_offset=(0.0, 0.0, 0.01)
    )

    with pytest.raises(
        MODULE.IsaacScenarioResetError,
        match="read-back mismatch",
    ):
        MODULE.execute_reset(
            backend,
            scenario_id="S07",
            pose=MODULE.Pose2d(0.0, 0.0, 0.0),
        )


def test_execute_reset_rejects_invalid_tolerance():
    backend = FakeBackend()

    with pytest.raises(
        MODULE.IsaacScenarioResetError,
        match="position_tolerance",
    ):
        MODULE.execute_reset(
            backend,
            scenario_id="S07",
            pose=MODULE.Pose2d(0.0, 0.0, 0.0),
            position_tolerance_m=-1.0,
        )


def test_quaternion_round_trip_for_planar_yaw():
    for yaw in (
        -math.pi + 1.0e-6,
        -1.2,
        0.0,
        2.3,
        math.pi - 1.0e-6,
    ):
        quaternion = MODULE._yaw_to_wxyz(yaw)
        recovered = MODULE._wxyz_to_yaw(quaternion)

        wrapped_error = math.atan2(
            math.sin(recovered - yaw),
            math.cos(recovered - yaw),
        )

        assert wrapped_error == pytest.approx(0.0)


def test_pose2d_from_scenario_uses_physical_pose_not_localization_prior():
    scenario = {
        "robot": {
            "initial_pose": {
                "x": 3.0,
                "y": 4.0,
                "yaw": -0.75,
            }
        },
        "localization": {
            "mode": "known_pose",
            "initial_pose": {
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            },
        },
    }

    assert MODULE.pose2d_from_scenario(scenario) == MODULE.Pose2d(
        3.0,
        4.0,
        -0.75,
    )


def test_validate_pose_match_accepts_stable_post_physics_pose():
    requested = MODULE.Pose2d(
        1.0,
        -2.0,
        math.pi - 2.0e-5,
    )

    observed = MODULE.Pose3d(
        1.00002,
        -2.00003,
        0.35,
        -math.pi + 3.0e-5,
    )

    position_error, yaw_error = MODULE.validate_pose_match(
        requested,
        observed,
        position_tolerance_m=1.0e-4,
        yaw_tolerance_rad=1.0e-4,
    )

    assert position_error < 1.0e-4
    assert yaw_error == pytest.approx(5.0e-5)


def test_validate_pose_match_rejects_pose_that_reverts_after_immediate_reset():
    requested = MODULE.Pose2d(
        0.0,
        0.0,
        0.0,
    )

    reverted = MODULE.Pose3d(
        -0.8,
        0.0,
        0.35,
        0.0,
    )

    with pytest.raises(
        MODULE.IsaacScenarioResetError,
        match="read-back mismatch",
    ):
        MODULE.validate_pose_match(
            requested,
            reverted,
            position_tolerance_m=0.02,
            yaw_tolerance_rad=0.02,
        )