from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "environment_fault_geometry", ROOT / "scripts" / "environment_fault_geometry.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_partial_map_mismatch_adds_one_box_in_front_of_robot():
    boxes = MODULE.map_mismatch_geometry("partial", "added_box", (1.0, 2.0, 0.0))
    assert len(boxes) == 1
    assert boxes[0].x_m == pytest.approx(3.5)
    assert boxes[0].y_m == pytest.approx(2.0)
    assert boxes[0].size_x_m == pytest.approx(0.8)
    assert boxes[0].size_y_m == pytest.approx(0.8)


def test_geometry_rotates_with_anchor_pose():
    boxes = MODULE.map_mismatch_geometry(
        "partial", "added_box", (1.0, 2.0, math.pi / 2.0)
    )
    assert boxes[0].x_m == pytest.approx(1.0)
    assert boxes[0].y_m == pytest.approx(4.5)
    assert boxes[0].yaw_rad == pytest.approx(math.pi / 2.0)


def test_severe_map_mismatch_is_materially_larger_than_partial():
    partial = MODULE.map_mismatch_geometry("partial", "added_box", (0.0, 0.0, 0.0))
    severe = MODULE.map_mismatch_geometry("severe", "added_wall", (0.0, 0.0, 0.0))
    assert len(partial) == 1
    assert len(severe) == 3
    assert severe[0].size_y_m == pytest.approx(5.0)
    assert severe[0].height_m > partial[0].height_m


def test_dynamic_front_box_is_close_and_tall_enough_to_occlude_scan():
    boxes = MODULE.dynamic_obstruction_geometry("front_box", (0.0, 0.0, 0.0))
    assert len(boxes) == 1
    assert boxes[0].x_m == pytest.approx(1.8)
    assert boxes[0].y_m == pytest.approx(0.0)
    assert boxes[0].height_m == pytest.approx(2.0)


def test_rejects_unknown_profiles():
    with pytest.raises(MODULE.EnvironmentFaultGeometryError):
        MODULE.map_mismatch_geometry("partial", "unknown", (0.0, 0.0, 0.0))
    with pytest.raises(MODULE.EnvironmentFaultGeometryError):
        MODULE.map_mismatch_geometry("severe", "added_box", (0.0, 0.0, 0.0))
    with pytest.raises(MODULE.EnvironmentFaultGeometryError):
        MODULE.dynamic_obstruction_geometry("person", (0.0, 0.0, 0.0))


def test_rejects_nonfinite_or_noncanonical_anchor_pose():
    with pytest.raises(MODULE.EnvironmentFaultGeometryError):
        MODULE.dynamic_obstruction_geometry("front_box", (math.nan, 0.0, 0.0))
    with pytest.raises(MODULE.EnvironmentFaultGeometryError):
        MODULE.dynamic_obstruction_geometry("front_box", (0.0, 0.0, math.pi))
