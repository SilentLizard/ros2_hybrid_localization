#!/usr/bin/env python3
"""Isaac/USD adapter for deterministic #44 Stage 5 environment geometry."""

from __future__ import annotations

import re

import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics


FAULT_ROOT = "/World/HybridLocalizationFaults"


def _safe_name(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_]", "_", value)
    return cleaned or "fault"


class IsaacEnvironmentFaultController:
    def __init__(self, root_path: str = FAULT_ROOT) -> None:
        self._root_path = root_path

    def _stage(self):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("No USD stage is open")
        return stage

    def clear_all(self) -> None:
        stage = self._stage()
        prim = stage.GetPrimAtPath(self._root_path)
        if prim and prim.IsValid():
            stage.RemovePrim(self._root_path)

    def remove_fault(self, fault_id: str) -> None:
        stage = self._stage()
        path = f"{self._root_path}/{_safe_name(fault_id)}"
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            stage.RemovePrim(path)

    def apply_boxes(self, fault_id: str, boxes) -> str:
        stage = self._stage()
        UsdGeom.Xform.Define(stage, self._root_path)
        fault_path = f"{self._root_path}/{_safe_name(fault_id)}"
        existing = stage.GetPrimAtPath(fault_path)
        if existing and existing.IsValid():
            stage.RemovePrim(fault_path)
        UsdGeom.Xform.Define(stage, fault_path)

        for index, box in enumerate(boxes):
            path = f"{fault_path}/Box_{index:02d}"
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.0)
            cube.CreateDisplayColorAttr([Gf.Vec3f(0.8, 0.25, 0.1)])
            xform = UsdGeom.Xformable(cube.GetPrim())
            xform.AddTranslateOp().Set(
                Gf.Vec3d(box.x_m, box.y_m, box.height_m / 2.0)
            )
            xform.AddRotateZOp().Set(box.yaw_rad * 180.0 / 3.141592653589793)
            xform.AddScaleOp().Set(
                Gf.Vec3d(box.size_x_m, box.size_y_m, box.height_m)
            )
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        return fault_path
