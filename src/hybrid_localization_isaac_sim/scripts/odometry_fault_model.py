#!/usr/bin/env python3
"""ROS-independent deterministic odometry support for #44.

This module contains two independent pieces of odometry logic:

1. ``OdometryContinuityCompensator`` keeps localization-facing odometry
   continuous across an externally commanded physical teleport such as the
   S3 kidnapped-robot fault.  It waits for the scheduled discontinuity, detects
   the matching jump in raw Isaac odometry, and changes the raw->published
   odometry-frame transform so the teleport itself disappears while all later
   incremental motion remains visible.

2. ``DeterministicOdometryFaultModel`` applies the Stage-3 odometry degradation
   faults (scale, bias, noise, freeze) to an already-continuous odometry stream.

The physical Isaac state and simulator ground truth are never modified here.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import Any, Mapping


class OdometryFaultModelError(ValueError):
    """Raised when odometry samples or fault parameters are invalid."""


@dataclass(frozen=True)
class OdometrySample:
    sim_time_s: float
    x: float
    y: float
    yaw: float
    linear_velocity_x: float = 0.0
    linear_velocity_y: float = 0.0
    angular_velocity_z: float = 0.0


@dataclass(frozen=True)
class OdometryFaultParameters:
    linear_scale: float = 1.0
    angular_scale: float = 1.0
    linear_bias_m: float = 0.0
    angular_bias_rad: float = 0.0
    linear_noise_stddev_m: float = 0.0
    angular_noise_stddev_rad: float = 0.0
    freeze: bool = False


@dataclass
class _PoseState:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class _ScheduledDiscontinuity:
    fault_id: str
    scheduled_sim_time_s: float
    expected_dx_m: float
    expected_dy_m: float
    expected_dyaw_rad: float


@dataclass(frozen=True)
class AbsorbedDiscontinuity:
    fault_id: str
    sim_time_s: float
    raw_dx_m: float
    raw_dy_m: float
    raw_dyaw_rad: float


def normalize_yaw(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


def shortest_yaw_delta(current: float, previous: float) -> float:
    return normalize_yaw(current - previous)


def parameters_from_mapping(parameters: Mapping[str, Any]) -> OdometryFaultParameters:
    """Translate the already validated Stage-1 parameter mapping."""

    return OdometryFaultParameters(
        linear_scale=float(parameters.get("linear_scale", 1.0)),
        angular_scale=float(parameters.get("angular_scale", 1.0)),
        linear_bias_m=float(parameters.get("linear_bias_m", 0.0)),
        angular_bias_rad=float(parameters.get("angular_bias_rad", 0.0)),
        linear_noise_stddev_m=float(parameters.get("linear_noise_stddev_m", 0.0)),
        angular_noise_stddev_rad=float(parameters.get("angular_noise_stddev_rad", 0.0)),
        freeze=bool(parameters.get("freeze", False)),
    )


def _validate_sample(sample: OdometrySample) -> None:
    values = (
        sample.sim_time_s,
        sample.x,
        sample.y,
        sample.yaw,
        sample.linear_velocity_x,
        sample.linear_velocity_y,
        sample.angular_velocity_z,
    )
    if not all(math.isfinite(value) for value in values):
        raise OdometryFaultModelError("odometry sample values must be finite")
    if sample.sim_time_s < 0.0:
        raise OdometryFaultModelError("odometry sample simulation time must be >= 0")


class OdometryContinuityCompensator:
    """Hide scheduled physical teleports from localization-facing odometry.

    The compensator maintains a rigid SE(2) transform from Isaac's raw odometry
    pose into the continuous odometry pose published to localization.

    Before a kidnapping the transform is identity.  When the expected raw jump
    is observed, a new transform is chosen so that the first post-teleport raw
    sample maps exactly to the last pre-teleport published pose.  Subsequent
    raw increments are therefore preserved normally.

    Detection is intentionally tied to an explicit scheduled kidnapping.  Large
    ordinary odometry motions are never silently suppressed unless a matching
    discontinuity has been armed for that simulation time.
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._transform_x = 0.0
        self._transform_y = 0.0
        self._transform_yaw = 0.0
        self._previous_raw: OdometrySample | None = None
        self._last_output: OdometrySample | None = None
        self._pending: list[_ScheduledDiscontinuity] = []
        self._last_absorbed: AbsorbedDiscontinuity | None = None

    @property
    def pending_fault_ids(self) -> tuple[str, ...]:
        return tuple(item.fault_id for item in self._pending)

    @property
    def last_absorbed(self) -> AbsorbedDiscontinuity | None:
        return self._last_absorbed

    def arm(
        self,
        *,
        fault_id: str,
        scheduled_sim_time_s: float,
        expected_dx_m: float,
        expected_dy_m: float,
        expected_dyaw_rad: float,
    ) -> None:
        if not isinstance(fault_id, str) or not fault_id:
            raise OdometryFaultModelError("kidnap fault_id must be a non-empty string")
        values = (
            scheduled_sim_time_s,
            expected_dx_m,
            expected_dy_m,
            expected_dyaw_rad,
        )
        if not all(math.isfinite(value) for value in values):
            raise OdometryFaultModelError("kidnap discontinuity values must be finite")
        if scheduled_sim_time_s < 0.0:
            raise OdometryFaultModelError("kidnap scheduled simulation time must be >= 0")
        if math.hypot(expected_dx_m, expected_dy_m) < 1.0e-9 and abs(expected_dyaw_rad) < 1.0e-9:
            raise OdometryFaultModelError("kidnap discontinuity must contain a non-zero pose offset")

        self._pending.append(
            _ScheduledDiscontinuity(
                fault_id=fault_id,
                scheduled_sim_time_s=float(scheduled_sim_time_s),
                expected_dx_m=float(expected_dx_m),
                expected_dy_m=float(expected_dy_m),
                expected_dyaw_rad=normalize_yaw(float(expected_dyaw_rad)),
            )
        )
        self._pending.sort(key=lambda item: item.scheduled_sim_time_s)

    def process(self, raw: OdometrySample) -> OdometrySample:
        _validate_sample(raw)

        previous_raw = self._previous_raw
        last_output = self._last_output

        if previous_raw is not None and raw.sim_time_s <= previous_raw.sim_time_s:
            raise OdometryFaultModelError(
                "odometry continuity samples must have strictly increasing simulation time"
            )

        if previous_raw is not None and last_output is not None:
            pending = self._first_due_discontinuity(raw.sim_time_s)
            if pending is not None and self._matches_expected_jump(previous_raw, raw, pending):
                raw_dx = raw.x - previous_raw.x
                raw_dy = raw.y - previous_raw.y
                raw_dyaw = shortest_yaw_delta(raw.yaw, previous_raw.yaw)
                self._rebase_transform(raw, last_output)
                self._pending.remove(pending)
                self._last_absorbed = AbsorbedDiscontinuity(
                    fault_id=pending.fault_id,
                    sim_time_s=raw.sim_time_s,
                    raw_dx_m=raw_dx,
                    raw_dy_m=raw_dy,
                    raw_dyaw_rad=raw_dyaw,
                )

        output = self._apply_transform(raw)
        self._previous_raw = raw
        self._last_output = output
        return output

    def _first_due_discontinuity(self, sim_time_s: float) -> _ScheduledDiscontinuity | None:
        # Allow a small scheduler/transport skew.  We only *detect* here; no
        # compensation is applied until the raw jump itself is observed.
        early_arm_margin_s = 0.25
        for item in self._pending:
            if sim_time_s + early_arm_margin_s >= item.scheduled_sim_time_s:
                return item
        return None

    @staticmethod
    def _matches_expected_jump(
        previous: OdometrySample,
        current: OdometrySample,
        expected: _ScheduledDiscontinuity,
    ) -> bool:
        observed_dx = current.x - previous.x
        observed_dy = current.y - previous.y
        observed_dyaw = shortest_yaw_delta(current.yaw, previous.yaw)

        expected_translation = math.hypot(expected.expected_dx_m, expected.expected_dy_m)
        translation_tolerance = max(0.25, 0.20 * expected_translation)
        yaw_tolerance = max(0.15, 0.20 * abs(expected.expected_dyaw_rad))

        translation_residual = math.hypot(
            observed_dx - expected.expected_dx_m,
            observed_dy - expected.expected_dy_m,
        )
        yaw_residual = abs(shortest_yaw_delta(observed_dyaw, expected.expected_dyaw_rad))

        if expected_translation >= 1.0e-6 and translation_residual > translation_tolerance:
            return False
        if abs(expected.expected_dyaw_rad) >= 1.0e-6 and yaw_residual > yaw_tolerance:
            return False

        # For a purely translational fault, reject tiny ordinary increments even
        # if an unusually loose tolerance were configured by the magnitude rule.
        if expected_translation >= 1.0e-6:
            observed_translation = math.hypot(observed_dx, observed_dy)
            if observed_translation < max(0.10, 0.50 * expected_translation):
                return False

        # Likewise require a meaningful yaw jump for a pure rotational kidnap.
        if expected_translation < 1.0e-6 and abs(expected.expected_dyaw_rad) >= 1.0e-6:
            if abs(observed_dyaw) < max(0.05, 0.50 * abs(expected.expected_dyaw_rad)):
                return False

        return True

    def _rebase_transform(self, raw: OdometrySample, desired: OdometrySample) -> None:
        self._transform_yaw = normalize_yaw(desired.yaw - raw.yaw)
        cos_t = math.cos(self._transform_yaw)
        sin_t = math.sin(self._transform_yaw)
        rotated_x = cos_t * raw.x - sin_t * raw.y
        rotated_y = sin_t * raw.x + cos_t * raw.y
        self._transform_x = desired.x - rotated_x
        self._transform_y = desired.y - rotated_y

    def _apply_transform(self, raw: OdometrySample) -> OdometrySample:
        cos_t = math.cos(self._transform_yaw)
        sin_t = math.sin(self._transform_yaw)
        x = self._transform_x + cos_t * raw.x - sin_t * raw.y
        y = self._transform_y + sin_t * raw.x + cos_t * raw.y
        yaw = normalize_yaw(self._transform_yaw + raw.yaw)
        return OdometrySample(
            raw.sim_time_s,
            x,
            y,
            yaw,
            raw.linear_velocity_x,
            raw.linear_velocity_y,
            raw.angular_velocity_z,
        )


class DeterministicOdometryFaultModel:
    """Pass through nominal odometry or deterministically distort it while active."""

    def __init__(self) -> None:
        self._active = False
        self._parameters = OdometryFaultParameters()
        self._rng = random.Random(0)
        self._previous_raw: OdometrySample | None = None
        self._distorted_pose: _PoseState | None = None
        self._frozen_output: OdometrySample | None = None
        self._last_sim_time_s: float | None = None

    @property
    def active(self) -> bool:
        return self._active

    def activate(
        self,
        *,
        seed: int,
        parameters: Mapping[str, Any],
        anchor: OdometrySample,
    ) -> None:
        _validate_sample(anchor)
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise OdometryFaultModelError("seed must be a non-negative integer")

        self._parameters = parameters_from_mapping(parameters)
        self._rng = random.Random(seed)
        self._previous_raw = anchor
        self._distorted_pose = _PoseState(anchor.x, anchor.y, normalize_yaw(anchor.yaw))
        self._frozen_output = None
        self._last_sim_time_s = anchor.sim_time_s
        self._active = True

        if self._parameters.freeze:
            self._frozen_output = self._decorate_measurement(anchor)

    def deactivate(self) -> None:
        self._active = False
        self._previous_raw = None
        self._distorted_pose = None
        self._frozen_output = None
        self._last_sim_time_s = None

    def process(self, raw: OdometrySample) -> OdometrySample:
        _validate_sample(raw)
        if not self._active:
            return OdometrySample(
                raw.sim_time_s,
                raw.x,
                raw.y,
                normalize_yaw(raw.yaw),
                raw.linear_velocity_x,
                raw.linear_velocity_y,
                raw.angular_velocity_z,
            )

        if self._last_sim_time_s is not None and raw.sim_time_s <= self._last_sim_time_s:
            raise OdometryFaultModelError(
                "active odometry samples must have strictly increasing simulation time"
            )
        self._last_sim_time_s = raw.sim_time_s

        if self._parameters.freeze:
            assert self._frozen_output is not None
            frozen = self._frozen_output
            return OdometrySample(
                raw.sim_time_s,
                frozen.x,
                frozen.y,
                frozen.yaw,
                0.0,
                0.0,
                0.0,
            )

        previous = self._previous_raw
        pose = self._distorted_pose
        if previous is None or pose is None:
            raise OdometryFaultModelError("active odometry model is missing its anchor state")

        dx_world = raw.x - previous.x
        dy_world = raw.y - previous.y
        cos_raw = math.cos(previous.yaw)
        sin_raw = math.sin(previous.yaw)
        dx_body = cos_raw * dx_world + sin_raw * dy_world
        dy_body = -sin_raw * dx_world + cos_raw * dy_world
        dx_body *= self._parameters.linear_scale
        dy_body *= self._parameters.linear_scale

        cos_out = math.cos(pose.yaw)
        sin_out = math.sin(pose.yaw)
        pose.x += cos_out * dx_body - sin_out * dy_body
        pose.y += sin_out * dx_body + cos_out * dy_body
        pose.yaw = normalize_yaw(
            pose.yaw
            + self._parameters.angular_scale
            * shortest_yaw_delta(raw.yaw, previous.yaw)
        )
        self._previous_raw = raw

        latent = OdometrySample(
            raw.sim_time_s,
            pose.x,
            pose.y,
            pose.yaw,
            raw.linear_velocity_x * self._parameters.linear_scale,
            raw.linear_velocity_y * self._parameters.linear_scale,
            raw.angular_velocity_z * self._parameters.angular_scale,
        )
        return self._decorate_measurement(latent)

    def _decorate_measurement(self, latent: OdometrySample) -> OdometrySample:
        p = self._parameters
        yaw = normalize_yaw(latent.yaw + p.angular_bias_rad)

        x = latent.x + math.cos(latent.yaw) * p.linear_bias_m
        y = latent.y + math.sin(latent.yaw) * p.linear_bias_m

        if p.linear_noise_stddev_m > 0.0:
            x += self._rng.gauss(0.0, p.linear_noise_stddev_m)
            y += self._rng.gauss(0.0, p.linear_noise_stddev_m)
        if p.angular_noise_stddev_rad > 0.0:
            yaw = normalize_yaw(yaw + self._rng.gauss(0.0, p.angular_noise_stddev_rad))

        return OdometrySample(
            latent.sim_time_s,
            x,
            y,
            yaw,
            latent.linear_velocity_x,
            latent.linear_velocity_y,
            latent.angular_velocity_z,
        )
