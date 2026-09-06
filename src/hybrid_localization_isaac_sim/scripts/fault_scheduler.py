#!/usr/bin/env python3
"""ROS-independent simulation-time scheduler for deterministic localization faults.

Issue #44 Stage 2 uses this scheduler to anchor fault ``start_time_s`` values to
one runtime scenario activation.  The scheduler owns only timing/state; concrete
fault execution remains in the appropriate Isaac/ROS injector.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Iterable


class FaultSchedulerError(ValueError):
    """Raised when simulation-time scheduling invariants are violated."""


class FaultState(str, Enum):
    SCHEDULED = "scheduled"
    ACTIVE = "active"
    COMPLETED = "completed"
    ERROR = "error"


@dataclass
class ScheduledFault:
    specification: object
    state: FaultState = FaultState.SCHEDULED
    activation_sim_time_s: float | None = None
    completion_sim_time_s: float | None = None
    error_detail: str | None = None


class DeterministicFaultScheduler:
    """Schedule immutable fault specifications against scenario-relative sim time."""

    def __init__(self) -> None:
        self._scenario_id: str | None = None
        self._scenario_activation_sim_time_s: float | None = None
        self._last_sim_time_s: float | None = None
        self._faults: list[ScheduledFault] = []

    @property
    def scenario_id(self) -> str | None:
        return self._scenario_id

    @property
    def scenario_activation_sim_time_s(self) -> float | None:
        return self._scenario_activation_sim_time_s

    @property
    def faults(self) -> tuple[ScheduledFault, ...]:
        return tuple(self._faults)

    def configure(
        self,
        *,
        scenario_id: str,
        faults: Iterable[object],
        activation_sim_time_s: float,
    ) -> tuple[ScheduledFault, ...]:
        if not isinstance(scenario_id, str) or not scenario_id:
            raise FaultSchedulerError("scenario_id must be a non-empty string")
        if not math.isfinite(activation_sim_time_s) or activation_sim_time_s < 0.0:
            raise FaultSchedulerError("activation_sim_time_s must be finite and >= 0")

        fault_list = list(faults)
        # Stable ordering makes simultaneous triggers deterministic independent of
        # the source JSON list order after the specification has been validated.
        fault_list.sort(key=lambda fault: (float(fault.start_time_s), str(fault.fault_id)))

        self._scenario_id = scenario_id
        self._scenario_activation_sim_time_s = float(activation_sim_time_s)
        self._last_sim_time_s = float(activation_sim_time_s)
        self._faults = [ScheduledFault(fault) for fault in fault_list]
        return self.faults

    def clear(self) -> None:
        self._scenario_id = None
        self._scenario_activation_sim_time_s = None
        self._last_sim_time_s = None
        self._faults.clear()

    def elapsed_time_s(self, sim_time_s: float) -> float:
        if self._scenario_activation_sim_time_s is None:
            raise FaultSchedulerError("fault scheduler is not configured")
        if not math.isfinite(sim_time_s) or sim_time_s < 0.0:
            raise FaultSchedulerError("sim_time_s must be finite and >= 0")
        if self._last_sim_time_s is not None and sim_time_s < self._last_sim_time_s - 1.0e-12:
            raise FaultSchedulerError("simulation time moved backwards after scenario activation")
        self._last_sim_time_s = float(sim_time_s)
        return max(0.0, float(sim_time_s) - self._scenario_activation_sim_time_s)

    def activate_due(self, sim_time_s: float) -> tuple[ScheduledFault, ...]:
        elapsed = self.elapsed_time_s(sim_time_s)
        activated: list[ScheduledFault] = []
        for fault in self._faults:
            if fault.state is not FaultState.SCHEDULED:
                continue
            if elapsed + 1.0e-12 < float(fault.specification.start_time_s):
                continue
            fault.state = FaultState.ACTIVE
            fault.activation_sim_time_s = float(sim_time_s)
            activated.append(fault)
        return tuple(activated)

    def mark_completed(self, fault_id: str, *, sim_time_s: float) -> ScheduledFault:
        fault = self._find_active(fault_id)
        self.elapsed_time_s(sim_time_s)
        fault.state = FaultState.COMPLETED
        fault.completion_sim_time_s = float(sim_time_s)
        return fault

    def mark_error(self, fault_id: str, *, sim_time_s: float, detail: str) -> ScheduledFault:
        fault = self._find_active(fault_id)
        self.elapsed_time_s(sim_time_s)
        fault.state = FaultState.ERROR
        fault.completion_sim_time_s = float(sim_time_s)
        fault.error_detail = str(detail)
        return fault

    def _find_active(self, fault_id: str) -> ScheduledFault:
        for fault in self._faults:
            if fault.specification.fault_id == fault_id:
                if fault.state is not FaultState.ACTIVE:
                    raise FaultSchedulerError(
                        f"fault {fault_id!r} must be active before completion/error"
                    )
                return fault
        raise FaultSchedulerError(f"unknown scheduled fault {fault_id!r}")
