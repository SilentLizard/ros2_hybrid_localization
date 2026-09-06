from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def _load(name: str, filename: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / filename)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


faults = _load("stage2_fault_specification", "fault_specification.py")
scheduler_module = _load("stage2_fault_scheduler", "fault_scheduler.py")


def _kidnap(fault_id="kidnap", start_time_s=10.0):
    return faults.parse_fault(
        {
            "id": fault_id,
            "type": "kidnapped_robot",
            "start_time_s": start_time_s,
            "seed": 1777,
            "parameters": {
                "x_offset_m": 2.0,
                "y_offset_m": 1.0,
                "yaw_offset_rad": 0.0,
            },
        }
    )


def test_start_time_is_relative_to_scenario_activation_sim_time():
    scheduler = scheduler_module.DeterministicFaultScheduler()
    scheduler.configure(scenario_id="S3_KIDNAPPED", faults=[_kidnap()], activation_sim_time_s=42.0)

    assert scheduler.activate_due(51.999) == ()
    activated = scheduler.activate_due(52.0)

    assert len(activated) == 1
    assert activated[0].state is scheduler_module.FaultState.ACTIVE
    assert activated[0].activation_sim_time_s == pytest.approx(52.0)


def test_simultaneous_faults_activate_in_start_then_id_order():
    scheduler = scheduler_module.DeterministicFaultScheduler()
    scheduler.configure(
        scenario_id="test",
        faults=[_kidnap("b", 5.0), _kidnap("a", 5.0)],
        activation_sim_time_s=100.0,
    )

    activated = scheduler.activate_due(105.0)
    assert [item.specification.fault_id for item in activated] == ["a", "b"]


def test_completed_fault_does_not_reactivate():
    scheduler = scheduler_module.DeterministicFaultScheduler()
    scheduler.configure(scenario_id="test", faults=[_kidnap(start_time_s=0.0)], activation_sim_time_s=7.0)

    activated = scheduler.activate_due(7.0)
    scheduler.mark_completed("kidnap", sim_time_s=7.1)

    assert activated[0].state is scheduler_module.FaultState.COMPLETED
    assert scheduler.activate_due(8.0) == ()


def test_reconfigure_clears_previous_fault_state():
    scheduler = scheduler_module.DeterministicFaultScheduler()
    scheduler.configure(scenario_id="faulted", faults=[_kidnap( start_time_s=0.0)], activation_sim_time_s=1.0)
    scheduler.activate_due(1.0)

    scheduler.configure(scenario_id="baseline", faults=[], activation_sim_time_s=2.0)

    assert scheduler.scenario_id == "baseline"
    assert scheduler.faults == ()


def test_error_state_records_completion_time_and_detail():
    scheduler = scheduler_module.DeterministicFaultScheduler()
    scheduler.configure(scenario_id="test", faults=[_kidnap(start_time_s=0.0)], activation_sim_time_s=4.0)
    scheduler.activate_due(4.0)

    state = scheduler.mark_error("kidnap", sim_time_s=4.25, detail="teleport failed")

    assert state.state is scheduler_module.FaultState.ERROR
    assert state.completion_sim_time_s == pytest.approx(4.25)
    assert state.error_detail == "teleport failed"


def test_backwards_simulation_time_is_rejected():
    scheduler = scheduler_module.DeterministicFaultScheduler()
    scheduler.configure(scenario_id="test", faults=[_kidnap()], activation_sim_time_s=10.0)
    scheduler.activate_due(12.0)

    with pytest.raises(scheduler_module.FaultSchedulerError, match="moved backwards"):
        scheduler.activate_due(11.0)
