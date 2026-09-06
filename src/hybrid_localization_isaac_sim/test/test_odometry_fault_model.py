import importlib.util
import math
from pathlib import Path
import sys

import pytest


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "odometry_fault_model.py"
SPEC = importlib.util.spec_from_file_location("odometry_fault_model_under_test", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

Sample = MODULE.OdometrySample
Model = MODULE.DeterministicOdometryFaultModel
Continuity = MODULE.OdometryContinuityCompensator


def sample(t, x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0, wz=0.0):
    return Sample(t, x, y, yaw, vx, vy, wz)


def test_nominal_continuity_is_exact_pass_through():
    continuity = Continuity()
    values = [
        sample(1.0, 2.0, -1.0, 0.3, 0.5, -0.2, 0.1),
        sample(2.0, 2.5, -0.8, 0.4, 0.6, -0.1, 0.2),
    ]
    assert [continuity.process(value) for value in values] == values


def test_kidnap_translation_is_absorbed_and_later_motion_is_preserved():
    continuity = Continuity()
    continuity.arm(
        fault_id="kidnap_1",
        scheduled_sim_time_s=10.0,
        expected_dx_m=2.0,
        expected_dy_m=1.0,
        expected_dyaw_rad=0.0,
    )

    before = continuity.process(sample(9.90, 1.0, 2.0, 0.2))
    ordinary = continuity.process(sample(9.95, 1.02, 2.01, 0.2))
    jumped = continuity.process(sample(10.01, 3.02, 3.01, 0.2))
    moved = continuity.process(sample(10.10, 3.12, 3.01, 0.2))

    assert before.x == pytest.approx(1.0)
    assert ordinary.x == pytest.approx(1.02)
    assert ordinary.y == pytest.approx(2.01)

    # The physical +2,+1 discontinuity itself is hidden.
    assert jumped.x == pytest.approx(ordinary.x)
    assert jumped.y == pytest.approx(ordinary.y)
    assert jumped.yaw == pytest.approx(ordinary.yaw)

    # Later raw motion continues from the old odometry belief.
    assert moved.x == pytest.approx(ordinary.x + 0.10)
    assert moved.y == pytest.approx(ordinary.y)

    absorbed = continuity.last_absorbed
    assert absorbed is not None
    assert absorbed.fault_id == "kidnap_1"
    assert absorbed.raw_dx_m == pytest.approx(2.0)
    assert absorbed.raw_dy_m == pytest.approx(1.0)
    assert continuity.pending_fault_ids == ()


def test_kidnap_is_not_applied_early_without_matching_raw_jump():
    continuity = Continuity()
    continuity.arm(
        fault_id="kidnap_1",
        scheduled_sim_time_s=10.0,
        expected_dx_m=2.0,
        expected_dy_m=1.0,
        expected_dyaw_rad=0.0,
    )

    assert continuity.process(sample(9.70, 0.0, 0.0)).x == pytest.approx(0.0)
    assert continuity.process(sample(9.80, 0.05, 0.0)).x == pytest.approx(0.05)
    assert continuity.process(sample(9.95, 0.10, 0.0)).x == pytest.approx(0.10)
    assert continuity.last_absorbed is None
    assert continuity.pending_fault_ids == ("kidnap_1",)


def test_unexpected_large_motion_is_not_suppressed():
    continuity = Continuity()
    continuity.arm(
        fault_id="kidnap_1",
        scheduled_sim_time_s=10.0,
        expected_dx_m=2.0,
        expected_dy_m=1.0,
        expected_dyaw_rad=0.0,
    )
    continuity.process(sample(9.9, 0.0, 0.0))
    result = continuity.process(sample(10.0, 0.0, 2.0))
    assert result.x == pytest.approx(0.0)
    assert result.y == pytest.approx(2.0)
    assert continuity.last_absorbed is None


def test_rotational_kidnap_is_absorbed_and_future_motion_uses_continuous_frame():
    continuity = Continuity()
    continuity.arm(
        fault_id="kidnap_turn",
        scheduled_sim_time_s=5.0,
        expected_dx_m=0.0,
        expected_dy_m=0.0,
        expected_dyaw_rad=math.pi / 2.0,
    )

    before = continuity.process(sample(4.9, 1.0, 2.0, 0.0))
    jumped = continuity.process(sample(5.0, 1.0, 2.0, math.pi / 2.0))
    moved = continuity.process(sample(5.1, 1.0, 3.0, math.pi / 2.0))

    assert jumped.x == pytest.approx(before.x)
    assert jumped.y == pytest.approx(before.y)
    assert jumped.yaw == pytest.approx(before.yaw)

    # Raw +Y after the physical 90-degree teleport corresponds to +X in the
    # retained continuous odometry frame.
    assert moved.x == pytest.approx(before.x + 1.0)
    assert moved.y == pytest.approx(before.y)
    assert moved.yaw == pytest.approx(before.yaw)


def test_continuity_reset_removes_old_transform_and_pending_faults():
    continuity = Continuity()
    continuity.arm(
        fault_id="kidnap_1",
        scheduled_sim_time_s=1.0,
        expected_dx_m=2.0,
        expected_dy_m=1.0,
        expected_dyaw_rad=0.0,
    )
    continuity.process(sample(0.9, 0.0, 0.0))
    continuity.process(sample(1.0, 2.0, 1.0))
    continuity.reset()

    raw = sample(2.0, 8.0, -3.0, 0.4)
    assert continuity.process(raw) == raw
    assert continuity.pending_fault_ids == ()
    assert continuity.last_absorbed is None


def test_continuity_rejects_non_monotonic_samples():
    continuity = Continuity()
    continuity.process(sample(1.0))
    with pytest.raises(MODULE.OdometryFaultModelError, match="strictly increasing"):
        continuity.process(sample(1.0))


def test_nominal_model_is_exact_pass_through():
    model = Model()
    result = model.process(sample(1.0, 2.0, -1.0, 0.3, 0.5, -0.2, 0.1))
    assert result == sample(1.0, 2.0, -1.0, 0.3, 0.5, -0.2, 0.1)


def test_linear_scale_accumulates_incremental_drift():
    model = Model()
    model.activate(seed=1, parameters={"linear_scale": 1.1}, anchor=sample(0.0))
    first = model.process(sample(1.0, 1.0, 0.0, 0.0, vx=1.0))
    second = model.process(sample(2.0, 2.0, 0.0, 0.0, vx=1.0))
    assert first.x == pytest.approx(1.1)
    assert second.x == pytest.approx(2.2)
    assert second.linear_velocity_x == pytest.approx(1.1)


def test_angular_scale_changes_heading_and_subsequent_translation():
    model = Model()
    model.activate(seed=1, parameters={"angular_scale": 2.0}, anchor=sample(0.0))
    turned = model.process(sample(1.0, 0.0, 0.0, math.pi / 4.0))
    moved = model.process(sample(2.0, math.sqrt(0.5), math.sqrt(0.5), math.pi / 4.0))
    assert turned.yaw == pytest.approx(math.pi / 2.0)
    assert moved.x == pytest.approx(0.0, abs=1.0e-12)
    assert moved.y == pytest.approx(1.0)


def test_bias_is_measurement_offset_not_recursive_drift():
    model = Model()
    model.activate(
        seed=1,
        parameters={"linear_bias_m": 0.2, "angular_bias_rad": 0.1},
        anchor=sample(0.0),
    )
    first = model.process(sample(1.0, 1.0, 0.0, 0.0))
    second = model.process(sample(2.0, 2.0, 0.0, 0.0))
    assert first.x == pytest.approx(1.2)
    assert second.x == pytest.approx(2.2)
    assert second.yaw == pytest.approx(0.1)


def test_noise_is_deterministic_for_same_seed_and_sample_sequence():
    parameters = {"linear_noise_stddev_m": 0.1, "angular_noise_stddev_rad": 0.05}
    left = Model()
    right = Model()
    anchor = sample(0.0)
    left.activate(seed=1234, parameters=parameters, anchor=anchor)
    right.activate(seed=1234, parameters=parameters, anchor=anchor)
    left_values = [left.process(sample(float(i), float(i), 0.0, 0.0)) for i in range(1, 5)]
    right_values = [right.process(sample(float(i), float(i), 0.0, 0.0)) for i in range(1, 5)]
    assert left_values == right_values
    assert any(value.y != 0.0 for value in left_values)


def test_different_seeds_produce_different_noise():
    parameters = {"linear_noise_stddev_m": 0.1}
    left = Model()
    right = Model()
    left.activate(seed=1, parameters=parameters, anchor=sample(0.0))
    right.activate(seed=2, parameters=parameters, anchor=sample(0.0))
    assert left.process(sample(1.0, 1.0)) != right.process(sample(1.0, 1.0))


def test_freeze_holds_activation_measurement_and_zeroes_twist():
    model = Model()
    model.activate(seed=1, parameters={"freeze": True}, anchor=sample(5.0, 3.0, 4.0, 0.5))
    result = model.process(sample(6.0, 10.0, 20.0, 1.0, 2.0, 3.0, 4.0))
    assert result.x == pytest.approx(3.0)
    assert result.y == pytest.approx(4.0)
    assert result.yaw == pytest.approx(0.5)
    assert result.linear_velocity_x == 0.0
    assert result.linear_velocity_y == 0.0
    assert result.angular_velocity_z == 0.0


def test_deactivate_restores_nominal_pass_through_without_retaining_drift():
    model = Model()
    model.activate(seed=1, parameters={"linear_scale": 2.0}, anchor=sample(0.0))
    assert model.process(sample(1.0, 1.0)).x == pytest.approx(2.0)
    model.deactivate()
    assert model.process(sample(2.0, 2.0)).x == pytest.approx(2.0)


def test_yaw_wraparound_uses_shortest_increment():
    model = Model()
    anchor = sample(0.0, yaw=math.pi - 0.1)
    model.activate(seed=1, parameters={}, anchor=anchor)
    result = model.process(sample(1.0, yaw=-math.pi + 0.1))
    assert result.yaw == pytest.approx(-math.pi + 0.1)


def test_invalid_or_non_monotonic_samples_are_rejected_while_active():
    model = Model()
    model.activate(seed=1, parameters={}, anchor=sample(1.0))
    with pytest.raises(MODULE.OdometryFaultModelError, match="strictly increasing"):
        model.process(sample(1.0))
    model.deactivate()
    with pytest.raises(MODULE.OdometryFaultModelError, match="finite"):
        model.process(sample(2.0, x=math.nan))
