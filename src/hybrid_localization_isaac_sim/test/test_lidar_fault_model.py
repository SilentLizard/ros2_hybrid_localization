from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_ROOT / "scripts" / "lidar_fault_model.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("lidar_fault_model", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


model = _load_module()


def scan(*, t=1.0, ranges=(1.0, 2.0, 3.0, 4.0, 5.0), angle_min=-1.0, angle_increment=0.5):
    return model.LaserScanSample(
        sim_time_s=t,
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=0.1,
        range_max=10.0,
        ranges=tuple(ranges),
    )


def test_nominal_pass_through_is_exact():
    gateway = model.DeterministicLidarFaultModel()
    source = scan(ranges=(1.0, math.inf, math.nan, 4.0))
    output = gateway.process(source)

    assert output is source


def test_gaussian_noise_is_reproducible_from_seed():
    source1 = scan(t=1.0)
    source2 = scan(t=2.0)

    left = model.DeterministicLidarFaultModel()
    right = model.DeterministicLidarFaultModel()
    params = {"gaussian_noise_stddev_m": 0.05}
    left.activate(seed=4511, parameters=params)
    right.activate(seed=4511, parameters=params)

    assert left.process(source1).ranges == right.process(source1).ranges
    assert left.process(source2).ranges == right.process(source2).ranges


def test_different_noise_seeds_diverge():
    source = scan()
    left = model.DeterministicLidarFaultModel()
    right = model.DeterministicLidarFaultModel()
    params = {"gaussian_noise_stddev_m": 0.05}
    left.activate(seed=1, parameters=params)
    right.activate(seed=2, parameters=params)

    assert left.process(source).ranges != right.process(source).ranges


def test_beam_dropout_is_reproducible_and_uses_infinity():
    params = {"dropout_fraction": 0.5}
    left = model.DeterministicLidarFaultModel()
    right = model.DeterministicLidarFaultModel()
    left.activate(seed=4502, parameters=params)
    right.activate(seed=4502, parameters=params)

    a = left.process(scan())
    b = right.process(scan())
    assert a.ranges == b.ranges
    assert any(math.isinf(value) for value in a.ranges)
    assert any(math.isfinite(value) for value in a.ranges)


def test_angular_sector_dropout_handles_normal_interval():
    gateway = model.DeterministicLidarFaultModel()
    gateway.activate(
        seed=1,
        parameters={"sector_start_rad": -0.25, "sector_end_rad": 0.75},
    )

    output = gateway.process(scan(angle_min=-1.0, angle_increment=0.5))
    assert output.ranges[0] == 1.0
    assert output.ranges[1] == 2.0
    assert math.isinf(output.ranges[2])
    assert math.isinf(output.ranges[3])
    assert output.ranges[4] == 5.0


def test_angular_sector_dropout_handles_wraparound():
    gateway = model.DeterministicLidarFaultModel()
    gateway.activate(
        seed=1,
        parameters={"sector_start_rad": 2.8, "sector_end_rad": -2.8},
    )
    source = scan(
        ranges=(1.0, 2.0, 3.0),
        angle_min=2.9,
        angle_increment=0.3,
    )
    output = gateway.process(source)

    assert math.isinf(output.ranges[0])
    assert math.isinf(output.ranges[1])
    assert output.ranges[2] == 3.0


def test_reduced_range_updates_range_max_and_drops_far_returns():
    gateway = model.DeterministicLidarFaultModel()
    gateway.activate(seed=1, parameters={"max_range_m": 3.0})

    output = gateway.process(scan(ranges=(0.5, 3.0, 3.1, 8.0)))
    assert output.range_max == pytest.approx(3.0)
    assert output.ranges[:2] == pytest.approx((0.5, 3.0))
    assert math.isinf(output.ranges[2])
    assert math.isinf(output.ranges[3])


def test_outlier_returns_are_reproducible_and_bounded():
    gateway = model.DeterministicLidarFaultModel()
    gateway.activate(
        seed=4505,
        parameters={
            "outlier_fraction": 1.0,
            "outlier_min_m": 0.5,
            "outlier_max_m": 2.5,
        },
    )
    output = gateway.process(scan())

    assert all(0.5 <= value <= 2.5 for value in output.ranges)

    repeated = model.DeterministicLidarFaultModel()
    repeated.activate(
        seed=4505,
        parameters={
            "outlier_fraction": 1.0,
            "outlier_min_m": 0.5,
            "outlier_max_m": 2.5,
        },
    )
    assert repeated.process(scan()).ranges == output.ranges


def test_complete_scan_loss_keeps_geometry_and_returns_only_infinity():
    gateway = model.DeterministicLidarFaultModel()
    gateway.activate(seed=4506, parameters={"complete_scan_loss": True})
    source = scan()
    output = gateway.process(source)

    assert output.angle_min == source.angle_min
    assert output.angle_increment == source.angle_increment
    assert output.range_min == source.range_min
    assert output.range_max == source.range_max
    assert len(output.ranges) == len(source.ranges)
    assert all(math.isinf(value) for value in output.ranges)


def test_active_fault_canonicalizes_invalid_or_out_of_contract_source_returns():
    gateway = model.DeterministicLidarFaultModel()
    gateway.activate(seed=1, parameters={"gaussian_noise_stddev_m": 0.01})
    output = gateway.process(scan(ranges=(math.nan, math.inf, 0.01, 11.0, 2.0)))

    assert all(math.isinf(value) for value in output.ranges[:4])
    assert math.isfinite(output.ranges[4])


def test_noise_that_exits_valid_range_becomes_no_return():
    gateway = model.DeterministicLidarFaultModel()
    gateway.activate(seed=3, parameters={"gaussian_noise_stddev_m": 100.0})
    output = gateway.process(scan(ranges=(0.11, 9.99)))

    assert all(math.isinf(value) or 0.1 <= value <= 10.0 for value in output.ranges)


def test_deactivate_restores_nominal_pass_through_and_resets_time_guard():
    gateway = model.DeterministicLidarFaultModel()
    gateway.activate(seed=1, parameters={"complete_scan_loss": True})
    assert all(math.isinf(v) for v in gateway.process(scan(t=5.0)).ranges)

    gateway.deactivate()
    source = scan(t=1.0)
    assert gateway.process(source) is source


def test_active_samples_require_strictly_increasing_simulation_time():
    gateway = model.DeterministicLidarFaultModel()
    gateway.activate(seed=1, parameters={"dropout_fraction": 0.2})
    gateway.process(scan(t=2.0))

    with pytest.raises(model.LidarFaultModelError, match="strictly increasing"):
        gateway.process(scan(t=2.0))


def test_invalid_scan_metadata_and_seed_are_rejected():
    gateway = model.DeterministicLidarFaultModel()
    with pytest.raises(model.LidarFaultModelError, match="seed"):
        gateway.activate(seed=-1, parameters={"dropout_fraction": 0.2})

    bad = model.LaserScanSample(
        sim_time_s=1.0,
        angle_min=0.0,
        angle_increment=0.0,
        range_min=0.1,
        range_max=10.0,
        ranges=(1.0,),
    )
    with pytest.raises(model.LidarFaultModelError, match="angle_increment"):
        gateway.process(bad)
