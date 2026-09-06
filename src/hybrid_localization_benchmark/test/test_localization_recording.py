from __future__ import annotations

import csv
from pathlib import Path
import sys

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from hybrid_localization_benchmark.localization_recording import (  # noqa: E402
    LOCALIZATION_COLUMNS,
    LocalizationCsvWriter,
    PoseSample,
    PoseSynchronizer,
    build_localization_sample,
    wrapped_angle_difference,
)
from hybrid_localization_benchmark.run_schema import BenchmarkRun, create_run_directory  # noqa: E402


def make_run_directory(tmp_path: Path) -> Path:
    return create_run_directory(
        tmp_path,
        BenchmarkRun(
            run_id="S07-amcl-stage2",
            scenario_id="S07",
            world_scenario="S07",
            seed=1680,
            estimator="amcl",
            started_at_utc="2026-09-05T20:00:00Z",
        ),
    )


def test_computes_planar_position_and_wrapped_yaw_error() -> None:
    sample = build_localization_sample(
        PoseSample(1_000, 1.0, 2.0, 3.13),
        PoseSample(1_010, 4.0, 6.0, -3.13),
    )

    assert sample.sim_time_ns == 1_010
    assert sample.position_error_m == pytest.approx(5.0)
    assert sample.yaw_error_rad == pytest.approx(abs(wrapped_angle_difference(-3.13, 3.13)))
    assert sample.yaw_error_rad < 0.03


def test_synchronizer_uses_nearest_ground_truth_and_estimate_timestamp() -> None:
    sync = PoseSynchronizer(max_time_delta_ns=20)
    assert sync.add_ground_truth(PoseSample(90, 0.0, 0.0, 0.0)) == []
    assert sync.add_estimate(PoseSample(100, 1.0, 0.0, 0.0)) == []
    assert sync.add_ground_truth(PoseSample(108, 0.5, 0.0, 0.0)) == []
    emitted = sync.add_ground_truth(PoseSample(121, 2.0, 0.0, 0.0))

    assert len(emitted) == 1
    assert emitted[0].sim_time_ns == 100
    assert emitted[0].ground_truth_x_m == pytest.approx(0.5)
    assert emitted[0].position_error_m == pytest.approx(0.5)
    assert sync.matched_count == 1


def test_synchronizer_tie_breaks_to_earlier_ground_truth() -> None:
    sync = PoseSynchronizer(max_time_delta_ns=20)
    sync.add_ground_truth(PoseSample(90, 1.0, 0.0, 0.0))
    sync.add_estimate(PoseSample(100, 0.0, 0.0, 0.0))
    sync.add_ground_truth(PoseSample(110, 2.0, 0.0, 0.0))
    emitted = sync.add_ground_truth(PoseSample(121, 3.0, 0.0, 0.0))

    assert emitted[0].ground_truth_x_m == pytest.approx(1.0)


def test_synchronizer_drops_estimate_without_ground_truth_in_tolerance() -> None:
    sync = PoseSynchronizer(max_time_delta_ns=5)
    sync.add_ground_truth(PoseSample(90, 0.0, 0.0, 0.0))
    sync.add_estimate(PoseSample(100, 0.0, 0.0, 0.0))
    emitted = sync.add_ground_truth(PoseSample(106, 0.0, 0.0, 0.0))

    assert emitted == []
    assert sync.dropped_estimate_count == 1


def test_finalize_resolves_pending_tail_sample() -> None:
    sync = PoseSynchronizer(max_time_delta_ns=10)
    sync.add_ground_truth(PoseSample(100, 0.0, 0.0, 0.0))
    sync.add_estimate(PoseSample(104, 1.0, 0.0, 0.0))

    emitted = sync.finalize()
    assert len(emitted) == 1
    assert emitted[0].position_error_m == pytest.approx(1.0)


def test_non_monotonic_stream_timestamps_are_rejected() -> None:
    sync = PoseSynchronizer(max_time_delta_ns=10)
    sync.add_ground_truth(PoseSample(100, 0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        sync.add_ground_truth(PoseSample(100, 0.0, 0.0, 0.0))

    sync.add_estimate(PoseSample(100, 0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        sync.add_estimate(PoseSample(99, 0.0, 0.0, 0.0))


def test_invalid_pose_and_sync_configuration_are_rejected() -> None:
    with pytest.raises(ValueError):
        PoseSynchronizer(-1)
    with pytest.raises(ValueError):
        PoseSample(-1, 0.0, 0.0, 0.0).validate()
    with pytest.raises(ValueError):
        PoseSample(0, float("nan"), 0.0, 0.0).validate()


def test_csv_writer_creates_exact_schema_and_rows(tmp_path: Path) -> None:
    run_directory = make_run_directory(tmp_path)
    sample = build_localization_sample(
        PoseSample(100, 1.0, 2.0, 0.1),
        PoseSample(102, 1.3, 2.4, 0.2),
    )

    with LocalizationCsvWriter(run_directory) as writer:
        writer.append(sample)

    with (run_directory / "localization.csv").open(newline="", encoding="utf-8") as handle:
        rows = list(csv.reader(handle))
    assert tuple(rows[0]) == LOCALIZATION_COLUMNS
    assert len(rows) == 2
    assert int(rows[1][0]) == 102
    assert float(rows[1][7]) == pytest.approx(0.5)


def test_csv_writer_requires_manifest_and_never_reuses_output(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        LocalizationCsvWriter(tmp_path)

    run_directory = make_run_directory(tmp_path)
    writer = LocalizationCsvWriter(run_directory)
    writer.close()
    with pytest.raises(FileExistsError):
        LocalizationCsvWriter(run_directory)
