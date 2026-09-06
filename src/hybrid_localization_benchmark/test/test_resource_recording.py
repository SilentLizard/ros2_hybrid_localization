from __future__ import annotations

import csv
from pathlib import Path

import pytest

from hybrid_localization_benchmark.resource_recording import (
    RESOURCE_COLUMNS,
    RESOURCE_FILENAME,
    ProcessResourceTracker,
    ResourceCsvWriter,
    find_unique_process,
    ResourceSample,
    parse_proc_stat_total_ticks,
    parse_proc_status_rss_bytes,
)
from hybrid_localization_benchmark.run_schema import OUTPUT_STREAMS


def make_run(tmp_path: Path) -> Path:
    run = tmp_path / "run"
    run.mkdir()
    (run / "run.json").write_text("{}\n", encoding="utf-8")
    return run


def sample(**overrides: object) -> ResourceSample:
    values: dict[str, object] = {
        "monotonic_time_ns": 10_000_000_000,
        "sim_time_ns": 20_000_000_000,
        "process_cpu_percent": 25.0,
        "process_rss_bytes": 64 * 1024 * 1024,
        "update_duration_ns": 2_500_000,
        "update_period_ns": 0,
    }
    values.update(overrides)
    return ResourceSample(**values)  # type: ignore[arg-type]


def test_resource_columns_match_schema_v1() -> None:
    schema_columns = next(s.columns for s in OUTPUT_STREAMS if s.filename == RESOURCE_FILENAME)
    assert RESOURCE_COLUMNS == schema_columns


def test_resource_sample_validates_ranges() -> None:
    sample().validate()
    with pytest.raises(ValueError):
        sample(process_cpu_percent=-0.1).validate()
    with pytest.raises(ValueError):
        sample(process_rss_bytes=-1).validate()
    with pytest.raises(ValueError):
        sample(update_duration_ns=-1).validate()
    with pytest.raises(ValueError):
        sample(update_period_ns=-1).validate()


def test_proc_stat_parser_handles_spaces_in_command_name() -> None:
    # pid, (comm), then fields 3..15. utime=120 and stime=30.
    text = "42 (particle analysis node) S 1 2 3 4 5 6 7 8 9 10 120 30 0 0 0"
    assert parse_proc_stat_total_ticks(text) == 150


def test_proc_stat_parser_rejects_invalid_records() -> None:
    with pytest.raises(ValueError):
        parse_proc_stat_total_ticks("not a proc stat record")
    with pytest.raises(ValueError):
        parse_proc_stat_total_ticks("42 (x) S 1 2")


def test_proc_status_parser_converts_kib_to_bytes() -> None:
    text = "Name:\ttest\nVmPeak:\t100 kB\nVmRSS:\t2048 kB\n"
    assert parse_proc_status_rss_bytes(text) == 2048 * 1024


def test_proc_status_parser_requires_vmrss() -> None:
    with pytest.raises(ValueError):
        parse_proc_status_rss_bytes("Name:\ttest\n")




def test_find_unique_process_matches_executable_not_arguments(tmp_path: Path) -> None:
    proc_root = tmp_path / "proc"
    proc_root.mkdir()

    target = proc_root / "100"
    target.mkdir()
    (target / "cmdline").write_bytes(
        b"/opt/install/lib/hybrid_localization_ros/particle_analysis_observer\0--ros-args\0"
    )

    wrapper = proc_root / "101"
    wrapper.mkdir()
    (wrapper / "cmdline").write_bytes(
        b"/usr/bin/python3\0/opt/ros/jazzy/bin/ros2\0run\0hybrid_localization_ros\0"
        b"particle_analysis_observer\0"
    )

    recorder = proc_root / "102"
    recorder.mkdir()
    (recorder / "cmdline").write_bytes(
        b"/opt/install/resource_recorder.py\0--process-match\0particle_analysis_observer\0"
    )

    assert find_unique_process("particle_analysis_observer", proc_root=proc_root) == 100

def test_process_tracker_computes_interval_cpu_percent() -> None:
    tracker = ProcessResourceTracker(clock_ticks_per_second=100)
    assert tracker.update(1_000_000_000, 200) is None
    # 25 CPU ticks = 0.25 CPU s across 0.5 wall s => 50% of one core.
    assert tracker.update(1_500_000_000, 225) == pytest.approx(50.0)


def test_process_tracker_rejects_backwards_time_or_ticks() -> None:
    tracker = ProcessResourceTracker(clock_ticks_per_second=100)
    tracker.update(1_000_000_000, 200)
    with pytest.raises(ValueError):
        tracker.update(1_000_000_000, 201)

    tracker = ProcessResourceTracker(clock_ticks_per_second=100)
    tracker.update(1_000_000_000, 200)
    with pytest.raises(ValueError):
        tracker.update(2_000_000_000, 199)


def test_writer_creates_schema_rows_and_refuses_overwrite(tmp_path: Path) -> None:
    run = make_run(tmp_path)
    with ResourceCsvWriter(run) as writer:
        writer.write(sample())
        writer.write(
            sample(
                monotonic_time_ns=11_000_000_000,
                sim_time_ns=20_250_000_000,
                update_period_ns=250_000_000,
            )
        )

    with (run / RESOURCE_FILENAME).open(newline="", encoding="utf-8") as handle:
        rows = list(csv.reader(handle))
    assert tuple(rows[0]) == RESOURCE_COLUMNS
    assert len(rows) == 3
    assert rows[1][4] == "2500000"

    with pytest.raises(FileExistsError):
        ResourceCsvWriter(run)


def test_writer_enforces_period_and_timestamp_progression(tmp_path: Path) -> None:
    run = make_run(tmp_path)
    with ResourceCsvWriter(run) as writer:
        with pytest.raises(ValueError):
            writer.write(sample(update_period_ns=1))
        writer.write(sample())
        with pytest.raises(ValueError):
            writer.write(
                sample(
                    monotonic_time_ns=11_000_000_000,
                    sim_time_ns=20_250_000_000,
                    update_period_ns=123,
                )
            )
        with pytest.raises(ValueError):
            writer.write(
                sample(
                    monotonic_time_ns=9_000_000_000,
                    sim_time_ns=20_250_000_000,
                    update_period_ns=250_000_000,
                )
            )
