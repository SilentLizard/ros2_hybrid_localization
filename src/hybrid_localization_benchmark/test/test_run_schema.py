from __future__ import annotations

import json
from pathlib import Path
import sys

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from hybrid_localization_benchmark.run_schema import (  # noqa: E402
    BENCHMARK_SCHEMA_VERSION,
    OUTPUT_STREAMS,
    BenchmarkRun,
    create_run_directory,
)


def valid_run(**overrides: object) -> BenchmarkRun:
    values: dict[str, object] = {
        "run_id": "S07-amcl-seed1680-run01",
        "scenario_id": "S07",
        "world_scenario": "S07",
        "seed": 1680,
        "estimator": "amcl",
        "started_at_utc": "2026-09-05T20:00:00Z",
        "git_commit": "0123456789abcdef",
    }
    values.update(overrides)
    return BenchmarkRun(**values)  # type: ignore[arg-type]


def test_manifest_has_stable_stage_files_and_clock_contract() -> None:
    manifest = valid_run().to_manifest()

    assert manifest["schema_version"] == BENCHMARK_SCHEMA_VERSION
    assert [stream["filename"] for stream in manifest["streams"]] == [
        "localization.csv",
        "belief_runtime.csv",
        "resources.csv",
    ]
    assert [stream["stage"] for stream in manifest["streams"]] == [2, 3, 4]
    assert manifest["summary"]["filename"] == "summary.json"
    assert manifest["clock_contract"]["simulation_timestamp_unit"] == "nanoseconds"


def test_stream_columns_have_unique_names_and_time_key() -> None:
    for stream in OUTPUT_STREAMS:
        assert len(stream.columns) == len(set(stream.columns))
        assert "sim_time_ns" in stream.columns


def test_create_run_directory_writes_manifest_only(tmp_path: Path) -> None:
    run = valid_run()
    run_directory = create_run_directory(tmp_path, run)

    assert sorted(path.name for path in run_directory.iterdir()) == ["run.json"]
    manifest = json.loads((run_directory / "run.json").read_text(encoding="utf-8"))
    assert manifest["run"]["scenario_id"] == "S07"
    assert manifest["run"]["seed"] == 1680


def test_existing_run_directory_is_never_reused(tmp_path: Path) -> None:
    run = valid_run()
    create_run_directory(tmp_path, run)

    with pytest.raises(FileExistsError):
        create_run_directory(tmp_path, run)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("run_id", "bad run id"),
        ("scenario_id", "7"),
        ("world_scenario", "S7"),
        ("seed", -1),
        ("estimator", ""),
        ("started_at_utc", "2026-09-05T20:00:00"),
        ("git_commit", ""),
    ],
)
def test_invalid_run_metadata_is_rejected(field: str, value: object) -> None:
    with pytest.raises(ValueError):
        valid_run(**{field: value}).validate()
