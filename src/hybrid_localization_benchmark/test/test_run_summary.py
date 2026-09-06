from __future__ import annotations

import csv
import json
from pathlib import Path

import pytest

from hybrid_localization_benchmark.run_summary import (
    AcceptanceCriteria,
    build_run_summary,
    write_run_summary,
)
from hybrid_localization_benchmark.run_schema import BENCHMARK_SCHEMA_VERSION, OUTPUT_STREAMS


def _write_csv(path: Path, columns: tuple[str, ...], rows: list[tuple[object, ...]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(columns)
        writer.writerows(rows)


def make_complete_run(tmp_path: Path) -> Path:
    run = tmp_path / "run"
    run.mkdir()
    (run / "run.json").write_text(
        json.dumps({
            "schema_version": BENCHMARK_SCHEMA_VERSION,
            "run": {"run_id": "synthetic", "scenario_id": "S07", "estimator": "amcl"},
        }) + "\n",
        encoding="utf-8",
    )
    streams = {stream.filename: stream.columns for stream in OUTPUT_STREAMS}
    _write_csv(
        run / "localization.csv",
        streams["localization.csv"],
        [
            (100, 0, 0, 0, 0.1, 0, 0.01, 0.1, 0.01),
            (200, 0, 0, 0, 0.2, 0, 0.02, 0.2, 0.02),
            (300, 0, 0, 0, 0.3, 0, 0.03, 0.3, 0.03),
        ],
    )
    _write_csv(
        run / "belief_runtime.csv",
        streams["belief_runtime.csv"],
        [
            (100, 100, 80, 2, 0.9, 0.1, 0.4, 0.6, 1),
            (200, 120, 90, 1, 0.95, 0.05, 0.2, 0.8, 2),
        ],
    )
    _write_csv(
        run / "resources.csv",
        streams["resources.csv"],
        [
            (1000, 100, 20.0, 1_000_000, 1_000_000, 0),
            (2000, 200, 40.0, 1_200_000, 2_000_000, 100),
            (3000, 300, 60.0, 1_100_000, 3_000_000, 100),
        ],
    )
    return run


def test_summary_aggregates_all_three_streams(tmp_path: Path) -> None:
    summary = build_run_summary(make_complete_run(tmp_path))
    loc = summary["metrics"]["localization"]
    belief = summary["metrics"]["belief_runtime"]
    resources = summary["metrics"]["resources"]
    assert loc["sample_count"] == 3
    assert loc["position_error_mean_m"] == pytest.approx(0.2)
    assert loc["position_error_p95_m"] == pytest.approx(0.3)
    assert belief["gmm_component_count_max"] == 2
    assert belief["effective_sample_size_ratio_mean"] == pytest.approx((0.8 + 0.75) / 2)
    assert resources["process_cpu_percent_mean"] == pytest.approx(40.0)
    assert resources["process_rss_bytes_max"] == 1_200_000
    assert resources["update_duration_ns_p95"] == pytest.approx(3_000_000)


def test_default_acceptance_requires_nonempty_streams_and_passes_complete_run(tmp_path: Path) -> None:
    summary = build_run_summary(make_complete_run(tmp_path))
    acceptance = summary["acceptance"]
    assert acceptance["passed"] is True
    assert len(acceptance["checks"]) == 3


def test_quantitative_acceptance_can_pass(tmp_path: Path) -> None:
    criteria = AcceptanceCriteria(
        max_position_rmse_m=0.25,
        max_position_error_m=0.3,
        max_yaw_rmse_rad=0.03,
        max_mean_cpu_percent=40.0,
        max_peak_rss_bytes=1_200_000,
        max_update_duration_ns=3_000_000,
    )
    assert build_run_summary(make_complete_run(tmp_path), criteria)["acceptance"]["passed"] is True


def test_quantitative_acceptance_reports_individual_failure(tmp_path: Path) -> None:
    criteria = AcceptanceCriteria(max_position_error_m=0.2)
    acceptance = build_run_summary(make_complete_run(tmp_path), criteria)["acceptance"]
    assert acceptance["passed"] is False
    failed = [check for check in acceptance["checks"] if not check["passed"]]
    assert [check["name"] for check in failed] == ["position_error_max_m"]
    assert failed[0]["actual"] == pytest.approx(0.3)


def test_criteria_validation_rejects_invalid_limits() -> None:
    with pytest.raises(ValueError):
        AcceptanceCriteria(min_localization_samples=0).validate()
    with pytest.raises(ValueError):
        AcceptanceCriteria(max_position_rmse_m=-1.0).validate()
    with pytest.raises(ValueError):
        AcceptanceCriteria(max_peak_rss_bytes=-1).validate()


def test_summary_rejects_missing_stream(tmp_path: Path) -> None:
    run = make_complete_run(tmp_path)
    (run / "resources.csv").unlink()
    with pytest.raises(FileNotFoundError):
        build_run_summary(run)


def test_summary_rejects_wrong_csv_header(tmp_path: Path) -> None:
    run = make_complete_run(tmp_path)
    (run / "belief_runtime.csv").write_text("bad,column\n1,2\n", encoding="utf-8")
    with pytest.raises(ValueError):
        build_run_summary(run)


def test_summary_rejects_wrong_manifest_schema(tmp_path: Path) -> None:
    run = make_complete_run(tmp_path)
    (run / "run.json").write_text(json.dumps({"schema_version": 999}) + "\n", encoding="utf-8")
    with pytest.raises(ValueError):
        build_run_summary(run)


def test_writer_creates_summary_once(tmp_path: Path) -> None:
    run = make_complete_run(tmp_path)
    path = write_run_summary(run)
    assert path.name == "summary.json"
    parsed = json.loads(path.read_text(encoding="utf-8"))
    assert parsed["schema_version"] == BENCHMARK_SCHEMA_VERSION
    assert parsed["acceptance"]["passed"] is True
    with pytest.raises(FileExistsError):
        write_run_summary(run)


def test_percentile_behavior_is_deterministic_for_small_fixture(tmp_path: Path) -> None:
    summary = build_run_summary(make_complete_run(tmp_path))
    assert summary["metrics"]["localization"]["yaw_error_p95_rad"] == pytest.approx(0.03)
