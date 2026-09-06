import csv
import json
import math
from pathlib import Path

import pytest

from hybrid_localization_benchmark.belief_runtime_recording import (
    BELIEF_RUNTIME_COLUMNS,
    BELIEF_RUNTIME_FILENAME,
    BeliefRuntimeCsvWriter,
    BeliefRuntimeSample,
)


def valid_sample(**overrides):
    values = dict(
        sim_time_ns=1_000_000_000,
        particle_count=500,
        effective_sample_size=250.0,
        gmm_component_count=2,
        gmm_represented_weight=0.9,
        gmm_discarded_weight=0.1,
        gmm_entropy=0.6,
        dominant_component_weight=0.55,
        analysis_sequence=1,
    )
    values.update(overrides)
    return BeliefRuntimeSample(**values)


def make_run_directory(tmp_path: Path) -> Path:
    run_directory = tmp_path / "run"
    run_directory.mkdir()
    (run_directory / "run.json").write_text(json.dumps({"schema_version": 1}))
    return run_directory


def test_schema_matches_stage1_contract():
    assert BELIEF_RUNTIME_FILENAME == "belief_runtime.csv"
    assert BELIEF_RUNTIME_COLUMNS == (
        "sim_time_ns",
        "particle_count",
        "effective_sample_size",
        "gmm_component_count",
        "gmm_represented_weight",
        "gmm_discarded_weight",
        "gmm_entropy",
        "dominant_component_weight",
        "analysis_sequence",
    )


def test_validates_consistent_particle_and_gmm_metrics():
    valid_sample().validate()
    valid_sample(
        gmm_component_count=0,
        gmm_represented_weight=0.0,
        gmm_discarded_weight=1.0,
        gmm_entropy=0.0,
        dominant_component_weight=0.0,
    ).validate()


def test_rejects_invalid_counts_sequences_and_ess():
    for sample in (
        valid_sample(particle_count=0),
        valid_sample(gmm_component_count=-1),
        valid_sample(analysis_sequence=0),
        valid_sample(effective_sample_size=0.0),
        valid_sample(effective_sample_size=501.0),
    ):
        with pytest.raises(ValueError):
            sample.validate()


def test_accepts_and_clamps_roundoff_scale_ess_overshoot():
    sample = valid_sample(particle_count=709, effective_sample_size=709.0000000000036)
    sample.validate()
    assert sample.to_row()[2] == 709.0


def test_rejects_nonfinite_or_out_of_range_probabilities():
    for sample in (
        valid_sample(gmm_entropy=math.nan),
        valid_sample(gmm_entropy=1.01),
        valid_sample(gmm_represented_weight=-0.01),
        valid_sample(dominant_component_weight=1.01),
    ):
        with pytest.raises(ValueError):
            sample.validate()


def test_rejects_inconsistent_gmm_mass_and_dominant_weight():
    with pytest.raises(ValueError):
        valid_sample(gmm_represented_weight=0.8, gmm_discarded_weight=0.1).validate()
    with pytest.raises(ValueError):
        valid_sample(dominant_component_weight=0.95).validate()
    with pytest.raises(ValueError):
        valid_sample(
            gmm_component_count=0,
            gmm_represented_weight=0.1,
            gmm_discarded_weight=0.9,
            dominant_component_weight=0.0,
        ).validate()


def test_writer_requires_existing_manifest(tmp_path):
    with pytest.raises(FileNotFoundError):
        BeliefRuntimeCsvWriter(tmp_path / "missing-run")


def test_writer_emits_exact_header_and_rows(tmp_path):
    run_directory = make_run_directory(tmp_path)
    with BeliefRuntimeCsvWriter(run_directory) as writer:
        writer.write(valid_sample())
        writer.write(valid_sample(sim_time_ns=2_000_000_000, analysis_sequence=2))

    with (run_directory / BELIEF_RUNTIME_FILENAME).open(newline="") as source:
        rows = list(csv.reader(source))
    assert tuple(rows[0]) == BELIEF_RUNTIME_COLUMNS
    assert len(rows) == 3
    assert rows[1][0] == "1000000000"
    assert rows[1][-1] == "1"


def test_writer_rejects_existing_stream(tmp_path):
    run_directory = make_run_directory(tmp_path)
    (run_directory / BELIEF_RUNTIME_FILENAME).write_text("existing\n")
    with pytest.raises(FileExistsError):
        BeliefRuntimeCsvWriter(run_directory)


def test_writer_requires_strict_timestamp_and_sequence_progression(tmp_path):
    run_directory = make_run_directory(tmp_path)
    with BeliefRuntimeCsvWriter(run_directory) as writer:
        writer.write(valid_sample())
        with pytest.raises(ValueError):
            writer.write(valid_sample(sim_time_ns=1_000_000_000, analysis_sequence=2))
        with pytest.raises(ValueError):
            writer.write(valid_sample(sim_time_ns=2_000_000_000, analysis_sequence=1))
