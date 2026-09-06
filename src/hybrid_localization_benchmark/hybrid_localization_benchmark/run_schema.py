"""Versioned benchmark run metadata and output-file contract.

Stage 1 intentionally defines structure only. Later #9 stages populate the CSV
streams and summary without changing the run identity contract.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import re
from typing import Final


BENCHMARK_SCHEMA_VERSION: Final[int] = 1

_RUN_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")

# Canonical generated Isaac world identifiers remain strictly SNN because they
# refer directly to the committed/generated world catalog, for example S07 or
# S08.
_WORLD_SCENARIO_ID_RE = re.compile(r"^S[0-9]{2}$")

# Runtime experiment identifiers may either be one of the original canonical
# SNN scenarios or a named experiment layered on top of a canonical world.
#
# Examples:
#   S08
#   S0_BASELINE
#   S1_GLOBAL_INITIALIZATION
#   S3_KIDNAPPED
#   S9_COMBINED
#
# Keep the contract intentionally narrow and portable: runtime scenarios must
# begin with S followed by at least one digit and may then contain uppercase
# letters, digits, and underscores.
_RUNTIME_SCENARIO_ID_RE = re.compile(
    r"^S[0-9]+(?:_[A-Z0-9]+)*$"
)


@dataclass(frozen=True)
class BenchmarkStream:
    """One versioned file in a benchmark run directory."""

    filename: str
    stage: int
    purpose: str
    columns: tuple[str, ...]

    def to_dict(self) -> dict[str, object]:
        data = asdict(self)
        data["columns"] = list(self.columns)
        return data


OUTPUT_STREAMS: Final[tuple[BenchmarkStream, ...]] = (
    BenchmarkStream(
        filename="localization.csv",
        stage=2,
        purpose="Ground-truth and localization pose/error samples.",
        columns=(
            "sim_time_ns",
            "ground_truth_x_m",
            "ground_truth_y_m",
            "ground_truth_yaw_rad",
            "estimate_x_m",
            "estimate_y_m",
            "estimate_yaw_rad",
            "position_error_m",
            "yaw_error_rad",
        ),
    ),
    BenchmarkStream(
        filename="belief_runtime.csv",
        stage=3,
        purpose="Particle/GMM belief state and localization-runtime metrics.",
        columns=(
            "sim_time_ns",
            "particle_count",
            "effective_sample_size",
            "gmm_component_count",
            "gmm_represented_weight",
            "gmm_discarded_weight",
            "gmm_entropy",
            "dominant_component_weight",
            "analysis_sequence",
        ),
    ),
    BenchmarkStream(
        filename="resources.csv",
        stage=4,
        purpose="Process CPU, memory, and update-timing samples.",
        columns=(
            "monotonic_time_ns",
            "sim_time_ns",
            "process_cpu_percent",
            "process_rss_bytes",
            "update_duration_ns",
            "update_period_ns",
        ),
    ),
)

SUMMARY_FILENAME: Final[str] = "summary.json"
MANIFEST_FILENAME: Final[str] = "run.json"


@dataclass(frozen=True)
class BenchmarkRun:
    """Immutable identity and provenance for one benchmark execution."""

    run_id: str
    scenario_id: str
    world_scenario: str
    seed: int
    estimator: str
    started_at_utc: str
    ros_distro: str = "jazzy"
    git_commit: str | None = None
    schema_version: int = BENCHMARK_SCHEMA_VERSION

    def validate(self) -> None:
        if self.schema_version != BENCHMARK_SCHEMA_VERSION:
            raise ValueError(
                f"unsupported benchmark schema_version {self.schema_version}; "
                f"expected {BENCHMARK_SCHEMA_VERSION}"
            )

        if not _RUN_ID_RE.fullmatch(self.run_id):
            raise ValueError(
                "run_id must be a portable non-empty identifier"
            )

        if not _RUNTIME_SCENARIO_ID_RE.fullmatch(self.scenario_id):
            raise ValueError(
                "scenario_id must use a runtime scenario identifier such as "
                "S08, S0_BASELINE, or S9_COMBINED"
            )

        if not _WORLD_SCENARIO_ID_RE.fullmatch(self.world_scenario):
            raise ValueError(
                "world_scenario must use the canonical SNN form"
            )

        if self.seed < 0:
            raise ValueError("seed must be non-negative")

        if not self.estimator.strip():
            raise ValueError("estimator must not be empty")

        if not self.ros_distro.strip():
            raise ValueError("ros_distro must not be empty")

        if self.git_commit is not None and not self.git_commit.strip():
            raise ValueError(
                "git_commit must be null or a non-empty string"
            )

        try:
            parsed = datetime.fromisoformat(
                self.started_at_utc.replace("Z", "+00:00")
            )
        except ValueError as exc:
            raise ValueError(
                "started_at_utc must be valid ISO-8601"
            ) from exc

        if (
            parsed.tzinfo is None
            or parsed.utcoffset()
            != timezone.utc.utcoffset(parsed)
        ):
            raise ValueError(
                "started_at_utc must explicitly use UTC"
            )

    def to_manifest(self) -> dict[str, object]:
        self.validate()

        return {
            "schema_version": self.schema_version,
            "run": {
                "run_id": self.run_id,
                "scenario_id": self.scenario_id,
                "world_scenario": self.world_scenario,
                "seed": self.seed,
                "estimator": self.estimator,
                "started_at_utc": self.started_at_utc,
                "ros_distro": self.ros_distro,
                "git_commit": self.git_commit,
            },
            "clock_contract": {
                "primary_sample_clock": "ROS simulation time",
                "simulation_timestamp_unit": "nanoseconds",
                "resource_clock": "host monotonic time",
                "resource_timestamp_unit": "nanoseconds",
            },
            "streams": [
                stream.to_dict()
                for stream in OUTPUT_STREAMS
            ],
            "summary": {
                "filename": SUMMARY_FILENAME,
                "stage": 5,
                "purpose": (
                    "Aggregate run metrics and deterministic acceptance "
                    "result."
                ),
            },
        }


def create_run_directory(
    root: Path,
    run: BenchmarkRun,
) -> Path:
    """Create a new run directory and write only its immutable manifest.

    CSV files are created by their owning recorder stages. Refusing to reuse an
    existing directory prevents accidental mixing of samples from separate
    runs.
    """

    manifest = run.to_manifest()

    run_directory = root / run.run_id
    run_directory.mkdir(
        parents=True,
        exist_ok=False,
    )

    (run_directory / MANIFEST_FILENAME).write_text(
        json.dumps(
            manifest,
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )

    return run_directory