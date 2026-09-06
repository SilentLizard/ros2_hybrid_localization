"""Stage-3 particle/GMM belief-runtime recording primitives."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Final

from .run_schema import MANIFEST_FILENAME, OUTPUT_STREAMS


BELIEF_RUNTIME_FILENAME: Final[str] = "belief_runtime.csv"
BELIEF_RUNTIME_COLUMNS: Final[tuple[str, ...]] = next(
    stream.columns for stream in OUTPUT_STREAMS if stream.filename == BELIEF_RUNTIME_FILENAME
)


@dataclass(frozen=True)
class BeliefRuntimeSample:
    """One atomic particle/GMM observation at a ROS simulation timestamp."""

    sim_time_ns: int
    particle_count: int
    effective_sample_size: float
    gmm_component_count: int
    gmm_represented_weight: float
    gmm_discarded_weight: float
    gmm_entropy: float
    dominant_component_weight: float
    analysis_sequence: int

    def validate(self) -> None:
        if self.sim_time_ns < 0:
            raise ValueError("sim_time_ns must be non-negative")
        if self.particle_count <= 0:
            raise ValueError("particle_count must be positive")
        if self.gmm_component_count < 0:
            raise ValueError("gmm_component_count must be non-negative")
        if self.analysis_sequence <= 0:
            raise ValueError("analysis_sequence must be positive")

        finite_values = (
            self.effective_sample_size,
            self.gmm_represented_weight,
            self.gmm_discarded_weight,
            self.gmm_entropy,
            self.dominant_component_weight,
        )
        if not all(math.isfinite(value) for value in finite_values):
            raise ValueError("belief-runtime metrics must be finite")

        particle_count = float(self.particle_count)
        if self.effective_sample_size <= 0.0:
            raise ValueError("effective_sample_size must lie in (0, particle_count]")
        if self.effective_sample_size > particle_count and not math.isclose(
            self.effective_sample_size,
            particle_count,
            rel_tol=1e-12,
            abs_tol=1e-9,
        ):
            raise ValueError("effective_sample_size must lie in (0, particle_count]")
        for name, value in (
            ("gmm_represented_weight", self.gmm_represented_weight),
            ("gmm_discarded_weight", self.gmm_discarded_weight),
            ("gmm_entropy", self.gmm_entropy),
            ("dominant_component_weight", self.dominant_component_weight),
        ):
            if value < 0.0 or value > 1.0:
                raise ValueError(f"{name} must lie in [0, 1]")

        if not math.isclose(
            self.gmm_represented_weight + self.gmm_discarded_weight,
            1.0,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ValueError("represented and discarded GMM weights must sum to one")

        if self.gmm_component_count == 0:
            if self.gmm_represented_weight > 1e-9:
                raise ValueError("empty GMM cannot report represented weight")
            if self.dominant_component_weight > 1e-9:
                raise ValueError("empty GMM cannot report dominant component weight")
        elif self.dominant_component_weight > self.gmm_represented_weight + 1e-9:
            raise ValueError("dominant component weight cannot exceed represented weight")

    def to_row(self) -> tuple[object, ...]:
        self.validate()
        return (
            self.sim_time_ns,
            self.particle_count,
            min(self.effective_sample_size, float(self.particle_count)),
            self.gmm_component_count,
            self.gmm_represented_weight,
            self.gmm_discarded_weight,
            self.gmm_entropy,
            self.dominant_component_weight,
            self.analysis_sequence,
        )


class BeliefRuntimeCsvWriter:
    """Create and append the Stage-3 belief_runtime.csv artifact."""

    def __init__(self, run_directory: Path) -> None:
        self._run_directory = Path(run_directory)
        if not (self._run_directory / MANIFEST_FILENAME).is_file():
            raise FileNotFoundError(
                f"benchmark run manifest is missing: {self._run_directory / MANIFEST_FILENAME}"
            )

        self.path = self._run_directory / BELIEF_RUNTIME_FILENAME
        if self.path.exists():
            raise FileExistsError(f"refusing to overwrite existing benchmark stream: {self.path}")

        self._file = self.path.open("x", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file, lineterminator="\n")
        self._writer.writerow(BELIEF_RUNTIME_COLUMNS)
        self._file.flush()
        self._last_sim_time_ns: int | None = None
        self._last_analysis_sequence: int | None = None

    def write(self, sample: BeliefRuntimeSample) -> None:
        sample.validate()
        if self._last_sim_time_ns is not None and sample.sim_time_ns <= self._last_sim_time_ns:
            raise ValueError("belief-runtime timestamps must be strictly increasing")
        if (
            self._last_analysis_sequence is not None
            and sample.analysis_sequence <= self._last_analysis_sequence
        ):
            raise ValueError("analysis_sequence must be strictly increasing")

        self._writer.writerow(sample.to_row())
        self._file.flush()
        self._last_sim_time_ns = sample.sim_time_ns
        self._last_analysis_sequence = sample.analysis_sequence

    def close(self) -> None:
        if not self._file.closed:
            self._file.close()

    def __enter__(self) -> "BeliefRuntimeCsvWriter":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
