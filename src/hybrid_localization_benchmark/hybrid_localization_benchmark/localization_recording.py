"""Deterministic localization-accuracy recording primitives for benchmark Stage 2."""

from __future__ import annotations

from dataclasses import dataclass
import csv
import math
from pathlib import Path
from typing import Final

from .run_schema import MANIFEST_FILENAME, OUTPUT_STREAMS


_LOCALIZATION_STREAM = next(stream for stream in OUTPUT_STREAMS if stream.stage == 2)
LOCALIZATION_FILENAME: Final[str] = _LOCALIZATION_STREAM.filename
LOCALIZATION_COLUMNS: Final[tuple[str, ...]] = _LOCALIZATION_STREAM.columns


def normalize_angle(angle_rad: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    if not math.isfinite(angle_rad):
        raise ValueError("angle must be finite")
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def wrapped_angle_difference(lhs_rad: float, rhs_rad: float) -> float:
    """Return lhs-rhs using the shortest wrapped angular difference."""
    return normalize_angle(lhs_rad - rhs_rad)


@dataclass(frozen=True)
class PoseSample:
    """One planar pose sample expressed in the benchmark map frame."""

    sim_time_ns: int
    x_m: float
    y_m: float
    yaw_rad: float

    def validate(self) -> None:
        if self.sim_time_ns < 0:
            raise ValueError("sim_time_ns must be non-negative")
        if not all(math.isfinite(value) for value in (self.x_m, self.y_m, self.yaw_rad)):
            raise ValueError("pose values must be finite")

    def normalized(self) -> "PoseSample":
        self.validate()
        return PoseSample(
            sim_time_ns=self.sim_time_ns,
            x_m=self.x_m,
            y_m=self.y_m,
            yaw_rad=normalize_angle(self.yaw_rad),
        )


@dataclass(frozen=True)
class LocalizationSample:
    """One synchronized ground-truth/localization measurement row."""

    sim_time_ns: int
    ground_truth_x_m: float
    ground_truth_y_m: float
    ground_truth_yaw_rad: float
    estimate_x_m: float
    estimate_y_m: float
    estimate_yaw_rad: float
    position_error_m: float
    yaw_error_rad: float

    def as_row(self) -> tuple[object, ...]:
        return (
            self.sim_time_ns,
            self.ground_truth_x_m,
            self.ground_truth_y_m,
            self.ground_truth_yaw_rad,
            self.estimate_x_m,
            self.estimate_y_m,
            self.estimate_yaw_rad,
            self.position_error_m,
            self.yaw_error_rad,
        )


def build_localization_sample(
    ground_truth: PoseSample,
    estimate: PoseSample,
) -> LocalizationSample:
    """Compute deterministic planar position/yaw error for a synchronized pair.

    The benchmark row timestamp is the localization-estimate timestamp. Ground
    truth may be slightly offset in simulation time as permitted by the
    synchronizer tolerance.
    """

    gt = ground_truth.normalized()
    est = estimate.normalized()
    dx = est.x_m - gt.x_m
    dy = est.y_m - gt.y_m
    return LocalizationSample(
        sim_time_ns=est.sim_time_ns,
        ground_truth_x_m=gt.x_m,
        ground_truth_y_m=gt.y_m,
        ground_truth_yaw_rad=gt.yaw_rad,
        estimate_x_m=est.x_m,
        estimate_y_m=est.y_m,
        estimate_yaw_rad=est.yaw_rad,
        position_error_m=math.hypot(dx, dy),
        yaw_error_rad=abs(wrapped_angle_difference(est.yaw_rad, gt.yaw_rad)),
    )


class PoseSynchronizer:
    """Pair estimate samples to nearest ground truth using simulation time.

    Estimates are emitted in timestamp order. A pending estimate is resolved
    only once ground truth has advanced beyond ``estimate_time + tolerance``;
    this guarantees that a closer future ground-truth sample cannot still
    arrive. Exact timestamp ties are resolved deterministically in favor of the
    earlier ground-truth sample. Unmatched estimates are counted and dropped.
    """

    def __init__(self, max_time_delta_ns: int) -> None:
        if max_time_delta_ns < 0:
            raise ValueError("max_time_delta_ns must be non-negative")
        self._max_time_delta_ns = max_time_delta_ns
        self._ground_truth: list[PoseSample] = []
        self._pending_estimates: list[PoseSample] = []
        self._last_ground_truth_time: int | None = None
        self._last_estimate_time: int | None = None
        self._matched_count = 0
        self._dropped_estimate_count = 0

    @property
    def matched_count(self) -> int:
        return self._matched_count

    @property
    def dropped_estimate_count(self) -> int:
        return self._dropped_estimate_count

    def add_ground_truth(self, sample: PoseSample) -> list[LocalizationSample]:
        sample = sample.normalized()
        self._check_monotonic(sample.sim_time_ns, self._last_ground_truth_time, "ground truth")
        self._last_ground_truth_time = sample.sim_time_ns
        self._ground_truth.append(sample)
        return self._resolve_ready(final=False)

    def add_estimate(self, sample: PoseSample) -> list[LocalizationSample]:
        sample = sample.normalized()
        self._check_monotonic(sample.sim_time_ns, self._last_estimate_time, "estimate")
        self._last_estimate_time = sample.sim_time_ns
        self._pending_estimates.append(sample)
        return self._resolve_ready(final=False)

    def finalize(self) -> list[LocalizationSample]:
        """Resolve all remaining estimates using the ground truth received."""
        return self._resolve_ready(final=True)

    @staticmethod
    def _check_monotonic(current: int, previous: int | None, label: str) -> None:
        if previous is not None and current <= previous:
            raise ValueError(f"{label} timestamps must be strictly increasing")

    def _resolve_ready(self, final: bool) -> list[LocalizationSample]:
        emitted: list[LocalizationSample] = []
        while self._pending_estimates:
            estimate = self._pending_estimates[0]
            if not final:
                if self._last_ground_truth_time is None:
                    break
                if self._last_ground_truth_time <= estimate.sim_time_ns + self._max_time_delta_ns:
                    break

            self._pending_estimates.pop(0)
            nearest = self._nearest_ground_truth(estimate.sim_time_ns)
            if nearest is None:
                self._dropped_estimate_count += 1
                continue
            emitted.append(build_localization_sample(nearest, estimate))
            self._matched_count += 1

        self._prune_ground_truth()
        return emitted

    def _nearest_ground_truth(self, timestamp_ns: int) -> PoseSample | None:
        best: PoseSample | None = None
        best_delta: int | None = None
        for sample in self._ground_truth:
            delta = abs(sample.sim_time_ns - timestamp_ns)
            if delta > self._max_time_delta_ns:
                continue
            if best_delta is None or delta < best_delta or (
                delta == best_delta and sample.sim_time_ns < best.sim_time_ns  # type: ignore[union-attr]
            ):
                best = sample
                best_delta = delta
        return best

    def _prune_ground_truth(self) -> None:
        if not self._ground_truth:
            return
        if self._pending_estimates:
            cutoff = self._pending_estimates[0].sim_time_ns - self._max_time_delta_ns
        elif self._last_estimate_time is not None:
            cutoff = self._last_estimate_time - self._max_time_delta_ns
        else:
            return

        keep_from = 0
        while keep_from + 1 < len(self._ground_truth) and self._ground_truth[keep_from + 1].sim_time_ns < cutoff:
            keep_from += 1
        if keep_from:
            del self._ground_truth[:keep_from]


class LocalizationCsvWriter:
    """Create and append Stage-2 localization rows inside an existing run."""

    def __init__(self, run_directory: Path) -> None:
        if not (run_directory / MANIFEST_FILENAME).is_file():
            raise FileNotFoundError(
                f"benchmark manifest not found: {run_directory / MANIFEST_FILENAME}"
            )
        self.path = run_directory / LOCALIZATION_FILENAME
        if self.path.exists():
            raise FileExistsError(f"localization output already exists: {self.path}")
        self._file = self.path.open("x", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file, lineterminator="\n")
        self._writer.writerow(LOCALIZATION_COLUMNS)
        self._file.flush()

    def append(self, sample: LocalizationSample) -> None:
        self._writer.writerow(sample.as_row())
        self._file.flush()

    def close(self) -> None:
        if not self._file.closed:
            self._file.close()

    def __enter__(self) -> "LocalizationCsvWriter":
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()
