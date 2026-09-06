"""Process-resource and update-timing recording primitives for benchmark Stage 4."""

from __future__ import annotations

from dataclasses import dataclass
import csv
import math
import os
from pathlib import Path
from typing import Final

from .run_schema import MANIFEST_FILENAME, OUTPUT_STREAMS


RESOURCE_FILENAME: Final[str] = "resources.csv"
RESOURCE_COLUMNS: Final[tuple[str, ...]] = next(
    stream.columns for stream in OUTPUT_STREAMS if stream.filename == RESOURCE_FILENAME
)


@dataclass(frozen=True)
class ResourceSample:
    monotonic_time_ns: int
    sim_time_ns: int
    process_cpu_percent: float
    process_rss_bytes: int
    update_duration_ns: int
    update_period_ns: int

    def validate(self) -> None:
        if self.monotonic_time_ns < 0:
            raise ValueError("monotonic_time_ns must be non-negative")
        if self.sim_time_ns < 0:
            raise ValueError("sim_time_ns must be non-negative")
        if not math.isfinite(self.process_cpu_percent) or self.process_cpu_percent < 0.0:
            raise ValueError("process_cpu_percent must be finite and non-negative")
        if self.process_rss_bytes < 0:
            raise ValueError("process_rss_bytes must be non-negative")
        if self.update_duration_ns < 0:
            raise ValueError("update_duration_ns must be non-negative")
        if self.update_period_ns < 0:
            raise ValueError("update_period_ns must be non-negative")

    def to_row(self) -> tuple[object, ...]:
        self.validate()
        return (
            self.monotonic_time_ns,
            self.sim_time_ns,
            self.process_cpu_percent,
            self.process_rss_bytes,
            self.update_duration_ns,
            self.update_period_ns,
        )


def parse_proc_stat_total_ticks(text: str) -> int:
    """Return utime + stime ticks from one Linux /proc/<pid>/stat record.

    The command name is parenthesized and may contain spaces, so parsing begins
    after the final closing parenthesis rather than by naively splitting the
    entire line.
    """

    closing = text.rfind(")")
    opening = text.find("(")
    if opening <= 0 or closing <= opening:
        raise ValueError("invalid /proc stat record")

    fields_after_comm = text[closing + 1 :].strip().split()
    # fields_after_comm[0] is field 3 (state). Therefore fields 14/15 map to
    # indexes 11/12 in this suffix.
    if len(fields_after_comm) <= 12:
        raise ValueError("/proc stat record is missing CPU fields")

    try:
        utime = int(fields_after_comm[11])
        stime = int(fields_after_comm[12])
    except ValueError as exc:
        raise ValueError("/proc stat CPU fields must be integers") from exc

    if utime < 0 or stime < 0:
        raise ValueError("/proc stat CPU ticks must be non-negative")
    return utime + stime


def parse_proc_status_rss_bytes(text: str) -> int:
    """Return VmRSS in bytes from one Linux /proc/<pid>/status record."""

    for line in text.splitlines():
        if not line.startswith("VmRSS:"):
            continue
        parts = line.split()
        if len(parts) != 3 or parts[0] != "VmRSS:" or parts[2] != "kB":
            raise ValueError("unexpected VmRSS format")
        try:
            kib = int(parts[1])
        except ValueError as exc:
            raise ValueError("VmRSS value must be an integer") from exc
        if kib < 0:
            raise ValueError("VmRSS must be non-negative")
        return kib * 1024
    raise ValueError("VmRSS is missing from /proc status")


def read_process_snapshot(pid: int) -> tuple[int, int]:
    """Read (total CPU ticks, RSS bytes) for a Linux process."""

    if pid <= 0:
        raise ValueError("pid must be positive")
    proc = Path("/proc") / str(pid)
    try:
        stat_text = (proc / "stat").read_text(encoding="utf-8")
        status_text = (proc / "status").read_text(encoding="utf-8")
    except OSError as exc:
        raise RuntimeError(f"unable to read process {pid} from /proc") from exc
    return parse_proc_stat_total_ticks(stat_text), parse_proc_status_rss_bytes(status_text)


def find_unique_process(match: str, proc_root: Path = Path("/proc")) -> int:
    """Find exactly one process whose executable argv[0] basename contains *match*.

    Matching only argv[0] avoids false positives from ``ros2 run`` wrappers and
    from this recorder's own ``--process-match`` argument.
    """

    needle = match.strip()
    if not needle:
        raise ValueError("process match must not be empty")

    matches: list[int] = []
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / "cmdline").read_bytes()
        except OSError:
            continue
        argv = [part.decode("utf-8", errors="replace") for part in raw.split(b"\0") if part]
        if not argv:
            continue
        executable_name = Path(argv[0]).name
        if needle in executable_name:
            matches.append(int(entry.name))

    if not matches:
        raise RuntimeError(f"no process executable matches {needle!r}")
    if len(matches) != 1:
        raise RuntimeError(
            f"process match {needle!r} is ambiguous; matched PIDs {sorted(matches)}"
        )
    return matches[0]


class ProcessResourceTracker:
    """Convert Linux process snapshots into interval CPU percentages."""

    def __init__(self, clock_ticks_per_second: int | None = None) -> None:
        ticks = (
            int(os.sysconf("SC_CLK_TCK"))
            if clock_ticks_per_second is None
            else int(clock_ticks_per_second)
        )
        if ticks <= 0:
            raise ValueError("clock_ticks_per_second must be positive")
        self._ticks_per_second = ticks
        self._previous_monotonic_ns: int | None = None
        self._previous_cpu_ticks: int | None = None

    def update(self, monotonic_time_ns: int, total_cpu_ticks: int) -> float | None:
        if monotonic_time_ns < 0 or total_cpu_ticks < 0:
            raise ValueError("resource tracker inputs must be non-negative")

        if self._previous_monotonic_ns is None:
            self._previous_monotonic_ns = monotonic_time_ns
            self._previous_cpu_ticks = total_cpu_ticks
            return None

        assert self._previous_cpu_ticks is not None
        elapsed_ns = monotonic_time_ns - self._previous_monotonic_ns
        cpu_ticks = total_cpu_ticks - self._previous_cpu_ticks
        if elapsed_ns <= 0:
            raise ValueError("monotonic process samples must increase strictly")
        if cpu_ticks < 0:
            raise ValueError("process CPU ticks must not move backwards")

        self._previous_monotonic_ns = monotonic_time_ns
        self._previous_cpu_ticks = total_cpu_ticks

        elapsed_seconds = elapsed_ns / 1_000_000_000.0
        cpu_seconds = cpu_ticks / float(self._ticks_per_second)
        return 100.0 * cpu_seconds / elapsed_seconds


class ResourceCsvWriter:
    """Create and append schema-v1 resources.csv rows safely."""

    def __init__(self, run_directory: Path) -> None:
        self._run_directory = Path(run_directory)
        if not (self._run_directory / MANIFEST_FILENAME).is_file():
            raise FileNotFoundError("run directory must contain Stage-1 run.json")

        self.path = self._run_directory / RESOURCE_FILENAME
        if self.path.exists():
            raise FileExistsError(f"refusing to overwrite existing {self.path}")

        self._file = self.path.open("x", encoding="utf-8", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(RESOURCE_COLUMNS)
        self._file.flush()

        self._last_monotonic_time_ns: int | None = None
        self._last_sim_time_ns: int | None = None

    def write(self, sample: ResourceSample) -> None:
        sample.validate()
        if (
            self._last_monotonic_time_ns is not None
            and sample.monotonic_time_ns <= self._last_monotonic_time_ns
        ):
            raise ValueError("resource monotonic timestamps must increase strictly")
        if self._last_sim_time_ns is None:
            if sample.update_period_ns != 0:
                raise ValueError("first resource row must use update_period_ns = 0")
        else:
            if sample.sim_time_ns <= self._last_sim_time_ns:
                raise ValueError("resource simulation timestamps must increase strictly")
            expected_period = sample.sim_time_ns - self._last_sim_time_ns
            if sample.update_period_ns != expected_period:
                raise ValueError("update_period_ns must equal the simulation timestamp delta")

        self._writer.writerow(sample.to_row())
        self._file.flush()
        self._last_monotonic_time_ns = sample.monotonic_time_ns
        self._last_sim_time_ns = sample.sim_time_ns

    def close(self) -> None:
        self._file.close()

    def __enter__(self) -> "ResourceCsvWriter":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:  # type: ignore[no-untyped-def]
        self.close()
