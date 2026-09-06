"""ROS-independent helpers for orchestrating one live benchmark run."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
import math
import os
import signal
import subprocess
from typing import Sequence

from .run_schema import BenchmarkRun, create_run_directory


@dataclass(frozen=True)
class LiveRunConfig:
    output_root: Path
    run_id: str
    scenario_id: str
    world_scenario: str
    seed: int
    estimator: str = "amcl"
    ros_distro: str = "jazzy"
    git_commit: str | None = None
    duration_s: float = 30.0
    expected_frame: str = "map"
    max_sync_delta_ms: float = 50.0
    particle_analysis_pid: int = 0
    process_match: str = "particle_analysis_observer"

    def validate(self) -> None:
        if not math.isfinite(self.duration_s) or self.duration_s <= 0.0:
            raise ValueError("duration_s must be finite and positive")
        if not math.isfinite(self.max_sync_delta_ms) or self.max_sync_delta_ms < 0.0:
            raise ValueError("max_sync_delta_ms must be finite and non-negative")
        if not self.expected_frame:
            raise ValueError("expected_frame must not be empty")
        if self.particle_analysis_pid < 0:
            raise ValueError("particle_analysis_pid must be non-negative")
        if not self.process_match:
            raise ValueError("process_match must not be empty")
        BenchmarkRun(
            run_id=self.run_id,
            scenario_id=self.scenario_id,
            world_scenario=self.world_scenario,
            seed=self.seed,
            estimator=self.estimator,
            started_at_utc="2000-01-01T00:00:00Z",
            ros_distro=self.ros_distro,
            git_commit=self.git_commit,
        ).validate()


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def create_live_run_directory(config: LiveRunConfig) -> Path:
    config.validate()
    run = BenchmarkRun(
        run_id=config.run_id,
        scenario_id=config.scenario_id,
        world_scenario=config.world_scenario,
        seed=config.seed,
        estimator=config.estimator,
        started_at_utc=utc_now_iso(),
        ros_distro=config.ros_distro,
        git_commit=config.git_commit,
    )
    return create_run_directory(config.output_root, run)


def recorder_commands(config: LiveRunConfig, run_directory: Path) -> tuple[tuple[str, ...], ...]:
    """Return deterministic ros2-run commands for the three Stage 2-4 recorders."""
    config.validate()
    common = ("--run-directory", str(run_directory))
    localization = (
        "ros2", "run", "hybrid_localization_benchmark", "localization_recorder.py",
        *common,
        "--max-sync-delta-ms", str(config.max_sync_delta_ms),
        "--expected-frame", config.expected_frame,
    )
    belief = (
        "ros2", "run", "hybrid_localization_benchmark", "belief_runtime_recorder.py",
        *common,
        "--expected-frame", config.expected_frame,
    )
    resource = [
        "ros2", "run", "hybrid_localization_benchmark", "resource_recorder.py",
        *common,
        "--expected-frame", config.expected_frame,
    ]
    if config.particle_analysis_pid > 0:
        resource.extend(("--pid", str(config.particle_analysis_pid)))
    else:
        resource.extend(("--process-match", config.process_match))
    return localization, belief, tuple(resource)


def summary_command(
    run_directory: Path,
    extra_args: Sequence[str] = (),
) -> tuple[str, ...]:
    return (
        "ros2", "run", "hybrid_localization_benchmark", "summarize_run.py",
        "--run-directory", str(run_directory),
        *tuple(extra_args),
    )


def _signal_process_group(process: subprocess.Popen[object], sig: int) -> None:
    """Signal a recorder process group created with ``start_new_session=True``."""
    try:
        os.killpg(process.pid, sig)
    except ProcessLookupError:
        pass


def terminate_processes(processes: Sequence[subprocess.Popen[object]], timeout_s: float = 5.0) -> None:
    """Terminate recorder process groups, escalating to SIGKILL when necessary."""
    for process in processes:
        _signal_process_group(process, signal.SIGTERM)

    for process in processes:
        try:
            process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            _signal_process_group(process, signal.SIGKILL)
            process.wait(timeout=timeout_s)
