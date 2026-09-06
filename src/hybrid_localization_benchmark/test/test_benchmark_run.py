from pathlib import Path
import signal
import subprocess

import pytest

from hybrid_localization_benchmark.benchmark_run import (
    LiveRunConfig,
    create_live_run_directory,
    recorder_commands,
    summary_command,
    terminate_processes,
)


def config(tmp_path: Path, **changes) -> LiveRunConfig:
    values = dict(
        output_root=tmp_path,
        run_id="S07-live-test",
        scenario_id="S07",
        world_scenario="S07",
        seed=1680,
        duration_s=30.0,
    )
    values.update(changes)
    return LiveRunConfig(**values)


def test_creates_manifest_only(tmp_path: Path) -> None:
    run_directory = create_live_run_directory(config(tmp_path))
    assert (run_directory / "run.json").is_file()
    assert sorted(p.name for p in run_directory.iterdir()) == ["run.json"]


def test_rejects_invalid_duration(tmp_path: Path) -> None:
    with pytest.raises(ValueError):
        config(tmp_path, duration_s=0.0).validate()


def test_builds_all_three_recorder_commands(tmp_path: Path) -> None:
    cfg = config(tmp_path)
    commands = recorder_commands(cfg, tmp_path / cfg.run_id)
    assert [command[3] for command in commands] == [
        "localization_recorder.py",
        "belief_runtime_recorder.py",
        "resource_recorder.py",
    ]
    assert all("--run-directory" in command for command in commands)


def test_resource_command_uses_process_match_by_default(tmp_path: Path) -> None:
    command = recorder_commands(config(tmp_path), tmp_path / "run")[2]
    assert "--process-match" in command
    assert "particle_analysis_observer" in command
    assert "--pid" not in command


def test_resource_command_prefers_explicit_pid(tmp_path: Path) -> None:
    command = recorder_commands(
        config(tmp_path, particle_analysis_pid=1234), tmp_path / "run"
    )[2]
    assert command[-2:] == ("--pid", "1234")
    assert "--process-match" not in command


def test_summary_command_preserves_acceptance_arguments(tmp_path: Path) -> None:
    command = summary_command(
        tmp_path / "run",
        ("--max-position-rmse-m", "0.1"),
    )
    assert command[-2:] == ("--max-position-rmse-m", "0.1")


class FakeProcess:
    def __init__(self, pid: int = 1234, timeout: bool = False) -> None:
        self.pid = pid
        self.timeout = timeout
        self.wait_calls = 0

    def wait(self, timeout=None):
        self.wait_calls += 1
        if self.timeout and self.wait_calls == 1:
            raise subprocess.TimeoutExpired("fake", timeout)
        return 0


def test_terminate_processes_signals_process_group(monkeypatch) -> None:
    signals = []
    monkeypatch.setattr("os.killpg", lambda pgid, sig: signals.append((pgid, sig)))
    process = FakeProcess(pid=4321)
    terminate_processes([process])  # type: ignore[arg-type]
    assert signals == [(4321, signal.SIGTERM)]


def test_terminate_processes_escalates_process_group_after_timeout(monkeypatch) -> None:
    signals = []
    monkeypatch.setattr("os.killpg", lambda pgid, sig: signals.append((pgid, sig)))
    process = FakeProcess(pid=4321, timeout=True)
    terminate_processes([process], timeout_s=0.01)  # type: ignore[arg-type]
    assert signals == [
        (4321, signal.SIGTERM),
        (4321, signal.SIGKILL),
    ]
