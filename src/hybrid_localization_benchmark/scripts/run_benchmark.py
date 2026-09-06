#!/usr/bin/env python3
"""Run one complete live Stage-6 benchmark capture."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys
import time

from hybrid_localization_benchmark.benchmark_run import (
    LiveRunConfig,
    create_live_run_directory,
    recorder_commands,
    summary_command,
    terminate_processes,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture one complete localization benchmark run.")
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--world-scenario")
    parser.add_argument("--seed", required=True, type=int)
    parser.add_argument("--estimator", default="amcl")
    parser.add_argument("--ros-distro", default="jazzy")
    parser.add_argument("--git-commit")
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--expected-frame", default="map")
    parser.add_argument("--max-sync-delta-ms", type=float, default=50.0)
    parser.add_argument("--particle-analysis-pid", type=int, default=0)
    parser.add_argument("--process-match", default="particle_analysis_observer")
    parser.add_argument("--min-localization-samples", type=int, default=10)
    parser.add_argument("--min-belief-runtime-samples", type=int, default=10)
    parser.add_argument("--min-resource-samples", type=int, default=10)
    parser.add_argument("--max-position-rmse-m", type=float)
    parser.add_argument("--max-position-error-m", type=float)
    parser.add_argument("--max-yaw-rmse-rad", type=float)
    parser.add_argument("--max-yaw-error-rad", type=float)
    parser.add_argument("--max-mean-cpu-percent", type=float)
    parser.add_argument("--max-peak-rss-bytes", type=int)
    parser.add_argument("--max-mean-update-duration-ns", type=float)
    parser.add_argument("--max-update-duration-ns", type=int)
    return parser.parse_args()


def summary_args(args: argparse.Namespace) -> list[str]:
    values: list[str] = [
        "--min-localization-samples", str(args.min_localization_samples),
        "--min-belief-runtime-samples", str(args.min_belief_runtime_samples),
        "--min-resource-samples", str(args.min_resource_samples),
    ]
    optional = (
        ("--max-position-rmse-m", args.max_position_rmse_m),
        ("--max-position-error-m", args.max_position_error_m),
        ("--max-yaw-rmse-rad", args.max_yaw_rmse_rad),
        ("--max-yaw-error-rad", args.max_yaw_error_rad),
        ("--max-mean-cpu-percent", args.max_mean_cpu_percent),
        ("--max-peak-rss-bytes", args.max_peak_rss_bytes),
        ("--max-mean-update-duration-ns", args.max_mean_update_duration_ns),
        ("--max-update-duration-ns", args.max_update_duration_ns),
    )
    for flag, value in optional:
        if value is not None:
            values.extend((flag, str(value)))
    return values


def main() -> int:
    args = parse_args()
    config = LiveRunConfig(
        output_root=args.output_root,
        run_id=args.run_id,
        scenario_id=args.scenario,
        world_scenario=args.world_scenario or args.scenario,
        seed=args.seed,
        estimator=args.estimator,
        ros_distro=args.ros_distro,
        git_commit=args.git_commit,
        duration_s=args.duration_s,
        expected_frame=args.expected_frame,
        max_sync_delta_ms=args.max_sync_delta_ms,
        particle_analysis_pid=args.particle_analysis_pid,
        process_match=args.process_match,
    )

    run_directory = create_live_run_directory(config)
    print(f"benchmark run directory: {run_directory}")
    processes: list[subprocess.Popen[object]] = []
    try:
        for command in recorder_commands(config, run_directory):
            print("starting:", " ".join(command))
            processes.append(subprocess.Popen(command, start_new_session=True))
        deadline = time.monotonic() + config.duration_s
        while time.monotonic() < deadline:
            for process in processes:
                return_code = process.poll()
                if return_code is not None:
                    raise RuntimeError(f"benchmark recorder exited early with code {return_code}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("benchmark interrupted; finalizing captured samples", file=sys.stderr)
    finally:
        terminate_processes(processes)

    command = summary_command(run_directory, summary_args(args))
    print("summarizing:", " ".join(command))
    completed = subprocess.run(command, check=False)
    if completed.returncode != 0:
        return completed.returncode
    print(f"benchmark complete: {run_directory}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
