#!/usr/bin/env python3
"""Generate Stage-5 summary.json and deterministic acceptance results."""

from __future__ import annotations

import argparse
from pathlib import Path

from hybrid_localization_benchmark.run_summary import AcceptanceCriteria, write_run_summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-directory", required=True, type=Path)
    parser.add_argument("--min-localization-samples", type=int, default=1)
    parser.add_argument("--min-belief-runtime-samples", type=int, default=1)
    parser.add_argument("--min-resource-samples", type=int, default=1)
    parser.add_argument("--max-position-rmse-m", type=float)
    parser.add_argument("--max-position-error-m", type=float)
    parser.add_argument("--max-yaw-rmse-rad", type=float)
    parser.add_argument("--max-yaw-error-rad", type=float)
    parser.add_argument("--max-mean-cpu-percent", type=float)
    parser.add_argument("--max-peak-rss-bytes", type=int)
    parser.add_argument("--max-mean-update-duration-ns", type=float)
    parser.add_argument("--max-update-duration-ns", type=int)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    criteria = AcceptanceCriteria(
        min_localization_samples=args.min_localization_samples,
        min_belief_runtime_samples=args.min_belief_runtime_samples,
        min_resource_samples=args.min_resource_samples,
        max_position_rmse_m=args.max_position_rmse_m,
        max_position_error_m=args.max_position_error_m,
        max_yaw_rmse_rad=args.max_yaw_rmse_rad,
        max_yaw_error_rad=args.max_yaw_error_rad,
        max_mean_cpu_percent=args.max_mean_cpu_percent,
        max_peak_rss_bytes=args.max_peak_rss_bytes,
        max_mean_update_duration_ns=args.max_mean_update_duration_ns,
        max_update_duration_ns=args.max_update_duration_ns,
    )
    path = write_run_summary(args.run_directory, criteria)
    print(path)


if __name__ == "__main__":
    main()
