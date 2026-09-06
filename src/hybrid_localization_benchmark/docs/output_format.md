# Benchmark output format — schema version 1

## Purpose

A benchmark run is a self-contained, append-only measurement artifact tied to
one deterministic localization scenario and estimator configuration. Stage 1
defines the contract; Stages 2–5 populate the measurement files.

## Directory contract

```text
<output-root>/<run-id>/
├── run.json
├── localization.csv
├── belief_runtime.csv
├── resources.csv
└── summary.json
```

`run.json` is written before recording starts and is immutable. A run ID may
never reuse an existing directory; this prevents samples from separate runs
being mixed accidentally.

## Run manifest

`run.json` schema version 1 records:

- `run_id`: portable unique identifier chosen by the runner;
- `scenario_id`: active runtime localization scenario, e.g. `S07`;
- `world_scenario`: referenced generated-world scenario;
- `seed`: deterministic scenario seed;
- `estimator`: measured localization implementation, initially `amcl`;
- `started_at_utc`: wall-clock provenance timestamp in UTC;
- `ros_distro`: ROS distribution, initially `jazzy`;
- `git_commit`: repository revision when available;
- clock semantics and the exact expected columns for each stream.

## Clock contract

`sim_time_ns` is the primary join key for localization and belief samples. It
is ROS simulation time represented as integer nanoseconds, avoiding floating
point timestamp drift. Resource measurements additionally use
`monotonic_time_ns`, because CPU/memory sampling is a host-side activity whose
sampling cadence need not match simulation updates.

Wall-clock time is metadata only and must not be used as the deterministic
benchmark sample axis.

## Stage 2 — `localization.csv`

Comma-separated UTF-8 with one header row:

```text
sim_time_ns,ground_truth_x_m,ground_truth_y_m,ground_truth_yaw_rad,estimate_x_m,estimate_y_m,estimate_yaw_rad,position_error_m,yaw_error_rad
```

Ground truth comes from `/hybrid_localization/ground_truth/pose` as
`geometry_msgs/PoseStamped`. The initial estimate source is `/amcl_pose` as
`geometry_msgs/PoseWithCovarianceStamped`. Both must be expressed in `map`.

Each estimate is paired with the nearest ground-truth sample within the configured
simulation-time tolerance. The online synchronizer waits until ground truth has
advanced beyond the complete matching window before emitting a row; this makes
nearest-neighbor selection deterministic even when the closest ground-truth sample
arrives after the estimate. Equal-distance ties prefer the earlier ground-truth
timestamp. No pose interpolation is performed. Estimates with no ground-truth
sample inside the tolerance are dropped rather than emitted with fabricated data.

`sim_time_ns` in each row is the localization-estimate timestamp. Position error
is Euclidean XY distance in metres. Yaw error is the absolute shortest wrapped
angular difference in radians.

## Stage 3 — `belief_runtime.csv`

```text
sim_time_ns,particle_count,effective_sample_size,gmm_component_count,gmm_represented_weight,gmm_discarded_weight,gmm_entropy,dominant_component_weight,analysis_sequence
```

The schema intentionally contains both particle and GMM fields so the same
format remains usable when Phase C shadow tracking is added. Unavailable fields
may be emitted empty by a recorder, but their columns remain stable within
schema version 1.

## Stage 4 — `resources.csv`

```text
monotonic_time_ns,sim_time_ns,process_cpu_percent,process_rss_bytes,update_duration_ns,update_period_ns
```

One row is written for each accepted `/hybrid_localization/particle_analysis` update.
The row combines host-side process resource sampling with timing attached to that
analysis update:

- `monotonic_time_ns`: host `CLOCK_MONOTONIC`/steady-clock timestamp taken by the benchmark recorder;
- `sim_time_ns`: the `ParticleAnalysis.header.stamp` simulation timestamp;
- `process_cpu_percent`: target-process CPU consumption over the host-monotonic interval since the previous resource sample, where 100% represents one fully occupied CPU core and values above 100% are valid for multithreaded processes;
- `process_rss_bytes`: Linux `/proc/<pid>/status` `VmRSS`, converted from KiB to bytes;
- `update_duration_ns`: steady-clock duration of the actual `ParticleAnalysisProcessor::process(...)` call, measured in the observation node and carried in `ParticleAnalysis.processing_duration_ns`;
- `update_period_ns`: difference between consecutive `ParticleAnalysis.header.stamp` simulation timestamps; the first row uses `0`.

CPU time is read from Linux `/proc/<pid>/stat` (`utime + stime`) and normalized by
the host monotonic interval. The recorder accepts an explicit PID and can otherwise
resolve exactly one process whose executable name contains `particle_analysis_observer`.
Ambiguous process matches are rejected rather than guessed.

This stage intentionally measures the current observation pipeline process. Later
Phase-C GMM tracker processes can use the same recorder contract by selecting the
corresponding PID and update message source.

## Stage 5 — `summary.json`

Reserved for aggregate statistics, sample counts, run duration, deterministic
acceptance criteria, pass/fail state, and explicit failure reasons. Stage 5
will define its detailed object schema without changing the Stage-1 run
identity or CSV stream contracts unless a schema-version increment is required.

## Versioning rule

Breaking changes to file names, column semantics, units, or required manifest
fields require incrementing `schema_version`. Adding optional summary fields or
new backward-compatible metadata does not require changing existing CSV
columns.


### Stage 3: `belief_runtime.csv`

One row is written per accepted `/hybrid_localization/particle_analysis` message using the schema-v1 columns.

## Stage 5: `summary.json`

`summary.json` is generated after the three CSV streams are complete. It retains
schema version and run identity, then reports deterministic aggregate metrics:

- localization: sample count/duration plus mean, RMSE, p95, and maximum position/yaw error;
- belief runtime: particle count, ESS and ESS ratio, GMM component count, represented/discarded mass, entropy, and dominant weight summaries;
- resources: mean/p95/max CPU, mean/max RSS, mean/p95/max processing duration, and non-zero update-period summaries.

The `acceptance` object contains the complete criteria used, every individual
check (`actual`, `operator`, `limit`, `passed`), and one aggregate `passed`
boolean. Stream sample-count minima are always evaluated. Quantitative limits
are optional until a scenario defines them. `summary.json` is never overwritten;
a repeated summarization attempt is rejected to preserve run artifact integrity.

## Stage 6 live-run orchestration

`run_benchmark.py` does not introduce a new artifact or schema. It is the orchestration layer that completes one schema-v1 run:

1. creates `<output-root>/<run-id>/run.json`;
2. starts `localization_recorder.py`, `belief_runtime_recorder.py`, and `resource_recorder.py` against that same immutable run directory;
3. records for the requested host wall-clock duration, while all sample timestamps remain governed by their documented simulation/monotonic clocks;
4. terminates recorder children gracefully, escalating only if necessary;
5. invokes `summarize_run.py` and writes `summary.json`.

The runner fails if any recorder exits early. Existing run directories remain non-reusable, so a failed or interrupted run must be kept as evidence or removed explicitly before choosing a new run ID.

Scenario activation is deliberately outside the benchmark runner. The live benchmark must be started only after the selected Isaac/ROS scenario is ACTIVE and its observation topics are available.
