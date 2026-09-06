# hybrid_localization_benchmark

Benchmark and regression package for deterministic localization experiments.

Issue #9 is implemented in stages. Stage 1 defines a versioned run identity,
provenance manifest, clock contract, and stable output filenames/column names.
Stage 2 adds deterministic localization-accuracy recording against simulator
ground truth. Later stages add belief/runtime/resource streams and the aggregate
acceptance summary.

One benchmark execution owns one directory:

```text
<output-root>/<run-id>/
├── run.json                 # Stage 1: immutable identity/provenance + schema
├── localization.csv         # Stage 2: ground truth, estimate, pose error
├── belief_runtime.csv       # Stage 3: particle/GMM/runtime metrics
├── resources.csv            # Stage 4: CPU, memory, update timing
└── summary.json             # Stage 5: aggregate metrics + acceptance result
```

See [`docs/output_format.md`](docs/output_format.md) for the detailed contract.

## Stage 2 localization recorder

Stage 2 records AMCL pose accuracy using:

- ground truth: `/hybrid_localization/ground_truth/pose` (`geometry_msgs/PoseStamped`);
- estimate: `/amcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`);
- required frame: `map`;
- primary clock: integer ROS simulation-time nanoseconds.

The recorder pairs each estimate with the nearest ground-truth sample within a
configurable simulation-time tolerance. It waits until the ground-truth stream
has advanced beyond the complete matching window before committing a row, so a
closer future sample cannot change an already-recorded decision. No interpolation
is performed. Unmatched estimates are dropped and counted.

After a Stage-1 run directory has been created, the installed executable is:

```bash
ros2 run hybrid_localization_benchmark localization_recorder.py \
  --run-directory <output-root>/<run-id>
```

The default synchronization tolerance is 50 ms and can be changed with
`--max-sync-delta-ms`. Live S07 validation of the real topic timing is reserved
for #9 Stage 6.

## Stage 3 belief/runtime recorder

`belief_runtime_recorder.py` records one atomic row per `/hybrid_localization/particle_analysis` message. The aggregate message is used deliberately so particle and GMM metrics share one source timestamp and analysis sequence without cross-topic synchronization.

Recorded fields are the schema-v1 `belief_runtime.csv` columns: particle count, effective sample size, GMM component count, represented/discarded mass, normalized mixture entropy, dominant component weight, and analysis sequence. The recorder requires `map`, strictly increasing simulation timestamps, and strictly increasing analysis sequence numbers. It validates probability ranges and represented/discarded mass before writing.

```bash
ros2 run hybrid_localization_benchmark belief_runtime_recorder.py \
  --run-directory <output-root>/<run-id>
```

Stage 4 adds host process-resource and update-timing sampling. Live topic/rate validation remains deferred to Stage 6.


## Stage 4 — process resources and update timing

Stage 4 adds `resource_recorder.py`, which writes `resources.csv` once per accepted
`/hybrid_localization/particle_analysis` update. It combines Linux process metrics
with timing from the actual particle-analysis processing step.

The observation node now attaches `processing_duration_ns` to `ParticleAnalysis`;
this is measured with `std::chrono::steady_clock` around
`ParticleAnalysisProcessor::process(...)`, so it does not measure benchmark-recorder
overhead. The benchmark recorder reads the target process's CPU ticks and RSS from
`/proc`, computes CPU percentage over the host-monotonic interval, and derives the
update period from consecutive ROS simulation timestamps.

By default the recorder requires exactly one process whose command line contains
`particle_analysis_observer`; an explicit PID can be supplied to remove any ambiguity:

```bash
ros2 run hybrid_localization_benchmark resource_recorder.py \
  --run-directory <output-root>/<run-id> \
  --pid <particle-analysis-pid>
```

The Stage-1 schema remains unchanged. `process_cpu_percent` may exceed 100% for a
multithreaded process, RSS is recorded in bytes, and the first `update_period_ns`
value is zero because no previous analysis timestamp exists. Live process/PID
behavior is intentionally validated with the rest of the benchmark stack in Stage 6.

## Stage 5 run summary and acceptance

Stage 5 consumes the immutable `run.json` plus the completed Stage-2 through
Stage-4 CSV streams and writes `summary.json` exactly once. The summary contains
aggregate localization, belief-runtime, and resource metrics together with an
explicit deterministic acceptance result.

The default acceptance policy deliberately avoids inventing project performance
thresholds: it requires all three measurement streams to contain at least one
sample. Quantitative limits are opt-in and are recorded in `summary.json` so a
run's pass/fail result is reproducible and auditable. Supported limits include
position/yaw RMSE and maxima, mean CPU, peak RSS, and mean/maximum update duration.

Example:

```bash
ros2 run hybrid_localization_benchmark summarize_run.py \
  --run-directory <output-root>/<run-id> \
  --min-localization-samples 10 \
  --min-belief-runtime-samples 10 \
  --min-resource-samples 10 \
  --max-position-rmse-m 0.10 \
  --max-position-error-m 0.25
```

Scenario-specific quantitative thresholds are intentionally selected and
validated during Stage 6 live S07 acceptance rather than embedded as arbitrary
Stage-5 defaults.

## Stage 6 live benchmark runner

Stage 6 adds `run_benchmark.py`, which orchestrates one complete live benchmark capture without changing schema version 1. It creates the immutable run directory/manifest, starts the localization, belief-runtime, and resource recorders, stops them after the requested wall-clock duration (or Ctrl-C), and then runs `summarize_run.py` to produce `summary.json`.

The runner assumes the Isaac scenario, observation stack, ground truth, AMCL, and particle-analysis node are already active. It does not activate or reset the simulator itself; scenario control remains owned by `hybrid_localization_isaac_sim`.

Canonical S07 metadata is `scenario=S07`, `world_scenario=S07`, seed `1680`. A first structural live capture can therefore be run with:

```bash
ros2 run hybrid_localization_benchmark run_benchmark.py \
  --output-root ./benchmark_runs \
  --run-id S07-amcl-live-001 \
  --scenario S07 \
  --seed 1680 \
  --duration-s 30
```

By default the resource recorder requires exactly one process whose executable name contains `particle_analysis_observer`. Use `--particle-analysis-pid PID` when explicit PID selection is preferable.

Stage 6 should first validate end-to-end capture with structural minimum-sample checks. After inspecting that first real `summary.json`, rerun with documented quantitative limits where appropriate. This avoids inventing performance thresholds before the live measurement path has been exercised.
