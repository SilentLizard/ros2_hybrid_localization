# Current limitations

This project is a research prototype. The current documentation describes the
first complete observable runtime, not a production-ready hybrid localizer.

## AMCL is still the localization authority

Observation mode depends directly on Nav2 AMCL for:

- the weighted particle cloud;
- `/amcl_pose`;
- authoritative `map -> odom`.

The GMM currently visualized in ROS is extracted from AMCL particles. It is not
yet a recursively updated live alternative localizer and does not own TF.

Consequently, current live tests demonstrate particle analysis and Gaussian
compression quality, not resource savings from replacing AMCL.

## Startup GMM mismatch

At initialization, AMCL may have a broad or multimodal particle belief. A small
Gaussian mixture can be a poor compact representation of that distribution.
The current nominal validation showed weaker GMM/particle agreement initially
and close agreement after AMCL convergence.

That behavior is expected. The future supervisor must use convergence and
shadow-agreement evidence before considering GMM authority.

## Root hypothesis IDs are not temporal tracks in observation mode

The processor builds a fresh mixture from each incoming AMCL cloud. Particle
cluster-derived hypothesis IDs therefore identify hypotheses within the
analysis/provenance process but should not yet be interpreted as persistent
cluster identities across independent AMCL resampling cycles.

If later experiments show pathological spatial/component flicker despite a
stable particle distribution, temporal cluster association may need a dedicated
follow-up.

## Only a subset of core parameters is exposed live

The ROS observation node currently exposes:

```text
particle_clustering.*
health_policy.*
```

The core also contains prediction, update, GMM management, merging, splitting,
recovery, shadow comparison, and supervisor configuration. Those groups are not
all active ROS parameters because their corresponding live runtime nodes do not
exist yet.

## Health evidence is instantaneous

`TransitionEvidence` in observation mode is policy-thresholded but stateless.
The live observation node does not apply supervisor dwell times, consecutive
counts, cooldown, lifecycle switching, or TF authority transfer.

## Isaac Sim fixture is still manual

The current HEROS/SICK environment is much more representative than the earlier
TurtleBot development scene, but it is not yet a reproducible benchmark package.
Manual steps still include stage preparation, lidar presence/configuration, and
execution of the graph helper.

Missing simulator-platform features include:

- deterministic reset/teleport scenario controller;
- explicit ground-truth topic;
- scenario identity/seed management;
- sensor/odometry/map fault injection;
- automated benchmark execution;
- formal Isaac asset/version policy;
- automated pass/fail integration tests.

These belong to the later simulator-platform and benchmark work.

## Scanner mount is a fixture assumption

The supplied Innok description does not include the SICK scanner model or a
manufacturer scanner transform. The current `base_scan` placement is therefore
a simulator assumption validated against the test fixture, not a HEROS hardware
specification.

## Simulator configuration is not fully centralized yet

`heros_sim.yaml` documents fixture values, but the current Isaac graph helper
still contains hard-coded wheel dimensions, speed limits, frame names, and scan
frame identity. Centralizing simulator configuration and eliminating duplicate
source-of-truth values belongs naturally to the packaging work.

## Map/scan validator can be optimistic in symmetric geometry

The validator classifies scan endpoints as hits when they lie within a spatial
tolerance of occupied cells. In a symmetric map, an incorrectly oriented map
can still produce a deceptively good coarse score.

Use the validator as a gate together with:

- direct RViz map/scan inspection;
- asymmetric landmarks;
- tighter tolerance sweeps;
- TF inspection.

The current 5 cm and 10 cm results are useful sanity evidence, not a formal
localization-accuracy benchmark.

## Occupancy-grid resolution limits very tight endpoint tests

The current map uses 0.05 m cells. Wall rasterization, collision surface
thickness, lidar ray origin, and cell discretization make a 0.01 m endpoint
criterion unrealistically strict for this fixture. Failure at 1 cm does not by
itself imply an incorrect TF or AMCL estimate.

## Remote ROS networking requires environment setup

ROS processes outside the Isaac container currently rely on the local Fast DDS
environment and subnet discovery setup:

```bash
source ~/.ros/isaac_fastdds_env.sh
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

This is host/environment-specific and is not yet encapsulated by all runtime
helpers. A stale `ROS_LOCALHOST_ONLY` export may also produce deprecation
warnings even when it is later unset.

## RViz is diagnostic, not benchmark evidence

RViz is valuable for detecting map orientation, stale markers, GMM/particle
mismatch, and TF problems. Quantitative claims about localization accuracy,
resource efficiency, recovery time, or robustness require the later benchmark
recorder, ground truth, and deterministic scenarios.

## Current AMCL profile is a nominal validation profile

The HEROS AMCL YAML uses a deterministic initial pose and fixed random seed for
repeatable nominal observation testing. That configuration is not a global
localization experiment. Global ambiguity, kidnapped-robot behavior, and
recovery require dedicated later scenarios.

## No authoritative hybrid pose output yet

The target design requires one supervisor-owned authoritative output and
continuous handover between particle and GMM representations. The current
hybrid nodes intentionally do not publish authoritative TF. TF authority,
lifecycle integration, and continuity guards are future work.
