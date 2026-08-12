# Architecture

## Purpose

The project investigates a hybrid localization architecture that changes belief
representation according to uncertainty. Particles are well suited to global
initialization, disconnected hypotheses, and recovery. A bounded Gaussian
mixture is intended to become the efficient steady-state representation once the
belief has collapsed into a small number of locally Gaussian modes.

It is important to distinguish the **implemented observation-mode runtime** from
the **target hybrid architecture**.

## Implemented observation-mode architecture

The current ROS runtime does not yet perform GMM tracking or estimator authority
switching. Nav2 AMCL is the active localizer.

```text
                   occupancy map
                        |
                        v
/scan /odom /tf -->   AMCL
                        |
                        | /particle_cloud
                        | /amcl_pose
                        | map -> odom
                        v
             AmclParticleCloudAdapter
                        |
                        v
             ParticleAnalysisProcessor
              |       |       |       |
              |       |       |       +-> TransitionEvidence
              |       |       +----------> LocalizationHealth
              |       +------------------> GaussianMixture
              +--------------------------> ParticleAnalysis
                                                |
                                                v
                                 RViz marker visualization
```

### Authority

AMCL currently owns localization authority:

```text
map -> odom       Nav2 AMCL
odom -> base_link simulator / robot odometry
```

The hybrid observation node publishes diagnostics only. It does not publish a
competing `map -> odom`, does not deactivate AMCL, and does not claim GMM
authority.

### AMCL dependency

AMCL currently provides two things required by the runtime:

1. the authoritative pose/TF correction;
2. the weighted particle cloud consumed by the hybrid analysis pipeline.

The ROS adapter was validated against ROS 2 Jazzy/Nav2 AMCL. The observed
`/particle_cloud` contract is:

- type: `nav2_msgs/msg/ParticleCloud`;
- reliability: BEST_EFFORT;
- durability: VOLATILE;
- frame: `map`;
- adaptive particle count;
- finite, non-negative weights normalized to approximately one.

The adapter normalizes again at the ROS/core boundary and validates malformed
inputs before the core algorithms see them.

## Package boundaries

### `hybrid_localization_core`

ROS-independent C++20 implementation of:

- SE(2) geometry and circular angle handling;
- particle-weight validation/statistics;
- weighted clustering;
- Gaussian fitting and mixture representation;
- fit-quality metrics;
- prediction and correction primitives;
- pruning, merging, splitting, and hard component budgeting;
- recovery sampling;
- localization health metrics;
- particle/GMM comparison;
- evidence policy and transition supervisor;
- stateful GMM tracker orchestration.

The core deliberately does not depend on ROS messages, TF, lifecycle nodes, or
simulator APIs.

### `hybrid_localization_msgs`

ROS interfaces for:

- `GaussianComponent`;
- `GaussianMixture`;
- `HypothesisProvenance`;
- `LocalizationHealth`;
- `TransitionEvidence`;
- `LocalizationStatus`;
- `ParticleAnalysis`.

The covariance convention is row-major `[x, y, yaw]` with nine values.

### `hybrid_localization_ros`

Current ROS integration layer:

- Nav2 AMCL particle-cloud adapter;
- particle-analysis processor/node;
- runtime parameter translation and transactional validation;
- observation-mode RViz marker generator/node.

### `hybrid_localization_isaac_sim`

Development fixture for the current simulation-first validation path:

- Innok HEROS 3W model;
- SICK microScan3-style RTX lidar fixture;
- deterministic occupancy/world layout;
- map and AMCL configuration;
- Isaac graph helper;
- gamepad helper;
- observation-mode startup helper;
- scan/map alignment validator.

It is not yet the fully packaged deterministic experiment platform planned for
later simulator milestones.

## Observation processing

For every accepted AMCL particle cloud, the current processor executes:

```text
validate / normalize particles
        -> effective sample size
        -> weighted SE(2) clustering
        -> retained clusters + noise mass
        -> fit one Gaussian per retained cluster
        -> explicit discarded probability mass
        -> localization health metrics
        -> instantaneous transition evidence
        -> ROS observation messages
```

The observation node creates fresh root hypotheses from each incoming particle
cloud. Stable recursive GMM tracking and temporal hypothesis association are not
yet active in ROS observation mode.

## Target hybrid architecture

The completed core already contains many of the algorithms required for the
later runtime, but they are not all connected to live ROS inputs yet.

The intended runtime is:

```text
particle global initialization
        |
        v
particle convergence evidence
        |
        v
bounded GMM shadow validation
        |
        v
GMM authority / efficient tracking
        |
        +---- healthy ----> continue bounded GMM tracking
        |
        +---- inconsistent/ambiguous ----> local particle recovery
                                            |
                                            +-> global recovery if required
```

Only one subsystem may own authoritative localization TF at a time. The future
transition supervisor and TF authority manager must make handover explicit and
continuous.

## Why startup GMM disagreement is acceptable now

At global initialization AMCL can represent a broad, irregular, or multimodal
belief. Compressing that distribution into a small number of Gaussians is less
representative than it is after convergence. In the validated HEROS nominal
scenario the extracted GMM becomes closely consistent with the particle cloud
once AMCL converges.

This is not evidence that the current GMM is already a replacement localizer;
it is evidence that the particle-to-GMM extraction behaves sensibly in the
regime where a Gaussian representation is expected to become useful.
