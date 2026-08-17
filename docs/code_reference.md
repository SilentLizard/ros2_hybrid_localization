# Code reference

This document is a navigation aid for the public C++ interfaces. It complements
the algorithm and runtime documentation; it is not generated API documentation.

## Code overview

### `hybrid_localization_core`

The core library is ROS-independent. Public headers live under
`src/hybrid_localization_core/include/hybrid_localization_core/`.

| Header | Responsibility |
|---|---|
| `types.hpp` | `Pose2d`, weighted particles, Gaussian components |
| `geometry.hpp` | wrapped-angle normalization and differences |
| `particle_statistics.hpp` | particle validation, normalization, means, covariance, ESS |
| `particle_clustering.hpp` | weighted DBSCAN-style SE(2) clustering |
| `gaussian_statistics.hpp` | weighted Gaussian fitting |
| `gaussian_mixture.hpp` | cluster-to-GMM conversion and discarded-mass accounting |
| `gaussian_fit_quality.hpp` | Mahalanobis and angular Gaussian-fit diagnostics |
| `gaussian_mixture_prediction.hpp` | SE(2) mixture motion prediction |
| `gaussian_mixture_update.hpp` | generic direct-pose Gaussian correction |
| `gaussian_mixture_management.hpp` | normalization, pruning, hard component cap |
| `gaussian_mixture_merging.hpp` | deterministic compatible-component merging |
| `gaussian_mixture_splitting.hpp` | evidence-driven component splitting |
| `gaussian_mixture_tracker.hpp` | stateful recursive tracker orchestration |
| `localization_health.hpp` | representation-level health metrics |
| `particle_gmm_comparison.hpp` | particle/GMM disagreement metrics |
| `localization_evidence_policy.hpp` | stateless threshold policy for transition evidence |
| `transition_supervisor.hpp` | temporal hysteresis and representation authority state machine |
| `recovery_sampling.hpp` | local particles sampled from retained Gaussian components |
| `global_particle_sampling.hpp` | map-aware global exploration samples |
| `adaptive_recovery_sampling.hpp` | bounded local/global recovery allocation |
| `hypothesis_provenance.hpp` | stable IDs and direct-parent provenance |

#### Important core conventions

- Poses are SE(2): `[x, y, yaw]`.
- Yaw is normalized to `[-pi, pi)`.
- Covariance arrays are row-major over `[x, y, yaw]`.
- Retained GMM weights preserve absolute normalized particle mass.
- `sum(component.weight) + discarded_weight == 1` within floating-point tolerance.
- The core does not own ROS lifecycle, TF publication, or middleware state.

### `hybrid_localization_ros`

Public headers live under
`src/hybrid_localization_ros/include/hybrid_localization_ros/`.

| Header | Responsibility |
|---|---|
| `amcl_particle_cloud_adapter.hpp` | validated Nav2 AMCL `ParticleCloud` -> core particle boundary |
| `particle_analysis_processor.hpp` | one-cloud observation analysis pipeline |
| `particle_analysis_parameters.hpp` | declaration and transactional update of live parameter groups |
| `particle_analysis_markers.hpp` | deterministic RViz marker generation |

The observation-mode processor is intentionally not the recursive GMM tracker.
It creates a fresh Gaussian approximation from each accepted AMCL cloud,
computes representation health/evidence, and publishes diagnostics while AMCL
remains authoritative.

### ROS messages

`src/hybrid_localization_msgs/msg/` contains the wire-level representation for
Gaussian components, mixtures, health, evidence, supervisor status, provenance,
and the aggregate particle-analysis result. Message definitions are kept in a
separate package so ROS interfaces do not leak into the numerical core.

### Simulator integration

`hybrid_localization_isaac_sim` is integration code rather than numerical core.
The graph helper constructs the current ROS bridge paths for drive, clock,
odometry/TF, and RTX lidar output. The fixture-specific scanner mount remains a
simulation assumption and should not be interpreted as manufacturer geometry.

## Where to start when reading the code

The repository separates algorithmic code from ROS integration so the numerical
and state-management layers can be tested independently.

### Recursive Gaussian-mixture tracker

A useful starting point for the core architecture is:

```text
src/hybrid_localization_core/
├── include/hybrid_localization_core/gaussian_mixture_tracker.hpp
├── src/gaussian_mixture_tracker.cpp
└── test/test_gaussian_mixture_tracker.cpp
```

`GaussianMixtureTracker` owns the current mixture and hypothesis-ID generator.
An update performs the bounded recursive processing sequence:

```text
prediction
  -> optional measurement correction
  -> health evaluation
  -> normalization / pruning
  -> deterministic merging
  -> optional evidence-driven splitting
  -> final component-budget enforcement
  -> state commit
```

The update is transactional: tracker state and update sequence are committed only
after the complete processing cycle succeeds.

The associated implementation leads naturally into:

```text
gaussian_mixture_prediction.*
gaussian_mixture_update.*
gaussian_mixture_management.*
gaussian_mixture_merging.*
gaussian_mixture_splitting.*
hypothesis_provenance.*
localization_health.*
```

### Representation supervision

For the policy/state-machine side, start with:

```text
src/hybrid_localization_core/
├── src/localization_evidence_policy.cpp
├── src/transition_supervisor.cpp
├── test/test_localization_evidence_policy.cpp
└── test/test_transition_supervisor.cpp
```

The evidence policy converts numerical localization metrics into instantaneous
transition evidence. The supervisor separately owns temporal behavior such as
consecutive-evidence requirements, dwell time, cooldown, recovery escalation,
and localization authority.

Keeping numerical thresholds and temporal state-machine behavior separate is an
intentional design decision.

### Probability and clustering

For the particle-to-Gaussian path:

```text
particle_statistics.*
  -> particle_clustering.*
  -> gaussian_statistics.*
  -> gaussian_mixture.*
  -> gaussian_fit_quality.*
```

Important conventions include wrapped SE(2) yaw handling and explicit
probability-mass accounting:

```text
sum(component.weight) + discarded_weight == 1
```

within floating-point tolerance.

### Current runnable observation mode

For the current runnable observation mode, a useful path is:

```text
amcl_particle_cloud_adapter.hpp/.cpp
  -> particle_analysis_processor.hpp/.cpp
      -> particle_clustering.*
      -> gaussian_mixture.*
      -> localization_health.*
      -> localization_evidence_policy.*
  -> particle_analysis_observer.cpp
  -> particle_analysis_markers.*
  -> particle_analysis_visualization_node.cpp
```

A useful implementation/test pair is:

```text
src/hybrid_localization_ros/src/particle_analysis_processor.cpp
src/hybrid_localization_ros/test/test_particle_analysis_processor.cpp
```

The processor receives the validated AMCL particle representation and executes
the ROS-independent analysis pipeline. Sequence numbers and hypothesis-ID
allocation are updated transactionally only after the complete analysis and
message-conversion path succeeds.

The adapter and processor illustrate the intended dependency direction:

```text
ROS / Nav2 messages
       |
       v
validated ROS boundary
       |
       v
ROS-independent core algorithms
       |
       v
ROS observation messages
```

ROS message types do not leak into `hybrid_localization_core`.
