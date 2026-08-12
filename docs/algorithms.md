# Algorithms

## Scope

The repository contains more core localization functionality than is currently
connected to ROS observation mode. This document separates the algorithms that
are actively used in the live observation-mode workflow from core algorithms intended for
later GMM tracking, recovery, and authority switching.

## SE(2) representation

Core poses use:

```text
[x, y, yaw]
```

Yaw is normalized to `[-pi, pi)`, angular differences use the shortest wrapped
angle, and Gaussian covariance is stored row-major over `[x, y, yaw]`.

Eigen3 is the numerical backend for fixed-size 3x3 covariance operations while
the public covariance representation remains a row-major nine-element array.

## Particle validation and statistics

Incoming particle weights are validated and normalized at the ROS/core
boundary. The core computes:

- weighted XY mean;
- circular yaw mean;
- weighted population covariance;
- effective sample size (ESS).

For normalized weights `w_i`, ESS is conceptually:

```text
ESS = 1 / sum(w_i^2)
```

A concentrated particle belief has lower ESS than a uniformly weighted cloud
of the same size.

## Weighted particle clustering

Observation mode uses a weighted DBSCAN-style clustering method in SE(2).
Neighborhood distance combines normalized translational and angular separation.
Conceptually:

```text
normalized XY distance  ~= position difference / position_scale
normalized yaw distance ~= wrapped yaw difference / yaw_scale
```

`epsilon` defines the neighborhood radius in this normalized space. A core
particle must satisfy the configured neighbor-count and local-weight criteria.
Clusters below `minimum_cluster_weight` are not retained and their mass becomes
noise/discarded mass.

The implementation intentionally favors deterministic behavior and validation
over asymptotic efficiency. The current neighbor search is quadratic.

## Particle clusters to Gaussian mixture

Each retained particle cluster is fit to one SE(2) Gaussian. The fit uses:

- weighted XY mean;
- circular weighted yaw mean;
- weighted covariance over `[x, y, yaw]`.

Retained component weights preserve **absolute normalized particle mass**. They
are not renormalized to sum to one after noise is removed. Instead the mixture
stores explicit discarded mass:

```text
sum(component.weight) + discarded_weight == 1
```

within floating-point tolerance.

This accounting matters later for recovery and for determining whether a GMM
actually represents enough of the original particle belief.

## Hypothesis provenance

Gaussian components carry a nonzero 64-bit hypothesis ID plus compact direct
parentage metadata. Core operations follow these identity rules:

- particle-cluster fitting creates root hypotheses;
- prediction/correction preserve identity;
- merging creates a new child with two parents;
- splitting creates new children with one direct parent each;
- pruning reports removed IDs.

Current ROS observation mode creates a new particle-derived mixture from each
AMCL cloud, so these IDs should not yet be interpreted as temporal track IDs
across independent AMCL resampling cycles.

## Localization health

Raw GMM health summarizes:

- represented/discarded probability mass;
- dominant component weight;
- normalized mixture entropy;
- effective component count;
- weighted and maximum position/yaw variance;
- optional measurement-update consistency;
- optional Gaussian fit quality;
- optional recovery failure severity.

The observation pipeline currently has mixture structure/uncertainty available;
measurement-update and recursive-tracker evidence are only populated when the
corresponding core functionality is used by a later runtime.

## Evidence policy

The evidence policy converts metrics into boolean instantaneous evidence such as:

```text
particle_belief_converged
gmm_available
gmm_health_good
gmm_health_bad
tracking_ambiguous
shadow_agreement_good
```

The policy is deterministic and stateless. It does not own dwell time,
consecutive-update counts, cooldown, or authority. Those temporal decisions
belong to the transition supervisor.

## Core algorithms already implemented for later phases

### Prediction

Each Gaussian component can be propagated by a body-frame SE(2) motion
increment. Covariance propagation uses state/motion Jacobians and adds process
noise while preserving wrapped yaw.

### Measurement update

The core supports direct SE(2) pose observations with covariance, wrapped-yaw
innovation, Mahalanobis gating, Kalman correction, Joseph-form covariance
update, likelihood evaluation, and component reweighting.

This generic observation update is not yet connected to live lidar scan
matching in the ROS runtime.

### Mixture management

Implemented primitives include:

- normalization;
- minimum-weight pruning;
- hard maximum component count;
- deterministic moment-preserving merging;
- evidence-driven splitting;
- final component-budget enforcement.

### Recursive tracker

A ROS-independent `GaussianMixtureTracker` orchestrates:

```text
predict
 -> optional correct
 -> pre-topology management
 -> merge
 -> optional bounded split
 -> final management / hard cap
 -> health evaluation
```

The tracker owns one persistent mixture and one monotonic hypothesis-ID
generator. It is not yet the live ROS authority.

### Recovery sampling

Two particle sources are implemented:

- local recovery particles sampled from inflated retained Gaussian components;
- global exploratory particles sampled from valid occupancy-grid free space.

Discarded mass is not spatially reconstructable after compression, so global
particles represent new exploration rather than reconstructed old noise.

### Transition supervisor

The core state machine includes:

```text
PARTICLE_GLOBAL
PARTICLE_CONVERGING
MIXTURE_SHADOW
MIXTURE_TRACKING
TRACKING_AMBIGUOUS
LOCAL_RECOVERY
GLOBAL_RECOVERY
LOCALIZATION_LOST
```

It supports consecutive evidence, dwell time, cooldown, recovery escalation,
and explicit transition reasons. ROS lifecycle/authority integration remains a
future phase.
