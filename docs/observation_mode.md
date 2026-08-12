# Observation mode

## What observation mode does

Observation mode is the first complete live ROS workflow in this repository.
It observes Nav2 AMCL without taking localization authority.

The workflow:

1. receives AMCL's weighted `/particle_cloud`;
2. validates and normalizes the cloud;
3. clusters particles in SE(2);
4. converts retained clusters into a bounded Gaussian mixture;
5. computes particle/GMM health metrics;
6. produces instantaneous transition evidence;
7. publishes ROS messages and RViz markers.

AMCL remains authoritative throughout this mode.

## Prerequisites

The ROS side requires:

- a `/map` occupancy grid;
- `/scan` sensor input;
- a valid `odom -> base_link` TF chain;
- Nav2 AMCL configured for `map`, `odom`, and `base_link`;
- AMCL `/particle_cloud` output;
- simulation time when using Isaac.

For the current HEROS simulator fixture, initialize every external ROS terminal
with:

```bash
source /opt/ros/jazzy/setup.bash
source ~/.ros/isaac_fastdds_env.sh
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
source ~/Development/ros2_hybrid_localization/install/setup.bash
```

## Start the observation stack

After Isaac is already publishing `/clock`, `/scan`, `/odom`, `/tf`, and
`/tf_static`:

```bash
ros2 run hybrid_localization_isaac_sim start_observation_mode.sh
```

The helper starts:

```text
nav2_map_server/map_server
nav2_amcl/amcl
hybrid_localization_ros/particle_analysis_observer
hybrid_localization_ros/particle_analysis_visualization
```

It configures and activates the map server and AMCL lifecycle nodes, waits for
`/map` and `/particle_cloud`, then starts analysis and visualization. Ctrl-C
stops only the ROS-side observation processes; Isaac and independent gamepad
teleoperation may remain running.

The default files are resolved from installed package shares:

```text
hybrid_localization_isaac_sim/maps/heros_localization_map.yaml
hybrid_localization_isaac_sim/config/amcl_heros.yaml
hybrid_localization_ros/config/particle_analysis.yaml
```

They may be overridden with:

```bash
MAP_YAML=/path/map.yaml \
AMCL_PARAM_FILE=/path/amcl.yaml \
PARTICLE_ANALYSIS_PARAM_FILE=/path/analysis.yaml \
ros2 run hybrid_localization_isaac_sim start_observation_mode.sh
```

## Verify the runtime

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic hz /particle_cloud
ros2 topic hz /hybrid_localization/particle_analysis
ros2 topic hz /hybrid_localization/rviz_markers
ros2 run tf2_ros tf2_echo map base_link
```

Expected lifecycle state:

```text
active [3]
```

The particle-cloud rate is update-driven rather than fixed.

## Topics

### Inputs

`particle_analysis_observer` consumes:

```text
/particle_cloud    nav2_msgs/msg/ParticleCloud
```

The measured AMCL QoS is BEST_EFFORT/VOLATILE. The adapter preserves source
frame/timestamp metadata and requires the expected `map` frame.

### Outputs

The observer publishes:

```text
/hybrid_localization/particle_analysis
/hybrid_localization/gaussian_mixture
/hybrid_localization/localization_health
/hybrid_localization/transition_evidence
```

The visualization node consumes the aggregate analysis topic using reliable,
volatile QoS and publishes:

```text
/hybrid_localization/rviz_markers
```

### `ParticleAnalysis`

The aggregate message contains:

- source header;
- analysis sequence;
- particle count;
- retained cluster count;
- effective sample size;
- retained cluster mass;
- noise mass;
- dominant cluster mass;
- Gaussian mixture;
- localization health;
- instantaneous transition evidence.

## Runtime parameters

Parameters are loaded from:

```text
src/hybrid_localization_ros/config/particle_analysis.yaml
```

and may be changed live on `/particle_analysis_observer`. Updates are applied as
one prospective configuration and validated before commit. If any part of a
transaction is invalid, the previous configuration remains active.

### Particle clustering

The clustering metric normalizes translational and yaw separation before
applying the neighborhood radius. These parameters have immediate visible
effects on retained clusters and therefore on the extracted GMM.

| Parameter | Default | Meaning and expected effect |
|---|---:|---|
| `particle_clustering.position_scale` | `0.25` m | Translation represented by one normalized distance unit. Smaller values make the same physical XY separation look larger and therefore tend to split clusters more aggressively. Larger values tend to connect particles over larger physical distances. Must be positive. |
| `particle_clustering.yaw_scale` | `0.35` rad | Yaw represented by one normalized distance unit. Smaller values make angular differences more significant; larger values make clustering more tolerant of yaw spread. Must be positive. |
| `particle_clustering.epsilon` | `1.0` | Maximum normalized neighborhood distance. Increasing it generally connects more particles/modes; decreasing it generally yields more separated clusters/noise. |
| `particle_clustering.minimum_neighbors` | `5` | Minimum particle count in a core neighborhood, including the center particle. Increasing it suppresses small/sparse modes; decreasing it makes sparse structures easier to retain. |
| `particle_clustering.minimum_core_weight` | `0.0` | Minimum normalized probability mass in a core neighborhood. Raising it requires locally meaningful probability mass rather than particle count alone. |
| `particle_clustering.minimum_cluster_weight` | `0.01` | Minimum normalized mass for a final retained cluster. Raising it discards low-probability modes into noise/discarded mass. |

Example:

```bash
ros2 param set /particle_analysis_observer particle_clustering.epsilon 0.8
```

This is useful for live tuning because changes should be visible immediately in
the number/shape of GMM components.

### Health and evidence policy

These parameters do **not** switch estimators in the current runtime. They only
set thresholds for the instantaneous `TransitionEvidence` message and the RViz
health/status display. Temporal hysteresis and authority transfer belong to the
later supervisor runtime.

#### Particle convergence evidence

| Parameter | Default | Effect |
|---|---:|---|
| `health_policy.particle_minimum_retained_weight` | `0.80` | Minimum particle mass that must belong to retained clusters for convergence evidence. Higher is stricter. |
| `health_policy.particle_minimum_dominant_weight` | `0.55` | Minimum mass of the dominant retained cluster. Higher requires stronger single-mode dominance. |
| `health_policy.particle_maximum_noise_weight` | `0.20` | Maximum allowed unclustered/noise mass. Lower is stricter. |
| `health_policy.particle_maximum_retained_clusters` | `3` | Maximum retained mode count compatible with convergence evidence. Lower values reject more multimodal beliefs. |

#### GMM availability and healthy-entry thresholds

| Parameter | Default | Effect |
|---|---:|---|
| `health_policy.gmm_minimum_available_represented_weight` | `0.05` | Minimum represented mixture mass before a GMM is considered available. |
| `health_policy.good_minimum_represented_weight` | `0.80` | Healthy GMM requires at least this represented mass. |
| `health_policy.good_minimum_dominant_weight` | `0.45` | Healthy GMM requires at least this dominant-component mass. |
| `health_policy.good_maximum_normalized_entropy` | `0.75` | Healthy GMM requires entropy no greater than this; lower means more concentrated component mass. |
| `health_policy.good_maximum_weighted_position_variance` | `1.00` m² | Healthy GMM requires weighted position variance no greater than this. |
| `health_policy.good_maximum_weighted_yaw_variance` | `0.50` rad² | Healthy GMM requires weighted yaw variance no greater than this. |

#### Bad-entry thresholds

| Parameter | Default | Effect |
|---|---:|---|
| `health_policy.bad_maximum_represented_weight` | `0.50` | Low represented mass at or below this contributes bad-health evidence. |
| `health_policy.bad_maximum_dominant_weight` | `0.20` | Low dominant mass at or below this contributes bad-health evidence. |
| `health_policy.bad_minimum_normalized_entropy` | `0.95` | High entropy at or above this contributes bad-health evidence. |
| `health_policy.bad_minimum_weighted_position_variance` | `4.00` m² | Large weighted position variance at or above this contributes bad-health evidence. |
| `health_policy.bad_minimum_weighted_yaw_variance` | `1.50` rad² | Large weighted yaw variance at or above this contributes bad-health evidence. |

Good and bad thresholds deliberately leave a neutral deadband. Cross-threshold
combinations are validated and contradictory updates are rejected.

#### Ambiguity evidence

| Parameter | Default | Effect |
|---|---:|---|
| `health_policy.ambiguity_entry_minimum_entropy` | `0.75` | Entropy threshold for entering ambiguity evidence. |
| `health_policy.ambiguity_exit_maximum_entropy` | `0.55` | Lower entropy threshold for leaving ambiguity, forming hysteresis. |
| `health_policy.ambiguity_entry_minimum_effective_component_count` | `2.50` | Effective component count indicating multiple meaningful modes. |
| `health_policy.ambiguity_exit_maximum_effective_component_count` | `1.75` | Lower effective count compatible with ambiguity exit. |
| `health_policy.ambiguity_entry_maximum_dominant_weight` | `0.45` | Weak dominant component can trigger ambiguity. |
| `health_policy.ambiguity_exit_minimum_dominant_weight` | `0.60` | Stronger dominance required to exit ambiguity. |
| `health_policy.ambiguity_component_budget_fraction` | `0.90` | Treats proximity to the configured component budget as ambiguity pressure. |
| `health_policy.maximum_component_count` | `8` | Component budget used by the evidence policy. This is not yet the active ROS recursive tracker cap because recursive GMM tracking is not connected in observation mode. |

Only the parameter subset above is exposed in the current ROS node. Additional
core policy fields for measurement consistency, fit quality, shadow comparison,
and recovery already exist in the library but are not yet exposed by the current observation-mode node.

## RViz semantics

The canonical config is:

```text
src/hybrid_localization_ros/rviz/particle_analysis.rviz
```

For every GMM component, the marker generator publishes:

- namespace `gmm_covariance`: 2-sigma XY covariance ellipse;
- namespace `gmm_mean`: mean sphere;
- namespace `gmm_heading`: yaw arrow;
- namespace `gmm_label`: text label.

The label format is:

```text
H<id>  w=<weight>  n=<source particle count>
```

The health text reports GOOD/BAD/NEUTRAL, optional ambiguity, component count,
represented/discarded mass, entropy, particle count, ESS, retained cluster
count, and noise mass.

The authority marker always reports:

```text
Authority: AMCL / particles
Hybrid output: observation only
```

Stale component marker slots are deleted individually rather than using
`DELETEALL`, reducing RViz flicker when component counts decrease.

## Interpreting startup

Do not expect the extracted GMM to summarize the initial AMCL cloud as cleanly
as a converged particle belief. A broad or strongly multimodal distribution can
be poorly approximated by a small number of Gaussians. In the current nominal
HEROS validation, GMM and particle cloud become closely consistent after AMCL
convergence.
