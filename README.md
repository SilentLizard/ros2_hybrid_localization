# ros2_hybrid_localization

Adaptive ROS 2 localization research prototype combining particle-filter global
localization/recovery with bounded Gaussian-mixture observation and tracking.

Current runtime work focuses on AMCL particle-cloud ingestion, particle-to-GMM
analysis, runtime parameterization, and RViz diagnostics while AMCL remains
authoritative.

Engineering architecture and roadmap are tracked in `PROJECT.md`; current
implementation status is tracked in `PROGRESS.md`.

## Repository layout

```text
src/hybrid_localization_core/       ROS-independent algorithms
src/hybrid_localization_msgs/       ROS interfaces
src/hybrid_localization_ros/        ROS adapters, observation node, RViz markers
src/hybrid_localization_isaac_sim/  HEROS Isaac fixture and observation bringup
tools/rviz/                         client-side RViz Docker tooling
scripts/                            generic repository validation utilities
docs/                               focused interface documentation
```

The HEROS fixture uses one shared world-layout definition to generate both the
occupancy map and Isaac collision geometry. Build both before interpreting
scan/map localization quality.
