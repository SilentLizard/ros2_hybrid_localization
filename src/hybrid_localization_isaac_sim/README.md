# Innok HEROS Isaac Sim development fixture

This package contains the current Isaac Sim 6.0 localization development fixture
used by `ros2_hybrid_localization`.

The active fixture is based on:

- Innok HEROS 3W differential-drive geometry;
- a SICK microScan3-style RTX `OmniLidar` simulation sensor;
- one shared world-layout JSON for occupancy-map and collision geometry;
- ROS 2 `/cmd_vel`, `/clock`, `/odom`, `/tf`, `/tf_static`, and `/scan` graphs;
- independent gamepad teleoperation;
- Nav2 map-server/AMCL observation-mode startup;
- scan/map alignment validation.

The package is a manually prepared development environment. It is not yet the
fully packaged deterministic experiment platform planned for later simulator
issues.

## Important fixture assumption

The original Innok description does not include the SICK scanner or a scanner
mount transform. The current `base_scan` pose is therefore a simulator fixture
assumption:

```text
x=0.95 m, y=0.0 m, z=0.10 m relative to base_link
```

The validated stage has the sensor oriented correctly in Isaac, so
`base_scan -> lidar_frame` uses identity rotation. Do not reintroduce the old
TurtleBot-derived 180-degree correction unless a future sensor asset actually
requires it.

## Shared map/world source

```text
config/heros_world_layout.json
  -> scripts/generate_heros_map.py -> maps/heros_localization_map.png
  -> scripts/build_heros_world.py  -> /World/HEROS_TestWorld
```

Always keep the generated map and simulated collision world synchronized before
interpreting localization quality.

## Host-side commands

Initialize ROS/Fast DDS outside the Isaac container:

```bash
source /opt/ros/jazzy/setup.bash
source ~/.ros/isaac_fastdds_env.sh
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
source ~/Development/ros2_hybrid_localization/install/setup.bash
```

Optional gamepad:

```bash
ros2 run hybrid_localization_isaac_sim start_gamepad.sh
```

Observation mode:

```bash
ros2 run hybrid_localization_isaac_sim start_observation_mode.sh
```

Geometry validation:

```bash
ros2 run hybrid_localization_isaac_sim validate_scan_map_alignment.py --tolerance 0.05
```

For complete setup and interpretation, see the repository-level
`docs/isaac_sim.md` and `docs/observation_mode.md`.
