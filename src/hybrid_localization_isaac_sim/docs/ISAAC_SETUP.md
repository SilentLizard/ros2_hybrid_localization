# HEROS -> Isaac Sim 6.0 setup

This is the package-local quick setup reference. The repository-level
`docs/isaac_sim.md` contains the complete workflow and current limitations.

## 1. Start Isaac through the repository launcher

```bash
cd ~/Development/ros2_hybrid_localization
tools/isaac_sim/run.sh
```

The repository is mounted inside the container at
`/workspace/ros2_hybrid_localization`, and the package root is exported through
`HYBRID_LOCALIZATION_ISAAC_SIM_ROOT` so Script Editor temporary files can still
resolve configuration data.

## 2. Import or load HEROS

Import `robot/heros_3w_isaac.urdf` as a floating/mobile robot, or load the saved
working scene. The current graph helper expects the imported root:

```text
/World/heros_3w
```

The validated hierarchy separates geometry and physics scopes and contains the
rear drive joints:

```text
joint_base_wheel_rear_left
joint_base_wheel_rear_right
```

Use `scripts/inspect_heros_stage.py` when importer nesting changes.

## 3. Scanner fixture

The Innok source description contains no SICK lidar or scanner mount. The
current fixture therefore adds `base_scan` at approximately:

```text
[0.95, 0.0, 0.10] m relative to base_link
```

Ensure exactly one SICK microScan3-style RTX `OmniLidar` exists beneath
`base_scan`. In the validated HEROS stage the sensor is already oriented
correctly, so `base_scan -> lidar_frame` is identity. Do not apply the old 180°
TurtleBot correction at the same time.

## 4. Build matching world geometry

Run `scripts/build_heros_world.py` in the Isaac Script Editor. It creates static
geometry below:

```text
/World/HEROS_TestWorld
```

from `config/heros_world_layout.json`.

The ROS occupancy map is generated from the same layout using:

```bash
python3 scripts/generate_heros_map.py
```

The currently validated map orientation includes the required image-axis
conversion. Do not manually flip the map in RViz.

## 5. Create ROS graphs

Run `scripts/heros_isaac_graph_helper.py` in the Script Editor. It resolves the
current HEROS articulation hierarchy and creates:

```text
/cmd_vel
/clock
/odom
/tf
/tf_static
/scan
```

The source-backed drive constants are 0.16 m wheel radius, 0.58 m wheel
separation, 1.0 m/s maximum linear speed, and 2.0 rad/s maximum angular speed.

## 6. Press Play and verify

Outside the Isaac container, initialize ROS/Fast DDS first:

```bash
source /opt/ros/jazzy/setup.bash
source ~/.ros/isaac_fastdds_env.sh
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
source ~/Development/ros2_hybrid_localization/install/setup.bash
```

Then:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link base_scan
ros2 run tf2_ros tf2_echo base_scan lidar_frame
```

## 7. Start observation mode

```bash
ros2 run hybrid_localization_isaac_sim start_observation_mode.sh
```

Before interpreting AMCL/GMM quality:

```bash
ros2 run hybrid_localization_isaac_sim validate_scan_map_alignment.py --tolerance 0.05
```

The #7 validated run achieved a 0.746 hit fraction at 0.05 m and 0.994 at
0.10 m after map-orientation correction.
