# HEROS -> Isaac Sim 6.0 setup

## 1. Import the robot

In Isaac Sim 6.0, use **File -> Import** and select `robot/heros_3w_isaac.urdf`.
Use a floating/mobile base (do not fix the base to the world). Keep the imported
root named `/World/innok_heros_3w`, or update `ROBOT_PRIM` in the graph helper.

The URDF was flattened from the Innok ROS 1 xacro package because Isaac imports
URDF, not the ROS package's Gazebo controller configuration. The manufacturer
meshes are retained, while Gazebo and ros_control tags are intentionally absent.

After import, confirm that the rear wheel joints are named:

- `joint_base_wheel_rear_left`
- `joint_base_wheel_rear_right`

Run `scripts/inspect_heros_stage.py` in the Script Editor if the importer creates
unexpected prim nesting.

## 2. Add the SICK RTX lidar

The supplied Innok package contains no lidar model or lidar mounting transform.
`base_scan` is therefore a **simulation assumption**, currently placed at
`[0.95, 0.0, 0.28]` m relative to `base_link`.

Attach the same SICK microScan3 RTX/OmniLidar asset used by the existing
TurtleBot fixture underneath the imported `base_scan` prim. Do not enable the
lidar debug visualization; the previous fixture showed that unnecessary sensor
render/debug work can contribute to WebRTC flicker.

The graph helper preserves the existing `base_scan -> lidar_frame` 180-degree
yaw correction. Revalidate it once on the HEROS model by checking the blind spot
and scan/map orientation. If the imported sensor orientation is corrected at the
USD level, set `SCAN_FRAME_YAW_DEGREES = 0.0` instead of applying both rotations.

## 3. Create ROS graphs

Open `scripts/heros_isaac_graph_helper.py` in Isaac's Script Editor and run it.
It creates the same external contract as the current TurtleBot fixture:

- `/cmd_vel`
- `/clock`
- `/odom`
- `/tf`
- `/tf_static`
- `/scan`

The drive constants are taken from the Innok description: 0.16 m wheel radius,
0.58 m wheel separation, 1.0 m/s maximum linear speed, and 2.0 rad/s maximum
angular speed.

## 4. Ground placement

The original ROS/Gazebo launch spawned HEROS at `z=0.4`. After importing into
Isaac, place the robot so the drive wheels and caster sit on the floor without
initial penetration. Exact root Z can depend on importer fixed-joint merging, so
check contacts visually/with physics rather than hard-coding the Gazebo spawn Z.

## 5. ROS validation

After pressing Play:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom lidar_frame
```

Then use `config/amcl_heros.yaml` with the same map-server/AMCL bringup pattern
as the existing issue-41 fixture.

## 6. Industrial-scale map

`maps/heros_localization_map.*` is a 30 m x 30 m, 5 cm/cell deterministic test
map with wider passages than the TurtleBot map. The corresponding collision
geometry still needs to be built in Isaac so rendered/raycast geometry and the
occupancy map match. Treat the PNG as the reference layout for that next step.

## 7. Build collision geometry from the map source

Before using AMCL for localization-quality checks, run
`scripts/build_heros_world.py` in Isaac Sim's Script Editor. The script creates
`/World/HEROS_TestWorld` from `config/heros_world_layout.json`.

`generate_heros_map.py` uses that same JSON file to produce
`maps/heros_localization_map.png`, so the occupancy grid and RTX-lidar collision
geometry have one source of truth. Do not validate AMCL/GMM accuracy against a
map whose corresponding Isaac collision geometry has not been generated.

After pressing Play, verify coarse alignment quantitatively with
`scripts/validate_scan_map_alignment.py` before interpreting the localization
result in RViz.
