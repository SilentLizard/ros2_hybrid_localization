# Innok HEROS Isaac Sim starter

This bundle converts the user-provided Innok HEROS ROS 1 description into a
practical first Isaac Sim 6.0 localization fixture.

## Why 3W first?

The Innok source contains both 3-wheel and 4-wheel configurations. The 3-wheel
version is the best first integration target because the source explicitly uses
a two-wheel differential drive for it and supplies a 3WD collision footprint.
That maps directly to Isaac's Differential Controller. A 4WD skid-steer model
should be added later when wheel-ground dynamics themselves become part of the
experiment.

## Included

- `robot/heros_3w_isaac.urdf` — Isaac-oriented, non-Gazebo URDF.
- `robot/meshes/` — required Innok HEROS STL geometry.
- `scripts/heros_isaac_graph_helper.py` — `/cmd_vel`, odom, TF, clock and RTX
  LaserScan graphs matching the existing simulator fixture.
- `scripts/inspect_heros_stage.py` — prim-path diagnostic helper.
- `config/heros_sim.yaml` — source-backed dimensions plus explicitly marked
  simulator assumptions.
- `config/amcl_heros.yaml` — AMCL starting point for the larger platform.
- `config/nav2_heros_footprint.yaml` — source footprint for later Nav2 bringup.
- `maps/heros_localization_map.*` — deterministic industrial-scale map.
- `scripts/generate_heros_map.py` — reproducible map generator.
- `docs/ISAAC_SETUP.md` — import and validation procedure.

## Manufacturer-derived values used

- standard wheel radius: 0.16 m
- wheel separation: 0.58 m
- maximum configured linear velocity: 1.0 m/s
- maximum configured angular velocity: 2.0 rad/s
- 3WD navigation footprint: x = -0.60..1.16 m, y = -0.39..0.39 m

## Deliberate assumptions

The uploaded Innok package contains no SICK lidar model or scanner transform.
The bundle therefore adds `base_scan` at x=0.95 m, z=0.28 m solely as a starting
simulation mount and retains the current TurtleBot fixture's 180-degree scan
frame correction. Both must be validated/replaced rather than treated as HEROS
specifications.

## Canonical fixture layout

The occupancy map and Isaac collision world are generated from the same
`config/heros_world_layout.json` source:

```text
heros_world_layout.json
  -> scripts/generate_heros_map.py -> maps/heros_localization_map.png
  -> scripts/build_heros_world.py  -> /World/HEROS_TestWorld
```

This is required before scan/map alignment is meaningful.

## Host-side bringup

After the HEROS robot, matching test-world geometry, lidar, and ROS graphs are
running in Isaac Sim:

```bash
ros2 run hybrid_localization_isaac_sim start_observation_mode.sh
```

Before judging localization quality, check coarse scan/map consistency:

```bash
ros2 run hybrid_localization_isaac_sim validate_scan_map_alignment.py
```
