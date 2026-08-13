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

The package now uses schema-v2 layouts whose obstacle coordinates are expressed
in **ROS map-cell coordinates**: `(0,0)` is the lower-left map cell, +x points
right, and +y points up. PNG/image coordinates are never used as the shared
geometry representation.

```text
config/heros_world_layout.json
        |
        +-> scripts/generate_localization_map.py
        |      -> maps/heros_localization_map.{png,yaml}
        |
        +-> scripts/build_localization_world.py
               -> /World/HEROS_TestWorld
```

`generate_heros_map.py` and `build_heros_world.py` remain convenience wrappers
for the nominal HEROS layout.

A deterministic example of a more complex layout is provided at:

```text
config/worlds/warehouse_demo.json
```

The generic layout schema supports axis-aligned occupied rectangles and circular
pillars. New deterministic room/warehouse layouts can also be generated with
`scripts/generate_world_layout.py`.

The historical map mismatch came from mixing image coordinates and ROS/world
coordinates. The conversion is now centralized in `scripts/world_layout.py`
and covered by tests: occupancy rendering flips only the image Y axis, while
Isaac uses the ROS/world coordinates directly.

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


## Generate another map

Example deterministic warehouse layout:

```bash
python3 src/hybrid_localization_isaac_sim/scripts/generate_world_layout.py \
  --preset warehouse \
  --seed 41 \
  --output /tmp/warehouse.json

python3 src/hybrid_localization_isaac_sim/scripts/generate_localization_map.py \
  --layout /tmp/warehouse.json \
  --image /tmp/warehouse.png \
  --yaml /tmp/warehouse.yaml
```

## Change the Isaac collision world

Inside Isaac Script Editor, `scripts/change_isaac_world.py` replaces only the
static localization collision world. Set `TARGET_LAYOUT` in that script or set
`HYBRID_LOCALIZATION_WORLD_LAYOUT` in the Isaac container.

For example:

```text
config/worlds/warehouse_demo.json
```

Map-server switching is intentionally separate. The matching ROS map must be
generated/reloaded explicitly; synchronized reset/map/ground-truth scenario
control belongs to the later scenario-controller milestone.


## Deterministic scenario catalog

`config/world_scenarios.json` defines 20 reproducible generated environments
(`S00` ... `S19`). They vary:

- world size from roughly 30 m to 60 m at 0.05 m/cell;
- warehouse, room, mixed, and corridor topology;
- deterministic seed;
- small obstacle count;
- large obstacle count;
- pillar count;
- clear area around the physical HEROS spawn.

Materialize one:

```bash
python3 src/hybrid_localization_isaac_sim/scripts/materialize_world_scenario.py \
  --scenario S07
```

Or materialize all twenty once:

```bash
python3 src/hybrid_localization_isaac_sim/scripts/materialize_world_scenario.py \
  --all
```

Outputs are written under `generated/scenarios/<ID>/` and ignored by Git.
The catalog plus generator and seed are the reproducibility contract.

To switch Isaac after materialization, set in `change_isaac_world.py`:

```python
TARGET_SCENARIO = "S07"
```

The matching ROS map is:

```text
src/hybrid_localization_isaac_sim/generated/scenarios/S07/map.yaml
```

Start observation mode with:

```bash
export MAP_YAML="$PWD/src/hybrid_localization_isaac_sim/generated/scenarios/S07/map.yaml"
```

### AMCL initialization modes

Observation mode keeps the existing deterministic known-pose startup as the
default:

```bash
export AMCL_INIT_MODE=known
```

For a realistic unknown starting pose, use AMCL's global-localization service:

```bash
export AMCL_INIT_MODE=global
```

For a repeatable but intentionally uncertain/wrong prior:

```bash
export AMCL_INIT_MODE=random-prior
export AMCL_RANDOM_SEED=73
export AMCL_RANDOM_LAYOUT="$PWD/src/hybrid_localization_isaac_sim/generated/scenarios/S07/layout.json"
```

`global` is the preferred test when no initial pose should be assumed. The
random-prior mode is useful for repeatable convergence stress tests.

These reset only AMCL's belief. They do not teleport the Isaac robot. Physical
robot reset/teleport, synchronized world/map changes, scenario identity, and
ground truth belong to the later scenario-controller layer.
