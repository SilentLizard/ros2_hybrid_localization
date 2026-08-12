# RViz Docker client

The RViz client is intentionally independent of the Isaac container. It can run
on a separate Linux workstation with X11 and Docker.

Build once (or after Dockerfile changes):

```bash
tools/rviz/build.sh
```

Run:

```bash
tools/rviz/run.sh
```

The script mounts the repository's canonical
`src/hybrid_localization_ros/rviz/particle_analysis.rviz` directly and starts
RViz with `use_sim_time=true`.

ROS discovery defaults to:

```text
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

If `~/rviz-container/config/fastdds.xml` exists it is mounted automatically.
Override with:

```bash
FAST_DDS_PROFILE=/path/to/fastdds.xml tools/rviz/run.sh
```

Other overrides:

- `RVIZ_IMAGE=...`
- `RVIZ_CONFIG=/path/to/file.rviz`
- `ROS_DOMAIN_ID=...`
- `RMW_IMPLEMENTATION=...`

The same directory can be copied to a client without a full repository checkout
if `RVIZ_CONFIG` points to a local copy of `particle_analysis.rviz`.
