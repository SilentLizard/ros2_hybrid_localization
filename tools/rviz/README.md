# RViz Docker client

Build once:

```bash
tools/rviz/build.sh
```

Run from the workstation that has X11 and Docker:

```bash
tools/rviz/run.sh
```

The script mounts the repository's current `particle_analysis.rviz` directly,
so there is no separate copy step after RViz configuration changes.

Optional overrides:

- `FAST_DDS_PROFILE=/path/to/fastdds.xml`
- `RVIZ_IMAGE=...`
- `RVIZ_CONFIG=/path/to/file.rviz`
- `ROS_DOMAIN_ID=...`
