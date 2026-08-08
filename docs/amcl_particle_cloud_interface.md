# Nav2 Jazzy AMCL particle-cloud runtime contract

Issue #41 validates the runtime interface consumed by the future
`hybrid_localization_ros` AMCL adapter.

## Verified environment

Validation was performed on ROS 2 Jazzy with Nav2 AMCL running against the
repository localization test fixture.

## Verified contract

- Resolved topic: `/particle_cloud`
- Message type: `nav2_msgs/msg/ParticleCloud`
- Publisher node: `/amcl`
- Publisher count: 1
- Reliability: `BEST_EFFORT`
- Durability: `VOLATILE`
- Frame: `map`
- Particle pose is planar: position `z == 0`; quaternion `x == y == 0` within
  floating-point tolerance.
- Quaternions are normalized.
- Particle weights are finite and non-negative.
- Observed complete-cloud particle weights sum to `1.0`.
- Particle count is adaptive rather than fixed.
- Observed sample counts during validation: 818, 742, 785, 774, 840.
- Observed publication rate is not fixed. During the motion run,
  `ros2 topic hz /particle_cloud` varied from roughly 4.5 to 8.3 Hz, with
  individual intervals from about 0.066 s to 0.950 s.
- Message timestamps use simulation/ROS time in this test environment.

## Adapter consequences for #6

The adapter must:

1. subscribe using sensor-data-compatible QoS (`BEST_EFFORT`, `VOLATILE`);
2. accept a variable particle count;
3. preserve header timestamp and frame metadata;
4. convert planar quaternion orientation to wrapped yaw;
5. reject non-finite poses and weights;
6. reject negative weights;
7. reject an empty cloud or zero total mass according to adapter policy;
8. normalize weights before handing them to the ROS-independent core rather
   than assuming upstream normalization as a permanent API invariant;
9. avoid assuming a fixed publication frequency.

## Validation commands

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic info /particle_cloud --verbose
ros2 topic echo /particle_cloud --once
python3 scripts/validate_amcl_particle_cloud.py \
  --topic /particle_cloud \
  --expected-frame map \
  --samples 5 \
  --timeout 30
ros2 topic hz /particle_cloud
```

The Python validator treats missing rclpy endpoint metadata as advisory when
real `ParticleCloud` messages are received. DDS graph endpoint discovery can
lag independently of message reception; `ros2 topic info --verbose` remains
the explicit QoS evidence for this validation issue.
