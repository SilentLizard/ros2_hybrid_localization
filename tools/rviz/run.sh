#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
IMAGE_NAME="${RVIZ_IMAGE:-ros-jazzy-hybrid-localization-rviz}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${REPO_ROOT}/src/hybrid_localization_ros/rviz/particle_analysis.rviz}"
FAST_DDS_PROFILE="${FAST_DDS_PROFILE:-${HOME}/rviz-container/config/fastdds.xml}"

[[ -f "$RVIZ_CONFIG" ]] || { echo "RViz config not found: $RVIZ_CONFIG" >&2; exit 2; }

DOCKER_ARGS=(
  --rm -it
  --name rviz-hybrid-localization
  --network host
  --ipc host
  -e DISPLAY="${DISPLAY}"
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  -e ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
  -e QT_X11_NO_MITSHM=1
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  -v "${RVIZ_CONFIG}:/config/particle_analysis.rviz:ro"
  --device /dev/dri:/dev/dri
)

if [[ -f "$FAST_DDS_PROFILE" ]]; then
  DOCKER_ARGS+=(
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/config/fastdds.xml
    -v "${FAST_DDS_PROFILE}:/config/fastdds.xml:ro"
  )
else
  echo "Note: Fast DDS profile not found at $FAST_DDS_PROFILE; using default DDS discovery."
fi

exec docker run "${DOCKER_ARGS[@]}" "$IMAGE_NAME" \
  --ros-args -p use_sim_time:=true -- \
  -d /config/particle_analysis.rviz
