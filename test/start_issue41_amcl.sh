#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PARAM_FILE="${SCRIPT_DIR}/amcl_issue41.yaml"
MAP_YAML="${SCRIPT_DIR}/localization_test_map.yaml"

if [[ ! -f "${MAP_YAML}" ]]; then
  echo "Map YAML not found: ${MAP_YAML}" >&2
  exit 2
fi

if [[ ! -f "${PARAM_FILE}" ]]; then
  echo "AMCL parameter file not found: ${PARAM_FILE}" >&2
  exit 2
fi

# ROS setup scripts may legitimately reference variables that are not defined
# in the calling shell. Source them before enabling bash nounset (-u).
source /opt/ros/jazzy/setup.bash

if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  source "${WORKSPACE_ROOT}/install/setup.bash"
fi

# Enable strict unset-variable checking for our own script after environment setup.
set -u

for topic in /clock /scan /odom /tf /tf_static; do
  if ! ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
    echo "Required localization-test topic is missing: ${topic}" >&2
    echo "Start the test fixture and ensure its ROS publishers are running." >&2
    exit 3
  fi
done

PIDS=()
cleanup() {
  set +e
  for pid in "${PIDS[@]:-}"; do
    kill "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Starting map_server with:"
echo "  ${MAP_YAML}"
ros2 run nav2_map_server map_server \
  --ros-args \
  -p use_sim_time:=true \
  -p yaml_filename:="${MAP_YAML}" &
PIDS+=("$!")

echo "Starting AMCL with:"
echo "  ${PARAM_FILE}"
ros2 run nav2_amcl amcl \
  --ros-args \
  --params-file "${PARAM_FILE}" &
PIDS+=("$!")

echo "Waiting for lifecycle services..."
for node in map_server amcl; do
  available=false
  for _ in $(seq 1 50); do
    if ros2 service list 2>/dev/null | grep -Fxq "/${node}/change_state"; then
      available=true
      break
    fi
    sleep 0.2
  done

  if [[ "${available}" != true ]]; then
    echo "Lifecycle service not available for /${node}" >&2
    exit 4
  fi
done

echo "Configuring map_server..."
ros2 lifecycle set /map_server configure
echo "Activating map_server..."
ros2 lifecycle set /map_server activate

echo "Configuring AMCL..."
ros2 lifecycle set /amcl configure
echo "Activating AMCL..."
ros2 lifecycle set /amcl activate

echo
echo "AMCL validation bringup is active."
echo
echo "Useful checks:"
echo "  ros2 lifecycle get /map_server"
echo "  ros2 lifecycle get /amcl"
echo "  ros2 topic info /particle_cloud --verbose"
echo "  ros2 topic echo /particle_cloud --once"
echo "  python3 scripts/validate_amcl_particle_cloud.py --topic /particle_cloud --expected-frame map --samples 5 --timeout 30"
echo
echo "If the cloud does not update while stationary:"
echo "  ros2 service call /request_nomotion_update std_srvs/srv/Empty '{}'"
echo
echo "Press Ctrl-C here to stop map_server and AMCL."

wait
