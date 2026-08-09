#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
AMCL_PARAM_FILE="${SCRIPT_DIR}/amcl_issue41.yaml"
MAP_YAML="${SCRIPT_DIR}/localization_test_map.yaml"
PARTICLE_ANALYSIS_PARAM_FILE="${WORKSPACE_ROOT}/install/hybrid_localization_ros/share/hybrid_localization_ros/config/particle_analysis.yaml"

source /opt/ros/jazzy/setup.bash
[[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]] && source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

for f in "${MAP_YAML}" "${AMCL_PARAM_FILE}" "${PARTICLE_ANALYSIS_PARAM_FILE}"; do
  [[ -f "$f" ]] || { echo "Required file missing: $f" >&2; exit 2; }
done

PIDS=()
cleanup() {
  set +e
  echo
  echo "Stopping observation-mode processes..."
  for ((i=${#PIDS[@]}-1; i>=0; --i)); do
    kill "${PIDS[i]}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait_for_topic() {
  local topic="$1"
  local attempts="${2:-50}"
  for _ in $(seq 1 "${attempts}"); do
    ros2 topic list 2>/dev/null | grep -Fxq "${topic}" && return 0
    sleep 0.2
  done
  echo "Timed out waiting for topic: ${topic}" >&2
  return 1
}

wait_for_service() {
  local service="$1"
  local attempts="${2:-50}"
  for _ in $(seq 1 "${attempts}"); do
    ros2 service list 2>/dev/null | grep -Fxq "${service}" && return 0
    sleep 0.2
  done
  echo "Timed out waiting for service: ${service}" >&2
  return 1
}

echo "Checking Isaac Sim ROS fixture..."
for topic in /clock /scan /odom /tf /tf_static; do
  wait_for_topic "${topic}" 50 || {
    echo "Start Isaac Sim and its ROS graphs first." >&2
    exit 3
  }
done

echo "Starting map_server..."
ros2 run nav2_map_server map_server   --ros-args   -p use_sim_time:=true   -p yaml_filename:="${MAP_YAML}" &
PIDS+=("$!")

echo "Starting AMCL..."
ros2 run nav2_amcl amcl   --ros-args   --params-file "${AMCL_PARAM_FILE}" &
PIDS+=("$!")

wait_for_service /map_server/change_state 50 || exit 4
wait_for_service /amcl/change_state 50 || exit 4

ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate

wait_for_topic /particle_cloud 100 || exit 5

echo "Starting particle-analysis observer..."
ros2 run hybrid_localization_ros particle_analysis_observer   --ros-args   -p use_sim_time:=true   --params-file "${PARTICLE_ANALYSIS_PARAM_FILE}" &
PIDS+=("$!")

wait_for_topic /hybrid_localization/particle_analysis 100 || exit 6

echo "Starting particle-analysis visualization..."
ros2 run hybrid_localization_ros particle_analysis_visualization   --ros-args   -p use_sim_time:=true &
PIDS+=("$!")

wait_for_topic /hybrid_localization/rviz_markers 100 || exit 7

echo
echo "Observation-mode stack is active."
echo "Order:"
echo "  1. Isaac Sim ROS fixture            [external]"
echo "  2. map_server                       [this script]"
echo "  3. AMCL                             [this script]"
echo "  4. particle_analysis_observer       [this script]"
echo "  5. particle_analysis_visualization  [this script]"
echo "  6. RViz client                      [start now]"
echo
echo "Checks:"
echo "  ros2 lifecycle get /map_server"
echo "  ros2 lifecycle get /amcl"
echo "  ros2 topic hz /hybrid_localization/particle_analysis"
echo "  ros2 topic hz /hybrid_localization/rviz_markers"
echo
echo "If stationary AMCL needs an update:"
echo "  ros2 service call /request_nomotion_update std_srvs/srv/Empty '{}'"
echo
echo "Press Ctrl-C to stop all ROS-side observation processes."

wait
