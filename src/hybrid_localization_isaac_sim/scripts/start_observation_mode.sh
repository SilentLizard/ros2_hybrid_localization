#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_PACKAGE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${SOURCE_PACKAGE_ROOT}/../.." && pwd)"

if ! command -v ros2 >/dev/null 2>&1; then
  source /opt/ros/jazzy/setup.bash
fi
if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  source "${WORKSPACE_ROOT}/install/setup.bash"
fi
set -u

ISAAC_PREFIX="$(ros2 pkg prefix hybrid_localization_isaac_sim)"
ROS_PREFIX="$(ros2 pkg prefix hybrid_localization_ros)"
ISAAC_SHARE="${ISAAC_PREFIX}/share/hybrid_localization_isaac_sim"
ROS_SHARE="${ROS_PREFIX}/share/hybrid_localization_ros"

MAP_YAML="${MAP_YAML:-${ISAAC_SHARE}/maps/heros_localization_map.yaml}"
AMCL_PARAM_FILE="${AMCL_PARAM_FILE:-${ISAAC_SHARE}/config/amcl_heros.yaml}"
PARTICLE_ANALYSIS_PARAM_FILE="${PARTICLE_ANALYSIS_PARAM_FILE:-${ROS_SHARE}/config/particle_analysis.yaml}"
AMCL_INIT_MODE="${AMCL_INIT_MODE:-known}"
AMCL_RANDOM_SEED="${AMCL_RANDOM_SEED:-41}"
AMCL_RANDOM_LAYOUT="${AMCL_RANDOM_LAYOUT:-}"

for file in "$MAP_YAML" "$AMCL_PARAM_FILE" "$PARTICLE_ANALYSIS_PARAM_FILE"; do
  [[ -f "$file" ]] || { echo "Required file missing: $file" >&2; exit 2; }
done

PIDS=()
cleanup() {
  set +e
  echo
  echo "Stopping observation-mode processes..."
  for ((i=${#PIDS[@]}-1; i>=0; --i)); do kill "${PIDS[i]}" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait_for_topic() {
  local topic="$1" attempts="${2:-75}"
  for _ in $(seq 1 "$attempts"); do
    ros2 topic list 2>/dev/null | grep -Fxq "$topic" && return 0
    sleep 0.2
  done
  echo "Timed out waiting for topic: $topic" >&2
  return 1
}

wait_for_service() {
  local service="$1" attempts="${2:-75}"
  for _ in $(seq 1 "$attempts"); do
    ros2 service list 2>/dev/null | grep -Fxq "$service" && return 0
    sleep 0.2
  done
  echo "Timed out waiting for service: $service" >&2
  return 1
}

echo "Checking HEROS Isaac ROS fixture..."
for topic in /clock /scan /odom /tf /tf_static; do
  wait_for_topic "$topic" 75 || {
    echo "Start Isaac Sim, build the HEROS world/graphs, and press Play first." >&2
    exit 3
  }
done

echo "Starting HEROS map_server: $MAP_YAML"
ros2 run nav2_map_server map_server --ros-args \
  -p use_sim_time:=true \
  -p yaml_filename:="$MAP_YAML" &
PIDS+=("$!")

echo "Starting HEROS AMCL: $AMCL_PARAM_FILE"
ros2 run nav2_amcl amcl --ros-args --params-file "$AMCL_PARAM_FILE" &
PIDS+=("$!")

wait_for_service /map_server/change_state || exit 4
wait_for_service /amcl/change_state || exit 4
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate

case "$AMCL_INIT_MODE" in
  known)
    echo "AMCL initialization: configured known initial pose"
    ;;
  global)
    echo "AMCL initialization: global free-space particle distribution"
    ros2 run hybrid_localization_isaac_sim reset_amcl_localization.py \
      --mode global || exit 5
    ;;
  random-prior)
    [[ -n "$AMCL_RANDOM_LAYOUT" ]] || {
      echo "AMCL_RANDOM_LAYOUT is required for AMCL_INIT_MODE=random-prior" >&2
      exit 5
    }
    echo "AMCL initialization: deterministic random prior (seed=$AMCL_RANDOM_SEED)"
    ros2 run hybrid_localization_isaac_sim reset_amcl_localization.py \
      --mode random-prior \
      --layout "$AMCL_RANDOM_LAYOUT" \
      --seed "$AMCL_RANDOM_SEED" || exit 5
    ;;
  *)
    echo "Invalid AMCL_INIT_MODE=$AMCL_INIT_MODE (known|global|random-prior)" >&2
    exit 5
    ;;
esac

wait_for_topic /map 100 || exit 5
wait_for_topic /particle_cloud 100 || exit 5

echo "Starting particle-analysis observer..."
ros2 run hybrid_localization_ros particle_analysis_observer --ros-args \
  -p use_sim_time:=true \
  --params-file "$PARTICLE_ANALYSIS_PARAM_FILE" &
PIDS+=("$!")
wait_for_topic /hybrid_localization/particle_analysis 100 || exit 6

echo "Starting particle-analysis visualization..."
ros2 run hybrid_localization_ros particle_analysis_visualization --ros-args \
  -p use_sim_time:=true &
PIDS+=("$!")
wait_for_topic /hybrid_localization/rviz_markers 100 || exit 7

echo
echo "HEROS observation-mode stack is active."
echo "  map:        $MAP_YAML"
echo "  AMCL:       $AMCL_PARAM_FILE"
echo "  analysis:   $PARTICLE_ANALYSIS_PARAM_FILE"
echo
echo "Start the RViz client now."
echo "Useful checks:"
echo "  ros2 topic hz /particle_cloud"
echo "  ros2 topic hz /hybrid_localization/rviz_markers"
echo "  ros2 run tf2_ros tf2_echo map base_link"
echo "  ros2 run hybrid_localization_isaac_sim validate_scan_map_alignment.py"
echo
echo "Press Ctrl-C to stop the ROS-side observation stack."
wait
