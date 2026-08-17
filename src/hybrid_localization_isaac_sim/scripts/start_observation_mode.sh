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

for file in \
  "$MAP_YAML" \
  "$AMCL_PARAM_FILE" \
  "$PARTICLE_ANALYSIS_PARAM_FILE"
do
  if [[ ! -f "$file" ]]; then
    echo "Required file missing: $file" >&2
    exit 2
  fi
done

PIDS=()
CLEANUP_RUNNING=0

node_exists() {
  local node="$1"

  ros2 node list 2>/dev/null |
    grep -Fxq "$node"
}

wait_for_topic() {
  local topic="$1"
  local attempts="${2:-75}"

  for _ in $(seq 1 "$attempts"); do
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      return 0
    fi

    sleep 0.2
  done

  echo "Timed out waiting for topic: $topic" >&2
  return 1
}

wait_for_service() {
  local service="$1"
  local attempts="${2:-75}"

  for _ in $(seq 1 "$attempts"); do
    if ros2 service list 2>/dev/null | grep -Fxq "$service"; then
      return 0
    fi

    sleep 0.2
  done

  echo "Timed out waiting for service: $service" >&2
  return 1
}

lifecycle_state() {
  local node="$1"

  ros2 lifecycle get "$node" 2>/dev/null |
    awk 'NR==1 {print $1}'
}

wait_for_lifecycle_state() {
  local node="$1"
  local expected="$2"
  local attempts="${3:-75}"
  local state=""

  for _ in $(seq 1 "$attempts"); do
    state="$(lifecycle_state "$node" || true)"

    if [[ "$state" == "$expected" ]]; then
      return 0
    fi

    sleep 0.2
  done

  echo \
    "Timed out waiting for lifecycle state '$expected' on $node; last state: ${state:-unknown}" \
    >&2

  return 1
}

transition_lifecycle() {
  local node="$1"
  local transition="$2"
  local expected="$3"
  local before=""

  before="$(lifecycle_state "$node" || true)"

  if [[ -z "$before" ]]; then
    echo \
      "Could not read lifecycle state for $node before '$transition'." \
      >&2
    return 1
  fi

  if ! ros2 lifecycle set "$node" "$transition"; then
    echo \
      "Lifecycle transition '$transition' failed for $node (state was '$before')." \
      >&2
    return 1
  fi

  if ! wait_for_lifecycle_state "$node" "$expected" 50; then
    echo \
      "Lifecycle transition '$transition' on $node did not reach '$expected'." \
      >&2
    return 1
  fi
}

stop_existing_observation_stack() {
  local deadline
  local found
  local node

  found=0

  for node in \
    /map_server \
    /amcl \
    /particle_analysis_observer \
    /particle_analysis_visualization
  do
    if node_exists "$node"; then
      found=1
      break
    fi
  done

  if (( found == 0 )); then
    return 0
  fi

  echo "Existing observation-mode stack detected."
  echo "Stopping stale observation-mode processes..."

  pkill -TERM -f 'nav2_map_server/map_server' || true
  pkill -TERM -f 'nav2_amcl/amcl' || true
  pkill -TERM -f 'particle_analysis_observer' || true
  pkill -TERM -f 'particle_analysis_visualization' || true

  deadline=$((SECONDS + 3))

  while (( SECONDS < deadline )); do
    found=0

    for node in \
      /map_server \
      /amcl \
      /particle_analysis_observer \
      /particle_analysis_visualization
    do
      if node_exists "$node"; then
        found=1
        break
      fi
    done

    if (( found == 0 )); then
      echo "Previous observation-mode stack stopped."
      return 0
    fi

    sleep 0.1
  done

  echo \
    "Observation-mode nodes survived SIGTERM; forcing shutdown..." \
    >&2

  pkill -KILL -f 'nav2_map_server/map_server' || true
  pkill -KILL -f 'nav2_amcl/amcl' || true
  pkill -KILL -f 'particle_analysis_observer' || true
  pkill -KILL -f 'particle_analysis_visualization' || true

  sleep 0.5

  for node in \
    /map_server \
    /amcl \
    /particle_analysis_observer \
    /particle_analysis_visualization
  do
    if node_exists "$node"; then
      echo \
        "ERROR: $node still exists after forced cleanup." \
        >&2
      return 1
    fi
  done

  echo "Previous observation-mode stack forcibly stopped."
}

cleanup() {
  local pid
  local deadline
  local any_alive

  if (( CLEANUP_RUNNING )); then
    return
  fi

  CLEANUP_RUNNING=1
  set +e

  if ((${#PIDS[@]} == 0)); then
    return
  fi

  echo
  echo "Stopping observation-mode processes..."

  # Stop in reverse startup order.
  for ((i=${#PIDS[@]}-1; i>=0; --i)); do
    pid="${PIDS[i]}"

    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done

  # Give processes a bounded grace period.
  deadline=$((SECONDS + 3))

  while (( SECONDS < deadline )); do
    any_alive=0

    for pid in "${PIDS[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        any_alive=1
        break
      fi
    done

    if (( any_alive == 0 )); then
      break
    fi

    sleep 0.1
  done

  # Escalate only for processes started by this script.
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      echo \
        "Process $pid did not stop after SIGTERM; sending SIGKILL." \
        >&2

      kill -KILL "$pid" 2>/dev/null || true
    fi
  done

  wait 2>/dev/null || true
}

handle_interrupt() {
  trap - INT TERM
  exit 130
}

handle_terminate() {
  trap - INT TERM
  exit 143
}

trap cleanup EXIT
trap handle_interrupt INT
trap handle_terminate TERM

echo "Checking HEROS Isaac ROS fixture..."

for topic in \
  /clock \
  /scan \
  /odom \
  /tf \
  /tf_static
do
  if ! wait_for_topic "$topic" 75; then
    echo \
      "Start Isaac Sim, build the HEROS world/graphs, and press Play first." \
      >&2
    exit 3
  fi
done

stop_existing_observation_stack || exit 4

echo "Starting HEROS map_server: $MAP_YAML"

ros2 run nav2_map_server map_server \
  --ros-args \
  -p use_sim_time:=true \
  -p yaml_filename:="$MAP_YAML" &

PIDS+=("$!")

echo "Starting HEROS AMCL: $AMCL_PARAM_FILE"

ros2 run nav2_amcl amcl \
  --ros-args \
  --params-file "$AMCL_PARAM_FILE" &

PIDS+=("$!")

wait_for_service /map_server/change_state || exit 4
wait_for_service /amcl/change_state || exit 4

wait_for_lifecycle_state /map_server unconfigured 75 || exit 4
wait_for_lifecycle_state /amcl unconfigured 75 || exit 4

transition_lifecycle \
  /map_server \
  configure \
  inactive || exit 4

transition_lifecycle \
  /map_server \
  activate \
  active || exit 4

transition_lifecycle \
  /amcl \
  configure \
  inactive || exit 4

transition_lifecycle \
  /amcl \
  activate \
  active || exit 4

case "$AMCL_INIT_MODE" in
  known)
    echo "AMCL initialization: configured known initial pose"
    ;;

  global)
    echo "AMCL initialization: global free-space particle distribution"

    ros2 run hybrid_localization_isaac_sim \
      reset_amcl_localization.py \
      --mode global || exit 5
    ;;

  random-prior)
    if [[ -z "$AMCL_RANDOM_LAYOUT" ]]; then
      echo \
        "AMCL_RANDOM_LAYOUT is required for AMCL_INIT_MODE=random-prior" \
        >&2
      exit 5
    fi

    echo \
      "AMCL initialization: deterministic random prior (seed=$AMCL_RANDOM_SEED)"

    ros2 run hybrid_localization_isaac_sim \
      reset_amcl_localization.py \
      --mode random-prior \
      --layout "$AMCL_RANDOM_LAYOUT" \
      --seed "$AMCL_RANDOM_SEED" || exit 5
    ;;

  *)
    echo \
      "Invalid AMCL_INIT_MODE=$AMCL_INIT_MODE (known|global|random-prior)" \
      >&2
    exit 5
    ;;
esac

wait_for_topic /map 100 || exit 5
wait_for_topic /particle_cloud 100 || exit 5

echo "Starting particle-analysis observer..."

ros2 run hybrid_localization_ros \
  particle_analysis_observer \
  --ros-args \
  -p use_sim_time:=true \
  --params-file "$PARTICLE_ANALYSIS_PARAM_FILE" &

PIDS+=("$!")

wait_for_topic \
  /hybrid_localization/particle_analysis \
  100 || exit 6

echo "Starting particle-analysis visualization..."

ros2 run hybrid_localization_ros \
  particle_analysis_visualization \
  --ros-args \
  -p use_sim_time:=true &

PIDS+=("$!")

wait_for_topic \
  /hybrid_localization/rviz_markers \
  100 || exit 7

echo
echo "HEROS observation-mode stack is active."
echo "  map:        $MAP_YAML"
echo "  AMCL:       $AMCL_PARAM_FILE"
echo "  analysis:   $PARTICLE_ANALYSIS_PARAM_FILE"
echo "  init mode:  $AMCL_INIT_MODE"
echo
echo "Useful checks:"
echo "  ros2 lifecycle get /map_server"
echo "  ros2 lifecycle get /amcl"
echo "  ros2 topic hz /particle_cloud"
echo "  ros2 topic hz /hybrid_localization/particle_analysis"
echo "  ros2 topic hz /hybrid_localization/rviz_markers"
echo "  ros2 run tf2_ros tf2_echo map base_link"
echo "  ros2 run hybrid_localization_isaac_sim validate_scan_map_alignment.py"
echo
echo "Press Ctrl-C to stop the ROS-side observation stack."

wait