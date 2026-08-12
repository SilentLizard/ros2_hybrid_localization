#!/usr/bin/env bash
set -eo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  source /opt/ros/jazzy/setup.bash
fi
set -u

JOYSTICK_DEVICE="${JOYSTICK_DEVICE:-/dev/input/js0}"
LINEAR_AXIS="${LINEAR_AXIS:-1}"
ANGULAR_AXIS="${ANGULAR_AXIS:-0}"
LINEAR_SCALE="${LINEAR_SCALE:-0.8}"
ANGULAR_SCALE="${ANGULAR_SCALE:-1.2}"
AUTOREPEAT_RATE="${AUTOREPEAT_RATE:-20.0}"

[[ -r "$JOYSTICK_DEVICE" ]] || {
  echo "Joystick device is unavailable or unreadable: $JOYSTICK_DEVICE" >&2
  exit 1
}

unset ROS_LOCALHOST_ONLY || true
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

PIDS=()
cleanup() {
  set +e
  for ((i=${#PIDS[@]}-1; i>=0; --i)); do kill "${PIDS[i]}" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 run joy_linux joy_linux_node --ros-args \
  -p dev:="$JOYSTICK_DEVICE" \
  -p autorepeat_rate:="$AUTOREPEAT_RATE" \
  -p deadzone:=0.05 &
PIDS+=("$!")

ros2 run teleop_twist_joy teleop_node --ros-args \
  -p require_enable_button:=false \
  -p axis_linear.x:="$LINEAR_AXIS" \
  -p scale_linear.x:="$LINEAR_SCALE" \
  -p axis_angular.yaw:="$ANGULAR_AXIS" \
  -p scale_angular.yaw:="$ANGULAR_SCALE" &
PIDS+=("$!")

wait -n "${PIDS[@]}"
