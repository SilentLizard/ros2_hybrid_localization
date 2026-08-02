#!/usr/bin/env bash
set -eo pipefail

ROS_SETUP="/opt/ros/jazzy/setup.bash"
ISAAC_ROS_ENV="$HOME/.ros/isaac_fastdds_env.sh"

# ROS setup scripts are not safe under `set -u`.
# shellcheck disable=SC1090
source "$ROS_SETUP"

# shellcheck disable=SC1090
source "$ISAAC_ROS_ENV"

# Enable strict unset-variable checking only after sourcing ROS.
set -u

JOYSTICK_DEVICE="${JOYSTICK_DEVICE:-/dev/input/js0}"

# Controller mappings. Override these as environment variables when needed.
LINEAR_AXIS="${LINEAR_AXIS:-1}"
ANGULAR_AXIS="${ANGULAR_AXIS:-0}"
LINEAR_SCALE="${LINEAR_SCALE:--0.5}"
ANGULAR_SCALE="${ANGULAR_SCALE:-0.8}"
AUTOREPEAT_RATE="${AUTOREPEAT_RATE:-20.0}"

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ERROR: ROS 2 Jazzy setup not found at: $ROS_SETUP" >&2
    exit 1
fi

if [[ ! -f "$ISAAC_ROS_ENV" ]]; then
    echo "ERROR: Isaac ROS environment file not found at: $ISAAC_ROS_ENV" >&2
    exit 1
fi

if [[ ! -e "$JOYSTICK_DEVICE" ]]; then
    echo "ERROR: Joystick device not found: $JOYSTICK_DEVICE" >&2
    echo "Available joystick devices:" >&2
    ls -l /dev/input/js* 2>/dev/null || true
    exit 1
fi

if [[ ! -r "$JOYSTICK_DEVICE" ]]; then
    echo "ERROR: Joystick device is not readable: $JOYSTICK_DEVICE" >&2
    echo "Current permissions:" >&2
    ls -l "$JOYSTICK_DEVICE" >&2
    echo >&2
    echo "Add your user to the input group, then log out and back in:" >&2
    echo "  sudo usermod -aG input \$USER" >&2
    exit 1
fi

# ROS_LOCALHOST_ONLY is deprecated in Jazzy. The Fast DDS profile remains active.
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

cleanup() {
    trap - EXIT INT TERM

    echo
    echo "Stopping gamepad drive nodes..."

    if [[ -n "${TELEOP_PID:-}" ]]; then
        kill "$TELEOP_PID" 2>/dev/null || true
    fi

    if [[ -n "${JOY_PID:-}" ]]; then
        kill "$JOY_PID" 2>/dev/null || true
    fi

    wait 2>/dev/null || true
}

trap cleanup EXIT INT TERM

echo "Starting Isaac Sim gamepad drive"
echo "  Device:          $JOYSTICK_DEVICE"
echo "  Linear axis:     $LINEAR_AXIS"
echo "  Angular axis:    $ANGULAR_AXIS"
echo "  Linear scale:    $LINEAR_SCALE"
echo "  Angular scale:   $ANGULAR_SCALE"
echo "  ROS domain:      ${ROS_DOMAIN_ID:-0}"
echo "  ROS middleware:  ${RMW_IMPLEMENTATION:-default}"
echo
echo "Publishing:"
echo "  $JOYSTICK_DEVICE -> /joy -> /cmd_vel"
echo

ros2 run joy_linux joy_linux_node --ros-args \
    -p dev:="$JOYSTICK_DEVICE" \
    -p autorepeat_rate:="$AUTOREPEAT_RATE" \
    -p deadzone:=0.05 &

JOY_PID=$!

sleep 1

if ! kill -0 "$JOY_PID" 2>/dev/null; then
    echo "ERROR: joy_linux_node stopped during startup." >&2
    wait "$JOY_PID"
    exit 1
fi

ros2 run teleop_twist_joy teleop_node --ros-args \
    -p require_enable_button:=false \
    -p axis_linear.x:="$LINEAR_AXIS" \
    -p scale_linear.x:="$LINEAR_SCALE" \
    -p axis_angular.yaw:="$ANGULAR_AXIS" \
    -p scale_angular.yaw:="$ANGULAR_SCALE" &

TELEOP_PID=$!

sleep 1

if ! kill -0 "$TELEOP_PID" 2>/dev/null; then
    echo "ERROR: teleop_twist_joy stopped during startup." >&2
    wait "$TELEOP_PID"
    exit 1
fi

echo "Gamepad drive is running. Press Ctrl+C to stop."
echo

# Exit if either process unexpectedly terminates.
wait -n "$JOY_PID" "$TELEOP_PID"

echo "ERROR: One of the ROS nodes stopped unexpectedly." >&2
exit 1