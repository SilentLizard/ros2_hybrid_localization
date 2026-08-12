#!/usr/bin/env bash
set -eo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  source /opt/ros/jazzy/setup.bash
fi

if [[ -f "$HOME/.ros/isaac_fastdds_env.sh" ]]; then
  source "$HOME/.ros/isaac_fastdds_env.sh"
fi
set -u

JOYSTICK_DEVICE="${JOYSTICK_DEVICE:-/dev/input/js0}"
LINEAR_AXIS="${LINEAR_AXIS:-1}"
ANGULAR_AXIS="${ANGULAR_AXIS:-0}"
LINEAR_SCALE="${LINEAR_SCALE:-0.8}"
ANGULAR_SCALE="${ANGULAR_SCALE:-2.0}"
AUTOREPEAT_RATE="${AUTOREPEAT_RATE:-20.0}"
TURN_CREEP="${TURN_CREEP:-0.12}"
LINEAR_DEADBAND="${LINEAR_DEADBAND:-0.04}"
ANGULAR_DEADBAND="${ANGULAR_DEADBAND:-0.08}"
LOW_SPEED_TURN_BOOST="${LOW_SPEED_TURN_BOOST:-2.6}"
LOW_SPEED_THRESHOLD="${LOW_SPEED_THRESHOLD:-0.20}"
MOVING_TURN_BOOST="${MOVING_TURN_BOOST:-2.35}"
MAX_ANGULAR_CMD="${MAX_ANGULAR_CMD:-4.0}"

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
  -p scale_angular.yaw:="$ANGULAR_SCALE" \
  -r cmd_vel:=cmd_vel_raw &
PIDS+=("$!")


python3 - "$TURN_CREEP" "$LINEAR_DEADBAND" "$ANGULAR_DEADBAND" "$LOW_SPEED_TURN_BOOST" "$LOW_SPEED_THRESHOLD" "$MOVING_TURN_BOOST" "$MAX_ANGULAR_CMD" <<'PY' &
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


TURN_CREEP = float(sys.argv[1])
LINEAR_DEADBAND = float(sys.argv[2])
ANGULAR_DEADBAND = float(sys.argv[3])
LOW_SPEED_TURN_BOOST = float(sys.argv[4])
LOW_SPEED_THRESHOLD = float(sys.argv[5])
MOVING_TURN_BOOST = float(sys.argv[6])
MAX_ANGULAR_CMD = float(sys.argv[7])


class HerosTwistShaper(Node):
    def __init__(self):
        super().__init__("heros_twist_shaper")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(
            Twist,
            "/cmd_vel_raw",
            self.callback,
            10,
        )

    def callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        if abs(linear) < LINEAR_DEADBAND:
            linear = 0.0

        if abs(angular) < ANGULAR_DEADBAND:
            angular = 0.0

        # Steering should feel car-like while reversing: the same stick
        # direction produces the opposite yaw command when backing up.
        if linear < 0.0:
            angular = -angular

        # Increase steering authority while driving. The low-speed boost is
        # stronger because the HEROS caster/drive geometry resists pivoting.
        if angular != 0.0:
            if abs(linear) < LOW_SPEED_THRESHOLD:
                angular *= LOW_SPEED_TURN_BOOST

                if linear == 0.0:
                    linear = TURN_CREEP
            else:
                angular *= MOVING_TURN_BOOST

        angular = max(-MAX_ANGULAR_CMD, min(MAX_ANGULAR_CMD, angular))

        out = Twist()
        out.linear.x = linear
        out.angular.z = angular
        self.pub.publish(out)


def main():
    rclpy.init()
    node = HerosTwistShaper()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
PY
PIDS+=("$!")

wait -n "${PIDS[@]}"
