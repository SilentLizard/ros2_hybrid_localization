#!/usr/bin/env python3

import math

import omni.graph.core as og
import omni.kit.app
import omni.usd
from pxr import Sdf


# ---------------------------------------------------------------------------
# Scene configuration
# ---------------------------------------------------------------------------

ROBOT_PRIM = "/World/turtlebot3_burger"
BASE_LINK_PRIM = f"{ROBOT_PRIM}/base_link"
BASE_SCAN_PRIM = f"{ROBOT_PRIM}/base_scan"
CASTER_LINK_PRIM = f"{ROBOT_PRIM}/caster_back_link"
LEFT_WHEEL_LINK_PRIM = f"{ROBOT_PRIM}/wheel_left_link"
RIGHT_WHEEL_LINK_PRIM = f"{ROBOT_PRIM}/wheel_right_link"

# The imported SICK asset is normally an Xform containing the actual OmniLidar.
LIDAR_SENSOR_ROOT = f"{BASE_SCAN_PRIM}/microScan3_sensor"
PREFERRED_LIDAR_PRIM = f"{LIDAR_SENSOR_ROOT}/Lidar"

# The LaserScan data is emitted in the intrinsic lidar coordinate system.
# A dedicated ROS frame rotates that data 180 degrees relative to base_scan.
SCAN_FRAME_ID = "lidar_frame"
SCAN_FRAME_PARENT = "base_scan"
SCAN_FRAME_YAW_DEGREES = 180.0

DRIVE_GRAPH = "/World/ROS_Drive_Graph"
CLOCK_GRAPH = "/World/ROS_Clock"
ODOM_TF_GRAPH = "/World/ROS_OdomTF"
LIDAR_GRAPH = "/World/ROS_Lidar"

LEFT_WHEEL_JOINT = "wheel_left_joint"
RIGHT_WHEEL_JOINT = "wheel_right_joint"

WHEEL_RADIUS = 0.025
WHEEL_DISTANCE = 0.160

MAX_LINEAR_SPEED = 0.22
MAX_ANGULAR_SPEED = 1.0


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------

def get_stage():
    stage = omni.usd.get_context().get_stage()

    if stage is None:
        raise RuntimeError("No USD stage is currently open.")

    return stage


def require_prim(stage, prim_path: str):
    prim = stage.GetPrimAtPath(prim_path)

    if not prim or not prim.IsValid():
        raise RuntimeError(f"Required prim does not exist: {prim_path}")

    return prim


def remove_local_graph(stage, graph_path: str) -> None:
    prim = stage.GetPrimAtPath(graph_path)

    if not prim or not prim.IsValid():
        return

    prim_stack = prim.GetPrimStack()

    if not prim_stack:
        raise RuntimeError(
            f"Cannot determine the authoring layer for {graph_path}."
        )

    current_layer = stage.GetEditTarget().GetLayer()
    authored_locally = any(
        specification.layer == current_layer
        for specification in prim_stack
    )

    if not authored_locally:
        raise RuntimeError(
            f"{graph_path} is ancestral or referenced and cannot be "
            "safely replaced in the current edit layer."
        )

    stage.RemovePrim(graph_path)


def enable_extensions() -> None:
    manager = omni.kit.app.get_app().get_extension_manager()

    extensions = (
        "isaacsim.core.nodes",
        "isaacsim.ros2.nodes",
        "isaacsim.ros2.bridge",
        "isaacsim.robot.wheeled_robots",
    )

    for extension in extensions:
        manager.set_extension_enabled_immediate(extension, True)


def find_omni_lidar(stage) -> str:
    preferred = stage.GetPrimAtPath(PREFERRED_LIDAR_PRIM)

    if (
        preferred
        and preferred.IsValid()
        and preferred.GetTypeName() == "OmniLidar"
    ):
        return PREFERRED_LIDAR_PRIM

    root = require_prim(stage, LIDAR_SENSOR_ROOT)

    for prim in stage.Traverse():
        prim_path = str(prim.GetPath())

        if not prim_path.startswith(str(root.GetPath()) + "/"):
            continue

        if prim.GetTypeName() == "OmniLidar":
            return prim_path

    raise RuntimeError(
        "No OmniLidar prim was found below "
        f"{LIDAR_SENSOR_ROOT}. The visual scanner mesh is not enough; "
        "create/import an RTX lidar sensor first."
    )


def graph_attribute(path: str):
    attribute = og.Controller.attribute(path)

    if attribute is None or not attribute.is_valid():
        raise RuntimeError(f"Graph attribute does not exist: {path}")

    return attribute


# ---------------------------------------------------------------------------
# Drive graph
# ---------------------------------------------------------------------------

def create_drive_graph(stage) -> None:
    remove_local_graph(stage, DRIVE_GRAPH)

    keys = og.Controller.Keys

    og.Controller.edit(
        {
            "graph_path": DRIVE_GRAPH,
            "evaluator_name": "execution",
        },
        {
            keys.CREATE_NODES: [
                (
                    "OnPlaybackTick",
                    "omni.graph.action.OnPlaybackTick",
                ),
                (
                    "ROS2Context",
                    "isaacsim.ros2.bridge.ROS2Context",
                ),
                (
                    "SubscribeTwist",
                    "isaacsim.ros2.bridge.ROS2SubscribeTwist",
                ),
                (
                    "BreakLinearVelocity",
                    "omni.graph.nodes.BreakVector3",
                ),
                (
                    "BreakAngularVelocity",
                    "omni.graph.nodes.BreakVector3",
                ),
                (
                    "DifferentialController",
                    "isaacsim.robot.wheeled_robots.DifferentialController",
                ),
                (
                    "ArticulationController",
                    "isaacsim.core.nodes.IsaacArticulationController",
                ),
            ],

            keys.SET_VALUES: [
                (
                    "SubscribeTwist.inputs:topicName",
                    "cmd_vel",
                ),
                (
                    "SubscribeTwist.inputs:queueSize",
                    10,
                ),
                (
                    "DifferentialController.inputs:wheelRadius",
                    WHEEL_RADIUS,
                ),
                (
                    "DifferentialController.inputs:wheelDistance",
                    WHEEL_DISTANCE,
                ),
                (
                    "DifferentialController.inputs:maxLinearSpeed",
                    MAX_LINEAR_SPEED,
                ),
                (
                    "DifferentialController.inputs:maxAngularSpeed",
                    MAX_ANGULAR_SPEED,
                ),
                (
                    "ArticulationController.inputs:jointNames",
                    [
                        LEFT_WHEEL_JOINT,
                        RIGHT_WHEEL_JOINT,
                    ],
                ),
                (
                    "ArticulationController.inputs:targetPrim",
                    [Sdf.Path(ROBOT_PRIM)],
                ),
            ],

            keys.CONNECT: [
                (
                    "OnPlaybackTick.outputs:tick",
                    "SubscribeTwist.inputs:execIn",
                ),
                (
                    "ROS2Context.outputs:context",
                    "SubscribeTwist.inputs:context",
                ),
                (
                    "SubscribeTwist.outputs:linearVelocity",
                    "BreakLinearVelocity.inputs:tuple",
                ),
                (
                    "SubscribeTwist.outputs:angularVelocity",
                    "BreakAngularVelocity.inputs:tuple",
                ),
                (
                    "BreakLinearVelocity.outputs:x",
                    "DifferentialController.inputs:linearVelocity",
                ),
                (
                    "BreakAngularVelocity.outputs:z",
                    "DifferentialController.inputs:angularVelocity",
                ),
                (
                    "OnPlaybackTick.outputs:deltaSeconds",
                    "DifferentialController.inputs:dt",
                ),
                (
                    "SubscribeTwist.outputs:execOut",
                    "DifferentialController.inputs:execIn",
                ),
                (
                    "DifferentialController.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                (
                    "OnPlaybackTick.outputs:tick",
                    "ArticulationController.inputs:execIn",
                ),
            ],
        },
    )


# ---------------------------------------------------------------------------
# Clock graph
# ---------------------------------------------------------------------------

def create_clock_graph(stage) -> None:
    remove_local_graph(stage, CLOCK_GRAPH)

    keys = og.Controller.Keys

    og.Controller.edit(
        {
            "graph_path": CLOCK_GRAPH,
            "evaluator_name": "execution",
        },
        {
            keys.CREATE_NODES: [
                (
                    "OnPlaybackTick",
                    "omni.graph.action.OnPlaybackTick",
                ),
                (
                    "ROS2Context",
                    "isaacsim.ros2.bridge.ROS2Context",
                ),
                (
                    "ReadSimulationTime",
                    "isaacsim.core.nodes.IsaacReadSimulationTime",
                ),
                (
                    "PublishClock",
                    "isaacsim.ros2.bridge.ROS2PublishClock",
                ),
            ],

            keys.SET_VALUES: [
                (
                    "PublishClock.inputs:topicName",
                    "clock",
                ),
            ],

            keys.CONNECT: [
                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishClock.inputs:execIn",
                ),
                (
                    "ROS2Context.outputs:context",
                    "PublishClock.inputs:context",
                ),
                (
                    "ReadSimulationTime.outputs:simulationTime",
                    "PublishClock.inputs:timeStamp",
                ),
            ],
        },
    )


# ---------------------------------------------------------------------------
# Odometry and TF graph
# ---------------------------------------------------------------------------

def create_odometry_tf_graph(stage) -> None:
    remove_local_graph(stage, ODOM_TF_GRAPH)

    keys = og.Controller.Keys

    robot_tf_targets = [
        Sdf.Path(BASE_SCAN_PRIM),
        Sdf.Path(CASTER_LINK_PRIM),
        Sdf.Path(LEFT_WHEEL_LINK_PRIM),
        Sdf.Path(RIGHT_WHEEL_LINK_PRIM),
    ]

    yaw_radians = math.radians(SCAN_FRAME_YAW_DEGREES)
    lidar_frame_rotation_ijkr = [
        0.0,
        0.0,
        math.sin(yaw_radians / 2.0),
        math.cos(yaw_radians / 2.0),
    ]

    og.Controller.edit(
        {
            "graph_path": ODOM_TF_GRAPH,
            "evaluator_name": "execution",
        },
        {
            keys.CREATE_NODES: [
                (
                    "OnPlaybackTick",
                    "omni.graph.action.OnPlaybackTick",
                ),
                (
                    "ROS2Context",
                    "isaacsim.ros2.bridge.ROS2Context",
                ),
                (
                    "ReadSimulationTime",
                    "isaacsim.core.nodes.IsaacReadSimulationTime",
                ),
                (
                    "ComputeOdometry",
                    "isaacsim.core.nodes.IsaacComputeOdometry",
                ),
                (
                    "PublishOdometry",
                    "isaacsim.ros2.bridge.ROS2PublishOdometry",
                ),
                (
                    "PublishOdomTransform",
                    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
                ),
                (
                    "ComputeRobotTransformTree",
                    "isaacsim.core.nodes.IsaacComputeTransformTree",
                ),
                (
                    "PublishRobotTransformTree",
                    "isaacsim.ros2.bridge.ROS2PublishTransformTree",
                ),
                (
                    "PublishLidarStaticTransform",
                    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
                ),
            ],

            keys.SET_VALUES: [
                (
                    "ComputeOdometry.inputs:chassisPrim",
                    [Sdf.Path(ROBOT_PRIM)],
                ),
                (
                    "PublishOdometry.inputs:topicName",
                    "odom",
                ),
                (
                    "PublishOdometry.inputs:odomFrameId",
                    "odom",
                ),
                (
                    "PublishOdometry.inputs:chassisFrameId",
                    "base_link",
                ),
                (
                    "PublishOdomTransform.inputs:topicName",
                    "tf",
                ),
                (
                    "PublishOdomTransform.inputs:parentFrameId",
                    "odom",
                ),
                (
                    "PublishOdomTransform.inputs:childFrameId",
                    "base_link",
                ),
                (
                    "PublishOdomTransform.inputs:staticPublisher",
                    False,
                ),
                (
                    "ComputeRobotTransformTree.inputs:parentPrim",
                    Sdf.Path(BASE_LINK_PRIM),
                ),
                (
                    "ComputeRobotTransformTree.inputs:targetPrims",
                    robot_tf_targets,
                ),
                (
                    "PublishRobotTransformTree.inputs:topicName",
                    "tf",
                ),
                (
                    "PublishRobotTransformTree.inputs:staticPublisher",
                    False,
                ),

                # Dedicated scan frame. The lidar helper labels scan-local data
                # with lidar_frame and this static TF applies the required 180°
                # yaw relative to base_scan.
                (
                    "PublishLidarStaticTransform.inputs:topicName",
                    "tf_static",
                ),
                (
                    "PublishLidarStaticTransform.inputs:parentFrameId",
                    SCAN_FRAME_PARENT,
                ),
                (
                    "PublishLidarStaticTransform.inputs:childFrameId",
                    SCAN_FRAME_ID,
                ),
                (
                    "PublishLidarStaticTransform.inputs:translation",
                    [0.0, 0.0, 0.0],
                ),
                (
                    "PublishLidarStaticTransform.inputs:rotation",
                    lidar_frame_rotation_ijkr,
                ),
                (
                    "PublishLidarStaticTransform.inputs:staticPublisher",
                    True,
                ),
            ],

            keys.CONNECT: [
                (
                    "OnPlaybackTick.outputs:tick",
                    "ComputeOdometry.inputs:execIn",
                ),

                (
                    "ComputeOdometry.outputs:execOut",
                    "PublishOdometry.inputs:execIn",
                ),
                (
                    "ComputeOdometry.outputs:position",
                    "PublishOdometry.inputs:position",
                ),
                (
                    "ComputeOdometry.outputs:orientation",
                    "PublishOdometry.inputs:orientation",
                ),
                (
                    "ComputeOdometry.outputs:linearVelocity",
                    "PublishOdometry.inputs:linearVelocity",
                ),
                (
                    "ComputeOdometry.outputs:angularVelocity",
                    "PublishOdometry.inputs:angularVelocity",
                ),
                (
                    "ReadSimulationTime.outputs:simulationTime",
                    "PublishOdometry.inputs:timeStamp",
                ),
                (
                    "ROS2Context.outputs:context",
                    "PublishOdometry.inputs:context",
                ),

                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishOdomTransform.inputs:execIn",
                ),
                (
                    "ComputeOdometry.outputs:position",
                    "PublishOdomTransform.inputs:translation",
                ),
                (
                    "ComputeOdometry.outputs:orientation",
                    "PublishOdomTransform.inputs:rotation",
                ),
                (
                    "ReadSimulationTime.outputs:simulationTime",
                    "PublishOdomTransform.inputs:timeStamp",
                ),
                (
                    "ROS2Context.outputs:context",
                    "PublishOdomTransform.inputs:context",
                ),

                (
                    "OnPlaybackTick.outputs:tick",
                    "ComputeRobotTransformTree.inputs:execIn",
                ),
                (
                    "ComputeRobotTransformTree.outputs:execOut",
                    "PublishRobotTransformTree.inputs:execIn",
                ),
                (
                    "ComputeRobotTransformTree.outputs:parentFrames",
                    "PublishRobotTransformTree.inputs:parentFrames",
                ),
                (
                    "ComputeRobotTransformTree.outputs:childFrames",
                    "PublishRobotTransformTree.inputs:childFrames",
                ),
                (
                    "ComputeRobotTransformTree.outputs:translations",
                    "PublishRobotTransformTree.inputs:translations",
                ),
                (
                    "ComputeRobotTransformTree.outputs:orientations",
                    "PublishRobotTransformTree.inputs:orientations",
                ),
                (
                    "ReadSimulationTime.outputs:simulationTime",
                    "PublishRobotTransformTree.inputs:timeStamp",
                ),
                (
                    "ROS2Context.outputs:context",
                    "PublishRobotTransformTree.inputs:context",
                ),

                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishLidarStaticTransform.inputs:execIn",
                ),
                (
                    "ReadSimulationTime.outputs:simulationTime",
                    "PublishLidarStaticTransform.inputs:timeStamp",
                ),
                (
                    "ROS2Context.outputs:context",
                    "PublishLidarStaticTransform.inputs:context",
                ),
            ],
        },
    )


# ---------------------------------------------------------------------------
# RTX lidar graph
# ---------------------------------------------------------------------------

def create_lidar_graph(stage, lidar_prim_path: str) -> None:
    lidar_prim = require_prim(stage, lidar_prim_path)

    if lidar_prim.GetTypeName() != "OmniLidar":
        raise RuntimeError(
            f"{lidar_prim_path} is {lidar_prim.GetTypeName()}, not OmniLidar."
        )

    remove_local_graph(stage, LIDAR_GRAPH)

    keys = og.Controller.Keys

    og.Controller.edit(
        {
            "graph_path": LIDAR_GRAPH,
            "evaluator_name": "execution",
        },
        {
            keys.CREATE_NODES: [
                (
                    "OnPlaybackTick",
                    "omni.graph.action.OnPlaybackTick",
                ),
                (
                    "ROS2Context",
                    "isaacsim.ros2.bridge.ROS2Context",
                ),
                (
                    "CreateRenderProduct",
                    "isaacsim.core.nodes.IsaacCreateRenderProduct",
                ),
                (
                    "PublishLaserScan",
                    "isaacsim.ros2.bridge.ROS2RtxLidarHelper",
                ),
            ],

            keys.SET_VALUES: [
                (
                    "CreateRenderProduct.inputs:cameraPrim",
                    [Sdf.Path(lidar_prim_path)],
                ),
                (
                    "PublishLaserScan.inputs:topicName",
                    "scan",
                ),
                (
                    "PublishLaserScan.inputs:frameId",
                    SCAN_FRAME_ID,
                ),
                (
                    "PublishLaserScan.inputs:type",
                    "laser_scan",
                ),
                (
                    "PublishLaserScan.inputs:enabled",
                    True,
                ),
                (
                    "PublishLaserScan.inputs:frameSkipCount",
                    0,
                ),
                (
                    "PublishLaserScan.inputs:showDebugView",
                    False,
                ),
            ],

            keys.CONNECT: [
                (
                    "OnPlaybackTick.outputs:tick",
                    "CreateRenderProduct.inputs:execIn",
                ),
                (
                    "CreateRenderProduct.outputs:execOut",
                    "PublishLaserScan.inputs:execIn",
                ),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "PublishLaserScan.inputs:renderProductPath",
                ),
                (
                    "ROS2Context.outputs:context",
                    "PublishLaserScan.inputs:context",
                ),
            ],
        },
    )


def normalize_path_value(value):
    """Normalize OmniGraph path values returned as either a path or 1-item list."""
    if isinstance(value, (list, tuple)):
        return [str(item) for item in value]

    return str(value)


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

def validate_setup(stage, lidar_prim_path: str) -> None:
    required_prims = (
        ROBOT_PRIM,
        BASE_LINK_PRIM,
        BASE_SCAN_PRIM,
        LEFT_WHEEL_LINK_PRIM,
        RIGHT_WHEEL_LINK_PRIM,
        DRIVE_GRAPH,
        CLOCK_GRAPH,
        ODOM_TF_GRAPH,
        LIDAR_GRAPH,
        lidar_prim_path,
    )

    missing = [
        path
        for path in required_prims
        if not stage.GetPrimAtPath(path).IsValid()
    ]

    if missing:
        raise RuntimeError(
            "Setup validation failed; missing prims:\n  "
            + "\n  ".join(missing)
        )

    checks = {
        "drive topic": (
            f"{DRIVE_GRAPH}/SubscribeTwist.inputs:topicName",
            "cmd_vel",
        ),
        "drive robot": (
            f"{DRIVE_GRAPH}/ArticulationController.inputs:targetPrim",
            Sdf.Path(ROBOT_PRIM),
        ),
        "scan topic": (
            f"{LIDAR_GRAPH}/PublishLaserScan.inputs:topicName",
            "scan",
        ),
        "scan frame": (
            f"{LIDAR_GRAPH}/PublishLaserScan.inputs:frameId",
            SCAN_FRAME_ID,
        ),
        "scan debug rendering": (
            f"{LIDAR_GRAPH}/PublishLaserScan.inputs:showDebugView",
            False,
        ),
        "scan helper enabled": (
            f"{LIDAR_GRAPH}/PublishLaserScan.inputs:enabled",
            True,
        ),
        "lidar render target": (
            f"{LIDAR_GRAPH}/CreateRenderProduct.inputs:cameraPrim",
            Sdf.Path(lidar_prim_path),
        ),
        "clock topic": (
            f"{CLOCK_GRAPH}/PublishClock.inputs:topicName",
            "clock",
        ),
        "odom topic": (
            f"{ODOM_TF_GRAPH}/PublishOdometry.inputs:topicName",
            "odom",
        ),
        "lidar TF parent": (
            f"{ODOM_TF_GRAPH}/PublishLidarStaticTransform.inputs:parentFrameId",
            SCAN_FRAME_PARENT,
        ),
        "lidar TF child": (
            f"{ODOM_TF_GRAPH}/PublishLidarStaticTransform.inputs:childFrameId",
            SCAN_FRAME_ID,
        ),
    }

    failures = []

    path_value_labels = {
        "drive robot",
        "lidar render target",
    }

    for label, (attribute_path, expected) in checks.items():
        actual = graph_attribute(attribute_path).get()

        if label in path_value_labels:
            actual_paths = normalize_path_value(actual)
            expected_paths = normalize_path_value(expected)

            # Isaac Sim may return a one-target relationship as either a
            # scalar Sdf.Path or a one-item list depending on node/version.
            if actual_paths != expected_paths:
                failures.append(
                    f"{label}: expected {expected_paths!r}, "
                    f"got {actual_paths!r}"
                )
        elif actual != expected:
            failures.append(
                f"{label}: expected {expected!r}, got {actual!r}"
            )

    point_cloud_node = stage.GetPrimAtPath(
        f"{LIDAR_GRAPH}/PublishPointCloud"
    )

    if point_cloud_node and point_cloud_node.IsValid():
        failures.append(
            "diagnostic PublishPointCloud node still exists and may add "
            "render load/flickering"
        )

    if failures:
        raise RuntimeError(
            "Setup validation failed:\n  "
            + "\n  ".join(failures)
        )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    enable_extensions()

    stage = get_stage()

    require_prim(stage, ROBOT_PRIM)
    require_prim(stage, BASE_LINK_PRIM)
    require_prim(stage, BASE_SCAN_PRIM)
    require_prim(stage, LEFT_WHEEL_LINK_PRIM)
    require_prim(stage, RIGHT_WHEEL_LINK_PRIM)
    require_prim(stage, LIDAR_SENSOR_ROOT)

    lidar_prim_path = find_omni_lidar(stage)

    create_drive_graph(stage)
    create_clock_graph(stage)
    create_odometry_tf_graph(stage)
    create_lidar_graph(stage, lidar_prim_path)

    validate_setup(stage, lidar_prim_path)

    print("")
    print("ROS navigation graphs created and validated.")
    print(f"  Drive graph:     {DRIVE_GRAPH}")
    print(f"  Clock graph:     {CLOCK_GRAPH}")
    print(f"  Odometry/TF:     {ODOM_TF_GRAPH}")
    print(f"  Lidar graph:     {LIDAR_GRAPH}")
    print(f"  OmniLidar prim:  {lidar_prim_path}")
    print(f"  Scan frame:      {SCAN_FRAME_ID}")
    print(
        "  Static scan TF:  "
        f"{SCAN_FRAME_PARENT} -> {SCAN_FRAME_ID} "
        f"(yaw {SCAN_FRAME_YAW_DEGREES:.1f} deg)"
    )
    print("  Debug rendering: disabled")
    print("  Lidar execution: direct playback tick (no RunOneSimulationFrame)")
    print("  Point cloud:     not created")
    print("")
    print("Expected ROS topics after pressing Play:")
    print("  /cmd_vel")
    print("  /clock")
    print("  /odom")
    print("  /tf")
    print("  /tf_static")
    print("  /scan")
    print("")
    print("Verification commands:")
    print("  ros2 topic echo /scan --once")
    print("  ros2 topic hz /scan")
    print("  ros2 run tf2_ros tf2_echo odom lidar_frame")


main()
