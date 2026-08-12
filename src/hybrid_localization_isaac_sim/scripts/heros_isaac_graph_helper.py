#!/usr/bin/env python3
"""Create the ROS 2 navigation graphs for an imported Innok HEROS 3W robot.

Run this from Isaac Sim's Script Editor after importing the HEROS robot and
placing one RTX OmniLidar below the base_scan prim.

Observed Isaac import hierarchy:

    /World/heros_3w
      /Geometry
        /base_link
          /beam_link
          ...
        /base_scan
          /SICK_microScan3
            /Lidar              (OmniLidar)
      /Physics
        /joint_base_wheel_rear_left
        /joint_base_wheel_rear_right
        ...

Only the localization-relevant TF chain is published:
    odom -> base_link -> base_scan -> lidar_frame

Wheel and caster prim transforms are intentionally omitted because the imported
USD may contain duplicate visual/collision prim names and Nav2 localization does
not require those frames.

The HEROS fixture assumes the lidar is already oriented correctly in Isaac.
Therefore base_scan -> lidar_frame adds no additional rotation.
"""
from __future__ import annotations

import omni.graph.core as og
import omni.kit.app
import omni.usd
from pxr import Sdf, UsdPhysics


ROBOT_PRIM = "/World/heros_3w"

BASE_LINK_NAME = "base_link"
BASE_SCAN_NAME = "base_scan"

LEFT_WHEEL_JOINT = "joint_base_wheel_rear_left"
RIGHT_WHEEL_JOINT = "joint_base_wheel_rear_right"

WHEEL_RADIUS = 0.16
WHEEL_DISTANCE = 0.58
MAX_LINEAR_SPEED = 1.0
MAX_ANGULAR_SPEED = 2.0

SCAN_FRAME_PARENT = "base_scan"
SCAN_FRAME_ID = "lidar_frame"

# The SICK asset is mounted/oriented correctly in Isaac. No TurtleBot-derived
# 180 degree correction is applied here.
SCAN_FRAME_TRANSLATION = [0.0, 0.0, 0.0]
SCAN_FRAME_ROTATION = [0.0, 0.0, 0.0, 1.0]

DRIVE_GRAPH = "/World/HEROS_ROS_Drive"
CLOCK_GRAPH = "/World/HEROS_ROS_Clock"
ODOM_TF_GRAPH = "/World/HEROS_ROS_OdomTF"
LIDAR_GRAPH = "/World/HEROS_ROS_Lidar"


def stage():
    current_stage = omni.usd.get_context().get_stage()
    if current_stage is None:
        raise RuntimeError("No USD stage is open")
    return current_stage


def find_unique_prim(current_stage, name, below=ROBOT_PRIM):
    matches = [
        prim
        for prim in current_stage.Traverse()
        if prim.GetName() == name and str(prim.GetPath()).startswith(below + "/")
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected one prim named {name!r} below {below}; "
            f"found {[str(prim.GetPath()) for prim in matches]}"
        )
    return str(matches[0].GetPath())


def find_articulation_root(current_stage):
    matches = [
        prim
        for prim in current_stage.Traverse()
        if str(prim.GetPath()).startswith(ROBOT_PRIM)
        and prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]

    if len(matches) == 1:
        return str(matches[0].GetPath())

    if not matches:
        raise RuntimeError(
            f"No UsdPhysics.ArticulationRootAPI found below {ROBOT_PRIM}. "
            "Check the imported robot's Articulation Root property."
        )

    raise RuntimeError(
        "Expected one articulation root below "
        f"{ROBOT_PRIM}; found {[str(prim.GetPath()) for prim in matches]}"
    )


def validate_joint(current_stage, joint_name):
    path = find_unique_prim(current_stage, joint_name)
    prim = current_stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"Joint prim {path} is invalid")
    return path


def find_lidar(current_stage, below_path):
    matches = [
        prim
        for prim in current_stage.Traverse()
        if prim.GetTypeName() == "OmniLidar"
        and str(prim.GetPath()).startswith(below_path + "/")
    ]

    if len(matches) != 1:
        raise RuntimeError(
            f"Expected exactly one OmniLidar below {below_path}; "
            f"found {[str(prim.GetPath()) for prim in matches]}. "
            "Attach/copy the SICK microScan3 RTX lidar below base_scan first."
        )

    return str(matches[0].GetPath())


def remove_local_graph(current_stage, path):
    prim = current_stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        current_stage.RemovePrim(path)


def enable_extensions():
    manager = omni.kit.app.get_app().get_extension_manager()
    for extension in (
        "isaacsim.core.nodes",
        "isaacsim.ros2.nodes",
        "isaacsim.ros2.bridge",
        "isaacsim.robot.wheeled_robots",
    ):
        manager.set_extension_enabled_immediate(extension, True)


def create_drive(current_stage, articulation_root):
    remove_local_graph(current_stage, DRIVE_GRAPH)
    keys = og.Controller.Keys

    og.Controller.edit(
        {"graph_path": DRIVE_GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("Twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLinear", "omni.graph.nodes.BreakVector3"),
                ("BreakAngular", "omni.graph.nodes.BreakVector3"),
                ("Diff", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("Articulation", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("Twist.inputs:topicName", "cmd_vel"),
                ("Twist.inputs:queueSize", 10),
                ("Diff.inputs:wheelRadius", WHEEL_RADIUS),
                ("Diff.inputs:wheelDistance", WHEEL_DISTANCE),
                ("Diff.inputs:maxLinearSpeed", MAX_LINEAR_SPEED),
                ("Diff.inputs:maxAngularSpeed", MAX_ANGULAR_SPEED),
                (
                    "Articulation.inputs:jointNames",
                    [LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT],
                ),
                (
                    "Articulation.inputs:targetPrim",
                    [Sdf.Path(articulation_root)],
                ),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "Twist.inputs:execIn"),
                ("ROS2Context.outputs:context", "Twist.inputs:context"),
                ("Twist.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
                ("Twist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
                ("BreakLinear.outputs:x", "Diff.inputs:linearVelocity"),
                ("BreakAngular.outputs:z", "Diff.inputs:angularVelocity"),
                ("Tick.outputs:deltaSeconds", "Diff.inputs:dt"),
                ("Twist.outputs:execOut", "Diff.inputs:execIn"),
                (
                    "Diff.outputs:velocityCommand",
                    "Articulation.inputs:velocityCommand",
                ),
                ("Tick.outputs:tick", "Articulation.inputs:execIn"),
            ],
        },
    )


def create_clock(current_stage):
    remove_local_graph(current_stage, CLOCK_GRAPH)
    keys = og.Controller.Keys

    og.Controller.edit(
        {"graph_path": CLOCK_GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("Time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("Clock.inputs:topicName", "clock"),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "Clock.inputs:execIn"),
                ("ROS2Context.outputs:context", "Clock.inputs:context"),
                ("Time.outputs:simulationTime", "Clock.inputs:timeStamp"),
            ],
        },
    )


def create_odom_tf(
    current_stage,
    base_link,
    base_scan,
):
    remove_local_graph(current_stage, ODOM_TF_GRAPH)
    keys = og.Controller.Keys

    og.Controller.edit(
        {"graph_path": ODOM_TF_GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("Time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                (
                    "PubOdomTF",
                    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
                ),
                (
                    "StaticMountTree",
                    "isaacsim.core.nodes.IsaacComputeTransformTree",
                ),
                (
                    "PubStaticMountTree",
                    "isaacsim.ros2.bridge.ROS2PublishTransformTree",
                ),
                (
                    "PubScanTF",
                    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
                ),
            ],
            keys.SET_VALUES: [
                # Odometry must be computed from the actual chassis rigid body,
                # not from the top-level /World/heros_3w Xform.
                ("Odom.inputs:chassisPrim", [Sdf.Path(base_link)]),
                ("PubOdom.inputs:topicName", "odom"),
                ("PubOdom.inputs:odomFrameId", "odom"),
                ("PubOdom.inputs:chassisFrameId", "base_link"),
                ("PubOdomTF.inputs:topicName", "tf"),
                ("PubOdomTF.inputs:parentFrameId", "odom"),
                ("PubOdomTF.inputs:childFrameId", "base_link"),
                ("PubOdomTF.inputs:staticPublisher", False),

                # base_scan is a fixed mount. Publish its actual USD transform
                # once on /tf_static.
                ("StaticMountTree.inputs:parentPrim", Sdf.Path(base_link)),
                (
                    "StaticMountTree.inputs:targetPrims",
                    [Sdf.Path(base_scan)],
                ),
                ("PubStaticMountTree.inputs:topicName", "tf_static"),
                ("PubStaticMountTree.inputs:staticPublisher", True),

                # The SICK asset is already aligned to base_scan in Isaac.
                ("PubScanTF.inputs:topicName", "tf_static"),
                ("PubScanTF.inputs:parentFrameId", SCAN_FRAME_PARENT),
                ("PubScanTF.inputs:childFrameId", SCAN_FRAME_ID),
                (
                    "PubScanTF.inputs:translation",
                    SCAN_FRAME_TRANSLATION,
                ),
                ("PubScanTF.inputs:rotation", SCAN_FRAME_ROTATION),
                ("PubScanTF.inputs:staticPublisher", True),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "Odom.inputs:execIn"),
                ("Odom.outputs:execOut", "PubOdom.inputs:execIn"),
                ("Odom.outputs:position", "PubOdom.inputs:position"),
                ("Odom.outputs:orientation", "PubOdom.inputs:orientation"),
                (
                    "Odom.outputs:linearVelocity",
                    "PubOdom.inputs:linearVelocity",
                ),
                (
                    "Odom.outputs:angularVelocity",
                    "PubOdom.inputs:angularVelocity",
                ),
                (
                    "Time.outputs:simulationTime",
                    "PubOdom.inputs:timeStamp",
                ),
                (
                    "ROS2Context.outputs:context",
                    "PubOdom.inputs:context",
                ),

                ("Tick.outputs:tick", "PubOdomTF.inputs:execIn"),
                (
                    "Odom.outputs:position",
                    "PubOdomTF.inputs:translation",
                ),
                (
                    "Odom.outputs:orientation",
                    "PubOdomTF.inputs:rotation",
                ),
                (
                    "Time.outputs:simulationTime",
                    "PubOdomTF.inputs:timeStamp",
                ),
                (
                    "ROS2Context.outputs:context",
                    "PubOdomTF.inputs:context",
                ),

                ("Tick.outputs:tick", "StaticMountTree.inputs:execIn"),
                (
                    "StaticMountTree.outputs:execOut",
                    "PubStaticMountTree.inputs:execIn",
                ),
                (
                    "StaticMountTree.outputs:parentFrames",
                    "PubStaticMountTree.inputs:parentFrames",
                ),
                (
                    "StaticMountTree.outputs:childFrames",
                    "PubStaticMountTree.inputs:childFrames",
                ),
                (
                    "StaticMountTree.outputs:translations",
                    "PubStaticMountTree.inputs:translations",
                ),
                (
                    "StaticMountTree.outputs:orientations",
                    "PubStaticMountTree.inputs:orientations",
                ),
                (
                    "Time.outputs:simulationTime",
                    "PubStaticMountTree.inputs:timeStamp",
                ),
                (
                    "ROS2Context.outputs:context",
                    "PubStaticMountTree.inputs:context",
                ),

                ("Tick.outputs:tick", "PubScanTF.inputs:execIn"),
                (
                    "Time.outputs:simulationTime",
                    "PubScanTF.inputs:timeStamp",
                ),
                (
                    "ROS2Context.outputs:context",
                    "PubScanTF.inputs:context",
                ),
            ],
        },
    )


def create_lidar(current_stage, lidar):
    remove_local_graph(current_stage, LIDAR_GRAPH)
    keys = og.Controller.Keys

    og.Controller.edit(
        {"graph_path": LIDAR_GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("Render", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("Scan", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            keys.SET_VALUES: [
                ("Render.inputs:cameraPrim", [Sdf.Path(lidar)]),
                ("Scan.inputs:topicName", "scan"),
                ("Scan.inputs:frameId", SCAN_FRAME_ID),
                ("Scan.inputs:type", "laser_scan"),
                ("Scan.inputs:enabled", True),
                ("Scan.inputs:frameSkipCount", 0),
                ("Scan.inputs:showDebugView", False),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "Render.inputs:execIn"),
                ("Render.outputs:execOut", "Scan.inputs:execIn"),
                (
                    "Render.outputs:renderProductPath",
                    "Scan.inputs:renderProductPath",
                ),
                ("ROS2Context.outputs:context", "Scan.inputs:context"),
            ],
        },
    )


def main():
    enable_extensions()
    current_stage = stage()

    robot = current_stage.GetPrimAtPath(ROBOT_PRIM)
    if not robot.IsValid():
        raise RuntimeError(
            f"Robot root {ROBOT_PRIM} not found. "
            "Expected /World/heros_3w from the current imported stage."
        )

    articulation_root = find_articulation_root(current_stage)

    base_link = find_unique_prim(current_stage, BASE_LINK_NAME)
    base_scan = find_unique_prim(current_stage, BASE_SCAN_NAME)

    left_joint = validate_joint(current_stage, LEFT_WHEEL_JOINT)
    right_joint = validate_joint(current_stage, RIGHT_WHEEL_JOINT)

    lidar = find_lidar(current_stage, base_scan)

    print("Resolved HEROS stage paths:")
    print("  robot root:       ", ROBOT_PRIM)
    print("  articulation root:", articulation_root)
    print("  base_link:        ", base_link)
    print("  base_scan:        ", base_scan)
    print("  left drive joint: ", left_joint)
    print("  right drive joint:", right_joint)
    print("  lidar:            ", lidar)

    create_drive(current_stage, articulation_root)
    create_clock(current_stage)
    create_odom_tf(
        current_stage,
        base_link,
        base_scan,
    )
    create_lidar(current_stage, lidar)

    print("HEROS ROS graphs created.")
    print("Topics: /cmd_vel /clock /odom /tf /tf_static /scan")
    print("Expected TF chain: odom -> base_link -> base_scan -> lidar_frame")
    print("Verify:")
    print("  ros2 run tf2_ros tf2_echo odom base_link")
    print("  ros2 run tf2_ros tf2_echo base_link base_scan")
    print("  ros2 run tf2_ros tf2_echo base_scan lidar_frame")


main()
