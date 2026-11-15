"""iad_calibration on its own.

Live:    ros2 launch iad_calibration calibration.launch.py
Replay:  ros2 launch iad_calibration calibration.launch.py bag:=data/bags/calibration_port
         (corner + reference LiDARs, odometry and the nominal extrinsics on /tf_static)
The vehicle must stand still for a second at the start (ground-plane step), then drive past structure.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VEHICLES = {"haul_truck": (6.6, 2.8, 4.2, 4.4), "container_agv": (10.0, 3.0, 3.0, 4.2)}  # front, rear, width, height
LIDARS = ["front_left", "front_right", "rear_left", "rear_right", "top"]


def _setup(ctx):
    share = get_package_share_directory("iad_calibration")
    a = {k: LaunchConfiguration(k).perform(ctx) for k in ("bag", "rate", "vehicle", "rviz", "use_sim_time", "output",
                                                           "odom_topic", "rviz_config")}
    f, r, w, h = VEHICLES[a["vehicle"]]
    p = {"vehicle.front": f, "vehicle.rear": r, "vehicle.width": w, "vehicle.height": h, "output": a["output"],
         "odom_topic": a["odom_topic"], "use_sim_time": a["use_sim_time"] == "true"}
    actions = [Node(package="iad_calibration", executable="calibration_node", name="calibration", output="screen",
                    parameters=[os.path.join(share, "config", "calibration.param.yaml"), p])]
    if a["rviz"] == "true":
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", a["rviz_config"] or os.path.join(share, "rviz", "calibration.rviz")],
                            parameters=[{"use_sim_time": a["use_sim_time"] == "true"}]))
    if a["bag"]:
        topics = [f"/sensing/lidar/{n}/points" for n in LIDARS] + [a["odom_topic"], "/tf", "/tf_static"]
        play = ExecuteProcess(cmd=["ros2", "bag", "play", a["bag"], "--clock", "--rate", a["rate"],
                                   "--qos-profile-overrides-path", os.path.join(share, "config", "bag_qos.yaml"),
                                   "--topics"] + topics, output="screen")
        actions += [play, RegisterEventHandler(OnProcessExit(target_action=play, on_exit=[EmitEvent(event=Shutdown())]))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bag", default_value=""),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("vehicle", default_value="container_agv"),
        DeclareLaunchArgument("odom_topic", default_value="/localization/kinematic_state"),
        DeclareLaunchArgument("output", default_value="", description="YAML file for the estimated extrinsics"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
