"""iad_localization on its own.

Live:    ros2 launch iad_localization localization.launch.py map_dir:=data/maps/port
Replay:  ros2 launch iad_localization localization.launch.py map_dir:=data/maps/port bag:=data/bags/localization_port
map_dir holds pointcloud_map.pcd + map_info.yaml (geodetic origin of the map frame).
Variants for comparison: use_ndt:=false (EKF on GNSS + odometry only).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VEHICLES = {"haul_truck": (6.6, 2.8, 4.2), "container_agv": (10.0, 3.0, 3.0)}  # front, rear, width [m]
INPUTS = ["/sensing/lidar/top/points", "/sensing/imu/data", "/sensing/gnss/fix", "/vehicle/status/velocity", "/tf_static"]


def _setup(ctx):
    share = get_package_share_directory("iad_localization")
    a = {k: LaunchConfiguration(k).perform(ctx) for k in
         ("map_dir", "bag", "rate", "vehicle", "rviz", "use_ndt", "initial_pose", "use_sim_time", "rviz_config")}
    map_dir = os.path.abspath(a["map_dir"])
    info = yaml.safe_load(open(os.path.join(map_dir, "map_info.yaml")))
    f, r, w = VEHICLES[a["vehicle"]]
    p = {"map_file": os.path.join(map_dir, "pointcloud_map.pcd"), "use_ndt": a["use_ndt"] == "true",
         "map_origin.latitude": float(info["origin"]["latitude"]), "map_origin.longitude": float(info["origin"]["longitude"]),
         "map_origin.altitude": float(info["origin"]["altitude"]), "vehicle.front": f, "vehicle.rear": r, "vehicle.width": w,
         "use_sim_time": a["use_sim_time"] == "true"}
    if a["initial_pose"]:
        p["initial_pose"] = [float(v) for v in a["initial_pose"].split(",")]
    actions = [Node(package="iad_localization", executable="localization_node", name="localization", output="screen",
                    parameters=[os.path.join(share, "config", "localization.param.yaml"), p])]
    if a["rviz"] == "true":
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", a["rviz_config"] or os.path.join(share, "rviz", "localization.rviz")],
                            parameters=[{"use_sim_time": a["use_sim_time"] == "true"}]))
    if a["bag"]:
        play = ExecuteProcess(cmd=["ros2", "bag", "play", a["bag"], "--clock", "--rate", a["rate"],
                                   "--qos-profile-overrides-path", os.path.join(share, "config", "bag_qos.yaml"),
                                   "--topics"] + INPUTS, output="screen")
        actions += [play, RegisterEventHandler(OnProcessExit(target_action=play, on_exit=[EmitEvent(event=Shutdown())]))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_dir", description="directory with pointcloud_map.pcd + map_info.yaml"),
        DeclareLaunchArgument("bag", default_value="", description="rosbag2 to replay (empty = live)"),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("vehicle", default_value="container_agv"),
        DeclareLaunchArgument("use_ndt", default_value="true"),
        DeclareLaunchArgument("initial_pose", default_value="", description="x,y,yaw (empty = automatic)"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
