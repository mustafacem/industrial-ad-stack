"""iad_perception on its own.

Live (inputs from a running simulator or vehicle):
  ros2 launch iad_perception perception.launch.py
Replay (no simulator): plays a rosbag2 with /sensing/lidar/top/points + /tf + /tf_static
  ros2 launch iad_perception perception.launch.py bag:=data/bags/perception_quarry
Baseline for comparison: mode:=naive (no dust filter, one RANSAC plane).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VEHICLES = {  # body box for the self-filter: front, rear, width, height [m] from base_link
    "haul_truck": (6.6, 2.8, 4.2, 4.4),
    "container_agv": (10.0, 3.0, 3.0, 4.2),
}


def _setup(ctx):
    share = get_package_share_directory("iad_perception")
    a = {k: LaunchConfiguration(k).perform(ctx) for k in
         ("bag", "rate", "mode", "vehicle", "rviz", "params_file", "name", "use_sim_time", "rviz_config")}
    f, r, w, h = VEHICLES[a["vehicle"]]
    params = [a["params_file"] or os.path.join(share, "config", "perception.param.yaml"),
              {"mode": a["mode"], "use_sim_time": a["use_sim_time"] == "true",
               "vehicle.front": f, "vehicle.rear": r, "vehicle.width": w, "vehicle.height": h}]
    actions = [Node(package="iad_perception", executable="perception_node", name=a["name"], output="screen",
                    parameters=params)]
    if a["rviz"] == "true":
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", a["rviz_config"] or os.path.join(share, "rviz", "perception.rviz")],
                            parameters=[{"use_sim_time": a["use_sim_time"] == "true"}]))
    if a["bag"]:
        play = ExecuteProcess(cmd=["ros2", "bag", "play", a["bag"], "--clock", "--rate", a["rate"],
                                   "--qos-profile-overrides-path", os.path.join(share, "config", "bag_qos.yaml"),
                                   "--topics", "/sensing/lidar/top/points", "/tf", "/tf_static"],
                              output="screen")
        actions += [play, RegisterEventHandler(OnProcessExit(target_action=play, on_exit=[EmitEvent(event=Shutdown())]))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bag", default_value="", description="rosbag2 to replay (empty = live)"),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("mode", default_value="ours", description="ours | naive"),
        DeclareLaunchArgument("vehicle", default_value="haul_truck", description="haul_truck | container_agv"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=""),
        DeclareLaunchArgument("params_file", default_value=""),
        DeclareLaunchArgument("name", default_value="perception"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
