"""iad_planning on its own.

Live:    ros2 launch iad_planning planning.launch.py map_dir:=data/maps/port vehicle:=container_agv
Replay:  ros2 launch iad_planning planning.launch.py map_dir:=data/maps/port bag:=data/bags/planning_port
         (replays odometry, live occupancy and goals; the planner re-plans on them)
Ad-hoc:  set a goal with RViz "2D Goal Pose" (published on /planning/goal).
map_dir holds costmap.yaml/.pgm (+ optional pointcloud_map.pcd for elevation).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VEHICLES = {  # wheelbase, width, front, rear, max_steer, max_steer_rate
    "haul_truck": (4.3, 4.2, 6.6, 2.8, 0.6, 0.35),
    "container_agv": (7.0, 3.0, 10.0, 3.0, 0.55, 0.4),
}
INPUTS = ["/localization/kinematic_state", "/perception/occupancy_grid", "/planning/goal", "/planning/route", "/tf", "/tf_static"]


def _setup(ctx):
    share = get_package_share_directory("iad_planning")
    a = {k: LaunchConfiguration(k).perform(ctx) for k in ("map_dir", "bag", "rate", "vehicle", "rviz", "use_sim_time",
                                                           "use_live_occupancy", "rviz_config", "odom_topic", "debug_dump", "viz_scale")}
    m = os.path.abspath(a["map_dir"])
    L, W, F, R, S, SR = VEHICLES[a["vehicle"]]
    p = {"costmap_file": os.path.join(m, "costmap.yaml"), "vehicle.wheelbase": L, "vehicle.width": W, "vehicle.front": F,
         "vehicle.rear": R, "vehicle.max_steer": S, "vehicle.max_steer_rate": SR, "use_sim_time": a["use_sim_time"] == "true",
         "use_live_occupancy": a["use_live_occupancy"] == "true", "odom_topic": a["odom_topic"], "debug_dump_file": a["debug_dump"], "viz_scale": float(a["viz_scale"])}
    if os.path.exists(os.path.join(m, "pointcloud_map.pcd")):
        p["elevation_file"] = os.path.join(m, "pointcloud_map.pcd")
    actions = [Node(package="iad_planning", executable="planner_node", name="planner", output="screen",
                    parameters=[os.path.join(share, "config", "planning.param.yaml"), p])]
    if a["rviz"] == "true":
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", a["rviz_config"] or os.path.join(share, "rviz", "planning.rviz")],
                            parameters=[{"use_sim_time": a["use_sim_time"] == "true"}]))
    if a["bag"]:
        play = ExecuteProcess(cmd=["ros2", "bag", "play", a["bag"], "--clock", "--rate", a["rate"],
                                   "--qos-profile-overrides-path", os.path.join(share, "config", "bag_qos.yaml"),
                                   "--topics"] + INPUTS + [a["odom_topic"]], output="screen")
        actions += [play, RegisterEventHandler(OnProcessExit(target_action=play, on_exit=[EmitEvent(event=Shutdown())]))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_dir", description="directory with costmap.yaml (+ pointcloud_map.pcd)"),
        DeclareLaunchArgument("bag", default_value=""),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("vehicle", default_value="haul_truck"),
        DeclareLaunchArgument("use_live_occupancy", default_value="true"),
        DeclareLaunchArgument("odom_topic", default_value="/localization/kinematic_state"),
        DeclareLaunchArgument("viz_scale", default_value="1.0", description="marker line-width multiplier"),
        DeclareLaunchArgument("debug_dump", default_value="", description="PGM of the fused costmap on a failed plan"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
