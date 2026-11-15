"""Planning demo (port): free-space Hybrid A* + Reeds-Shepp for the container AGV between named site goals,
an unmapped container on the first route (stop, wait, replan around it), reverse parking under a quay crane.
Live occupancy from iad_perception, tracking by iad_control; ground-truth TF stands in for localization."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from iad_bringup.demo import pcd_map, SIM_T, b, ego, hud, include, rviz, sim, world_dir
from launch_ros.actions import Node

ARGS = {"rviz": "true", "cameras": "false", "duration": "260", "start_after": "8", "bag": "", "goals": "rtg_lane_end,crane2_lane",
        "start": "start"}


def setup(ctx):
    a = {k: LaunchConfiguration(k).perform(ctx) for k in ARGS}
    wd = world_dir("port")
    acts = [sim(world="port", vehicle="container_agv", spawn_pose=a["start"], gt_tf="true", unmapped_obstacle="true",
                traffic="false", cameras=a["cameras"], duration=a["duration"], start_after=a["start_after"], bag=a["bag"],
                bag_topics="/perception/occupancy_grid,/planning/goal,/planning/route,/planning/trajectory"),
            include("iad_perception", "perception.launch.py", rviz="false", vehicle="container_agv"),
            include("iad_planning", "planning.launch.py", rviz="false", vehicle="container_agv", map_dir=os.path.join(wd, "maps"),
                    odom_topic="/sim/ground_truth/odom", viz_scale="2.5"),
            include("iad_control", "control.launch.py", rviz="false", vehicle="container_agv", odom_topic="/sim/ground_truth/odom"),
            Node(package="iad_tools", executable="mission", output="screen",
                 parameters=[{"site_file": os.path.join(wd, "site.yaml"), "goals": a["goals"], **SIM_T}]),
            pcd_map("port"), hud("planning"), ego("container_agv")]
    if b(a["rviz"]):
        acts.append(rviz("planning_demo.rviz"))
    return acts


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] + [OpaqueFunction(function=setup)])
