"""Control demo (quarry): the 70 t loaded haul truck (0.4 s actuator dead time, slow hydraulic steering)
drives from the dump approach and reverses into the dump bay on a grade. algorithm:=mpc|pure_pursuit,
delay_compensation:=true|false; scored against ground truth (iad_eval control_eval)."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from iad_bringup.demo import pcd_map, SIM_T, b, ego, evaluator, hud, include, rviz, sim, world_dir
from launch_ros.actions import Node

ARGS = {"rviz": "true", "cameras": "false", "duration": "90", "start_after": "8", "bag": "", "algorithm": "mpc",
        "delay_compensation": "true", "loaded": "true", "log_dir": "/ws/output/logs", "goals": "dump", "start": "dump_approach"}


def setup(ctx):
    a = {k: LaunchConfiguration(k).perform(ctx) for k in ARGS}
    wd = world_dir("quarry")
    tag = f"{a['algorithm']}_{'comp' if b(a['delay_compensation']) else 'nocomp'}"
    acts = [sim(world="quarry", vehicle="haul_truck", spawn_pose=a["start"], gt_tf="true", loaded=a["loaded"], traffic="false",
                cameras=a["cameras"], duration=a["duration"], start_after=a["start_after"], bag=a["bag"],
                bag_topics="/planning/trajectory,/planning/goal"),
            include("iad_planning", "planning.launch.py", rviz="false", vehicle="haul_truck", map_dir=os.path.join(wd, "maps"),
                    odom_topic="/sim/ground_truth/odom", use_live_occupancy="false"),
            include("iad_control", "control.launch.py", rviz="false", vehicle="haul_truck", algorithm=a["algorithm"],
                    odom_topic="/sim/ground_truth/odom",
                    delay_compensation=a["delay_compensation"]),
            Node(package="iad_tools", executable="mission", output="screen",
                 parameters=[{"site_file": os.path.join(wd, "site.yaml"), "goals": a["goals"], **SIM_T}]),
            evaluator("control_eval", log=f"{a['log_dir']}/control_{tag}.jsonl"),
            pcd_map("quarry"), hud("control"), ego("haul_truck")]
    if b(a["rviz"]):
        acts.append(rviz("control_demo.rviz"))
    return acts


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] + [OpaqueFunction(function=setup)])
