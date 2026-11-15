"""Full mission (quarry), whole stack closed loop with no ground-truth shortcuts: Gazebo sensors ->
iad_localization (TF map->base_link) -> iad_perception -> iad_planning (haul road in route mode, then
Hybrid A* reverse into the dump bay) -> iad_control -> drive-by-wire. Scored by the evaluators."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from iad_bringup.demo import SIM_T, b, ego, evaluator, hud, include, rviz, sim, world_dir
from launch_ros.actions import Node

ARGS = {"rviz": "true", "cameras": "false", "duration": "200", "start_after": "8", "bag": "", "goals": "route:haul>dump_approach,dump",
        "loaded": "true", "log_dir": "/ws/output/logs", "ego_dust": "true"}


def setup(ctx):
    a = {k: LaunchConfiguration(k).perform(ctx) for k in ARGS}
    wd = world_dir("quarry")
    maps = os.path.join(wd, "maps")
    L = a["log_dir"]
    acts = [sim(world="quarry", vehicle="haul_truck", loaded=a["loaded"], ego_dust=a["ego_dust"], cameras=a["cameras"], duration=a["duration"],
                start_after=a["start_after"], bag=a["bag"],
                bag_topics="/localization/kinematic_state,/perception/occupancy_grid,/planning/goal,/planning/route,/planning/trajectory"),
            include("iad_localization", "localization.launch.py", rviz="false", vehicle="haul_truck", map_dir=maps),
            include("iad_perception", "perception.launch.py", rviz="false", vehicle="haul_truck"),
            include("iad_planning", "planning.launch.py", rviz="false", vehicle="haul_truck", map_dir=maps,
                    debug_dump=f"{L}/planner_fail.pgm"),
            include("iad_control", "control.launch.py", rviz="false", vehicle="haul_truck"),
            Node(package="iad_tools", executable="mission", output="screen",
                 parameters=[{"site_file": os.path.join(wd, "site.yaml"), "goals": a["goals"], **SIM_T}]),
            evaluator("perception_eval", scenario="quarry", name="perception", log=f"{L}/mission_perception.jsonl"),
            evaluator("localization_eval", log=f"{L}/mission_localization.jsonl"),
            evaluator("control_eval", log=f"{L}/mission_control.jsonl"),
            hud("mission"), ego("haul_truck")]
    if b(a["rviz"]):
        acts.append(rviz("mission_demo.rviz"))
    return acts


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] + [OpaqueFunction(function=setup)])
