"""Localization demo (port): the container AGV drives a loop between container stacks and under the quay
cranes where GNSS degrades to float / single / no fix; iad_localization (EKF + NDT, its comparison variants
included) is scored against ground truth."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from iad_bringup.demo import b, ego, evaluator, hud, include, rviz, sim, world_dir

ARGS = {"rviz": "true", "cameras": "false", "duration": "175", "start_after": "8", "bag": "", "log_dir": "/ws/output/logs"}


def setup(ctx):
    a = {k: LaunchConfiguration(k).perform(ctx) for k in ARGS}
    acts = [sim(world="port", vehicle="container_agv", driver="route", route="loc_loop", cameras=a["cameras"],
                duration=a["duration"], start_after=a["start_after"], bag=a["bag"]),
            include("iad_localization", "localization.launch.py", rviz="false", vehicle="container_agv",
                    map_dir=os.path.join(world_dir("port"), "maps")),
            evaluator("localization_eval", log=f"{a['log_dir']}/localization.jsonl"),
            hud("localization"), ego("container_agv")]
    if b(a["rviz"]):
        acts.append(rviz("localization_demo.rviz"))
    return acts


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] + [OpaqueFunction(function=setup)])
