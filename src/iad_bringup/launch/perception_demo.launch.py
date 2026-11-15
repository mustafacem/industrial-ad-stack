"""Perception demo (quarry): the haul truck drives the haul road through dust; iad_perception runs twice,
the stack (mode ours) and the naive baseline, both scored live against ground truth (iad_eval).
Localization is replaced by the ground-truth TF here (this demo isolates perception)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from iad_bringup.demo import pcd_map, b, ego, evaluator, hud, include, logger, rviz, sim

ARGS = {"rviz": "true", "split": "true", "cameras": "false", "duration": "95", "start_after": "8", "bag": "", "log_dir": "/ws/output/logs"}


def setup(ctx):
    a = {k: LaunchConfiguration(k).perform(ctx) for k in ARGS}
    L = a["log_dir"]
    acts = [sim(world="quarry", vehicle="haul_truck", driver="route", route="haul", gt_tf="true", cameras=a["cameras"],
                duration=a["duration"], start_after=a["start_after"], bag=a["bag"]),
            include("iad_perception", "perception.launch.py", rviz="false", name="perception", mode="ours"),
            include("iad_perception", "perception.launch.py", rviz="false", name="perception_naive", mode="naive"),
            evaluator("perception_eval", scenario="quarry", name="perception", log=f"{L}/perception_ours.jsonl"),
            evaluator("perception_eval", scenario="quarry", perception_ns="/perception_naive", name="perception_naive",
                      log=f"{L}/perception_naive.jsonl"),
            pcd_map("quarry"), hud("perception"), ego("haul_truck")]
    if b(a["rviz"]):
        acts += [rviz("perception_naive.rviz", "rviz_naive"), rviz("perception_ours.rviz", "rviz_ours")] if b(a["split"]) \
            else [rviz("perception_demo.rviz")]
    return acts


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] + [OpaqueFunction(function=setup)])
