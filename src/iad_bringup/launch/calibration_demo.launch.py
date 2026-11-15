"""Calibration demo (port): the AGV's four corner LiDARs start from wrong nominal extrinsics (degrees and
decimetres off); iad_calibration estimates them while the AGV parks, then drives past the stacks; the
estimates are scored against the simulator's true sensor poses."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from iad_bringup.demo import pcd_map, b, ego, evaluator, hud, include, rviz, sim

ARGS = {"rviz": "true", "cameras": "false", "duration": "120", "start_after": "8", "bag": "", "error_deg": "3.0",
        "error_m": "0.3", "output": "/ws/output/calibration/extrinsics.yaml"}


def setup(ctx):
    a = {k: LaunchConfiguration(k).perform(ctx) for k in ARGS}
    acts = [sim(world="port", vehicle="container_agv", spawn_pose="calib_start", corner_lidars="true", gt_tf="true",
                extrinsic_error_deg=a["error_deg"], extrinsic_error_m=a["error_m"], driver="route", route="calib_route",
                traffic="false", cameras=a["cameras"], duration=a["duration"], start_after=a["start_after"], bag=a["bag"]),
            include("iad_calibration", "calibration.launch.py", rviz="false", vehicle="container_agv",
                    odom_topic="/sim/ground_truth/odom", output=a["output"]),
            evaluator("calibration_eval"), pcd_map("port"), hud("calibration"), ego("container_agv")]
    if b(a["rviz"]):
        acts.append(rviz("calibration_demo.rviz"))
    return acts


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] + [OpaqueFunction(function=setup)])
