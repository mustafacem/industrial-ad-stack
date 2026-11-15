"""iad_control on its own.

SIL (no simulator at all): the controller drives the built-in software-in-the-loop plant along a demo trajectory
  ros2 launch iad_control control.launch.py sil:=true
Live:   ros2 launch iad_control control.launch.py vehicle:=haul_truck
Replay: ros2 launch iad_control control.launch.py bag:=data/bags/control_quarry
        (open loop: replays trajectory, odometry and vehicle status; compare /control/command with the bag's)
Baselines: algorithm:=pure_pursuit, delay_compensation:=false.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VEHICLES = {  # wheelbase, max_steer, max_steer_rate, steer_tau, delay, accel_tau, max_acc, max_dec
    "haul_truck": (4.3, 0.6, 0.35, 0.25, 0.4, 0.6, 2.0, 2.5),
    "container_agv": (7.0, 0.55, 0.4, 0.2, 0.25, 0.4, 1.5, 2.0),
}
INPUTS = ["/planning/trajectory", "/localization/kinematic_state", "/vehicle/status/velocity", "/vehicle/status/steering",
          "/tf", "/tf_static"]


def _setup(ctx):
    share = get_package_share_directory("iad_control")
    a = {k: LaunchConfiguration(k).perform(ctx) for k in ("bag", "rate", "vehicle", "rviz", "use_sim_time", "algorithm",
                                                           "delay_compensation", "sil", "rviz_config", "actuator_delay", "odom_topic")}
    L, S, SR, ST, D, AT, AM, DM = VEHICLES[a["vehicle"]]
    if float(a["actuator_delay"]) >= 0:
        D = float(a["actuator_delay"])
    sil = a["sil"] == "true"
    sim_time = a["use_sim_time"] == "true" and not sil
    p = {"algorithm": a["algorithm"], "delay_compensation": a["delay_compensation"] == "true", "use_sim_time": sim_time,
         "odom_topic": a["odom_topic"],
         "vehicle.wheelbase": L, "vehicle.max_steer": S, "vehicle.max_steer_rate": SR, "vehicle.steer_tau": ST,
         "vehicle.delay": D, "vehicle.accel_tau": AT, "vehicle.max_acc": AM, "vehicle.max_dec": DM}
    actions = [Node(package="iad_control", executable="controller_node", name="controller", output="screen",
                    parameters=[os.path.join(share, "config", "control.param.yaml"), p])]
    if sil:
        actions.append(Node(package="iad_control", executable="sil_plant", output="screen",
                            parameters=[{"wheelbase": L, "delay": D, "steer_tau": ST, "max_steer_rate": SR, "max_steer": S,
                                         "accel_tau": AT, "publish_demo_trajectory": True}]))
    if a["rviz"] == "true":
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", a["rviz_config"] or os.path.join(share, "rviz", "control.rviz")],
                            parameters=[{"use_sim_time": sim_time}]))
    if a["bag"]:
        play = ExecuteProcess(cmd=["ros2", "bag", "play", a["bag"], "--clock", "--rate", a["rate"],
                                   "--qos-profile-overrides-path", os.path.join(share, "config", "bag_qos.yaml"),
                                   "--topics"] + INPUTS + [a["odom_topic"]], output="screen")
        actions += [play, RegisterEventHandler(OnProcessExit(target_action=play, on_exit=[EmitEvent(event=Shutdown())]))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sil", default_value="false", description="run the built-in SIL plant + demo trajectory"),
        DeclareLaunchArgument("bag", default_value=""),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("vehicle", default_value="haul_truck"),
        DeclareLaunchArgument("algorithm", default_value="mpc", description="mpc | pure_pursuit"),
        DeclareLaunchArgument("delay_compensation", default_value="true"),
        DeclareLaunchArgument("actuator_delay", default_value="-1", description="override the vehicle's dead time [s]"),
        DeclareLaunchArgument("odom_topic", default_value="/localization/kinematic_state"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
