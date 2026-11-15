"""Building blocks for the demo launch files: the Gazebo site, each standalone subsystem through its OWN
launch file, ground-truth evaluators, HUD, ego marker and RViz. Integration only - the subsystem packages
never import any of this."""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

SIM_T = {"use_sim_time": True}
VEHICLE_BOX = {"haul_truck": dict(front=6.6, rear=2.8, width=4.2, height=4.4, color=[0.95, 0.68, 0.12], lidar=[6.3, 0.0, 4.75]),
               "container_agv": dict(front=10.0, rear=3.0, width=3.0, height=4.2, color=[0.2, 0.5, 0.85], lidar=[9.85, 0.0, 5.05])}


def b(v):
    return str(v).lower() in ("1", "true", "yes", "on")


def include(pkg, launch_file, **args):
    """A package's own launch file in a scoped group. Launch configurations are global in ROS 2 launch: without
    the scope and the explicit empty `bag`, the demo's `bag:=` (record) would turn a subsystem's own launch file
    into replay mode."""
    if pkg != "iad_gz_sim":
        args.setdefault("bag", "")
    inc = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share(pkg), "launch", launch_file)),
                                   launch_arguments={k: str(v) for k, v in args.items()}.items())
    return GroupAction([inc], scoped=True)


def world_dir(world):
    return os.path.join(share("iad_gz_sim"), "worlds", world)


def sim(**args):
    return include("iad_gz_sim", "sim.launch.py", **args)


def rviz(config, name="rviz", x=None):
    return Node(package="rviz2", executable="rviz2", name=name, output="log", parameters=[SIM_T],
                arguments=["-d", os.path.join(share("iad_bringup"), "rviz", config)])


def hud(profile):
    # wall clock on purpose: the HUD only re-renders the latest metrics (a /clock subscription at the 500 Hz
    # physics rate would cost a fifth of a core per Python node)
    return Node(package="iad_tools", executable="hud", name=f"hud_{profile}", output="log",
                parameters=[{"profile": profile}])


def ego(vehicle):
    return Node(package="iad_tools", executable="vehicle_marker", output="log", parameters=[VEHICLE_BOX[vehicle]])


def evaluator(executable, **params):
    return Node(package="iad_eval", executable=executable, output="screen", parameters=[{**params, **SIM_T}])


def logger(path, topics):
    return Node(package="iad_tools", executable="metrics_logger", output="log",
                parameters=[{"path": path, "topics": topics, **SIM_T}])


def pcd_map(world, voxel=1.0):
    """Site point-cloud map for RViz context (demos without iad_localization, which publishes it itself)."""
    return Node(package="iad_tools", executable="pcd_map", output="log",
                parameters=[{"pcd_file": os.path.join(world_dir(world), "maps", "pointcloud_map.pcd"), "voxel": voxel}])
