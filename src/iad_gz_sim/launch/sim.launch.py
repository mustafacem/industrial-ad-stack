"""Gazebo Harmonic simulation of a site with the ego truck, its sensors bridged to ROS 2.

  ros2 launch iad_gz_sim sim.launch.py world:=quarry vehicle:=haul_truck spawn_pose:=start driver:=route route:=haul

Provides (sim time): /clock, /sensing/lidar/*/points, /sensing/imu/data, /sensing/gnss/fix,
/vehicle/status/{velocity,steering}, /tf_static (nominal extrinsics), /sim/ground_truth/odom; consumes
/control/command. The world starts paused; sim_supervisor unpauses it after `start_after` wall seconds
(-1 = leave paused) and ends the launch after `duration` sim seconds (0 = run forever).
"""
import hashlib
import os
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from iad_gz_sim.vehicles import SPECS, bridge_config, vehicle_sdf

ARGS = {
    "world": "quarry", "vehicle": "haul_truck", "spawn_pose": "start", "loaded": "false",
    "corner_lidars": "false", "cameras": "false", "ego_dust": "true", "traffic": "true", "dust": "true",
    "gt_tf": "false", "actuator_delay": "-1", "extrinsic_error_deg": "0.0", "extrinsic_error_m": "0.0",
    "gnss_effects": "true", "rtf": "1.0", "start_after": "6.0", "duration": "0", "unmapped_obstacle": "false",
    "driver": "none", "route": "", "speed": "-1", "seed": "7", "odom_scale_error": "0.015", "gz_verbose": "1",
    "bag": "", "bag_topics": "",
}

# Dataset recording (bag:=<dir>): every sensor and vehicle topic a subsystem consumes, plus ground truth
# for offline evaluation. Subsystem replays select their own inputs from it.
BAG_TOPICS = ["/sensing/lidar/top/points", "/sensing/imu/data", "/sensing/gnss/fix", "/vehicle/status/velocity",
              "/vehicle/status/steering", "/control/command", "/tf_static", "/tf", "/sim/ground_truth/odom",
              "/sim/ground_truth/extrinsics", "/sim/gnss/debug", "/sim/vehicle/debug", "/sim/traffic/poses"]


def _b(v):
    return str(v).lower() in ("1", "true", "yes", "on")


def _setup(context):
    a = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    share = get_package_share_directory("iad_gz_sim")
    worlds = os.path.join(share, "worlds")
    wname = a["world"]
    site_file = os.path.join(worlds, wname, "site.yaml")
    site = yaml.safe_load(open(site_file))
    cache = os.path.join(os.path.expanduser(os.environ.get("ROS_HOME", "~/.ros")), "iad_gz_sim")
    os.makedirs(cache, exist_ok=True)

    # World variant: real-time factor, traffic / dust blocks on or off.
    sdf = open(os.path.join(worlds, wname, "world.sdf")).read()
    sdf = sdf.replace("<real_time_factor>1.0</real_time_factor>", f"<real_time_factor>{float(a['rtf'])}</real_time_factor>")
    for block, on in (("traffic", _b(a["traffic"])), ("dust", _b(a["dust"]))):
        if not on:
            sdf = re.sub(rf"<!--BEGIN:{block}-->.*?<!--END:{block}-->", "", sdf, flags=re.S)
    # Absolute paths: particle emitters do not resolve model:// URIs for their textures.
    sdf = sdf.replace(f"model://{wname}/", os.path.join(worlds, wname) + "/")
    tag = hashlib.md5(sdf.encode()).hexdigest()[:8]
    world_file = os.path.join(cache, f"{wname}_{tag}.sdf")
    open(world_file, "w").write(sdf)

    spec = SPECS[a["vehicle"]]
    delay = float(a["actuator_delay"])
    vsdf = vehicle_sdf(a["vehicle"], wname, loaded=_b(a["loaded"]), corner_lidars=_b(a["corner_lidars"]),
                       cameras=_b(a["cameras"]), ego_dust=_b(a["ego_dust"]), gt_tf=_b(a["gt_tf"]),
                       actuator_delay=None if delay < 0 else delay, odom_scale_error=float(a["odom_scale_error"]))
    vsdf = vsdf.replace(f"model://{wname}/", os.path.join(worlds, wname) + "/")
    vfile = os.path.join(cache, f"ego_{hashlib.md5(vsdf.encode()).hexdigest()[:8]}.sdf")
    open(vfile, "w").write(vsdf)
    bfile = os.path.join(cache, f"bridge_{wname}_{a['vehicle']}_{int(_b(a['corner_lidars']))}.yaml")
    open(bfile, "w").write(bridge_config(wname, a["vehicle"], _b(a["corner_lidars"])))

    p = site["poses"][a["spawn_pose"]]
    env = {"GZ_SIM_RESOURCE_PATH": worlds + ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
           # gz-transport discovery inside a container needs both: loopback + an explicit partition
           "GZ_IP": os.environ.get("GZ_IP", "127.0.0.1"), "GZ_PARTITION": os.environ.get("GZ_PARTITION", "iad")}
    os.environ.update(env)  # nodes started below (bridge, create) inherit it
    sim_t = {"use_sim_time": True}
    gz_cmd = ["gz", "sim", "-s", "--headless-rendering", "-v", a["gz_verbose"], world_file]
    actions = [
        ExecuteProcess(cmd=gz_cmd, output="screen", additional_env=env, name="gz_server", sigterm_timeout="3"),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-world", wname, "-file", vfile, "-name", "ego", "-x", str(p["x"]), "-y", str(p["y"]),
                        "-z", str(p["z"] + 0.15), "-Y", str(p["yaw"])]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge", output="screen",
             parameters=[{"config_file": bfile, **sim_t}]),
        Node(package="iad_gz_sim", executable="vehicle_description", output="screen",
             parameters=[{"vehicle": a["vehicle"], "corner_lidars": _b(a["corner_lidars"]),
                          "extrinsic_error_deg": float(a["extrinsic_error_deg"]),
                          "extrinsic_error_m": float(a["extrinsic_error_m"]), "seed": int(a["seed"])}]),
        Node(package="iad_gz_sim", executable="gnss_effects", output="screen",
             parameters=[{"scenario": wname, "seed": int(a["seed"]), "enabled": _b(a["gnss_effects"]),
                          "origin_latitude": float(site["origin"]["latitude"]),
                          "origin_longitude": float(site["origin"]["longitude"]),
                          "origin_altitude": float(site["origin"]["altitude"]), **sim_t}]),
    ]
    sup = Node(package="iad_gz_sim", executable="sim_supervisor", output="screen",
               parameters=[{"world": wname, "duration": float(a["duration"]), "start_after": float(a["start_after"]),
                            **sim_t}])
    actions += [sup, RegisterEventHandler(OnProcessExit(target_action=sup, on_exit=[EmitEvent(event=Shutdown())]))]
    if _b(a["unmapped_obstacle"]):
        extra = os.path.join(worlds, wname, "extras", "unmapped_container.sdf")
        if os.path.exists(extra):
            # `create` overrides the file's <pose> with its own (default: origin): pass it explicitly.
            x, y, z, _, _, yaw = re.search(r"<pose>([^<]+)</pose>", open(extra).read()).group(1).split()
            actions.append(Node(package="ros_gz_sim", executable="create", output="screen",
                                arguments=["-world", wname, "-file", extra, "-name", "unmapped_container",
                                           "-x", x, "-y", y, "-z", z, "-Y", yaw]))
    if a["bag"]:
        topics = BAG_TOPICS + [f"/sensing/lidar/{l.name}/points" for l in spec.corners if _b(a["corner_lidars"])]
        topics += [t for t in a["bag_topics"].split(",") if t]
        actions.append(ExecuteProcess(
            cmd=["ros2", "bag", "record", "-s", "mcap", "--storage-preset-profile", "zstd_fast", "--use-sim-time",
                 "--max-cache-size", str(200 * 1024 * 1024), "-o", a["bag"]] + topics,
            output="log", sigterm_timeout="10"))
    if a["driver"] == "route":
        rp = {"site_file": site_file, "route": a["route"], "wheelbase": spec.wheelbase, "max_steer": spec.max_steer, **sim_t}
        if float(a["speed"]) > 0:
            rp["speed"] = float(a["speed"])
        actions.append(Node(package="iad_gz_sim", executable="route_driver", output="screen", parameters=[rp]))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] +
                             [OpaqueFunction(function=_setup)])
