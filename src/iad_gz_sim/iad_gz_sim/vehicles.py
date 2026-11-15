"""Vehicle descriptions for Gazebo: SDF models (sprung chassis, steered front wheels, driven rear
wheels, sensors), the nominal sensor extrinsics published as /tf_static, and the ros_gz_bridge
configuration. One source of truth for the sim side; the subsystems only ever see /tf_static and
topics."""
import math
import random
from dataclasses import dataclass, field


@dataclass
class Lidar:
    name: str
    xyz: tuple
    rpy: tuple = (0.0, 0.0, 0.0)
    channels: int = 32
    vmin: float = -25.0
    vmax: float = 15.0
    hsamples: int = 1200
    range_max: float = 120.0
    rate: float = 10.0


@dataclass
class Spec:
    name: str
    wheelbase: float
    track: float
    wheel_radius: float
    tyre_width: float
    front: float
    rear: float
    width: float
    height: float
    mass_empty: float
    payload: float
    max_steer: float
    max_steer_rate: float
    steer_tau: float
    delay: float
    accel_tau: float
    max_acc: float
    max_dec: float
    power: float
    color: str
    lidar_top: Lidar
    gnss: tuple
    imu: tuple = (1.5, 0.0, 1.2)
    corners: list = field(default_factory=list)
    chase: tuple = (-24.0, 0.0, 12.0, 0.0, 0.33, 0.0)


SPECS = {
    "haul_truck": Spec(
        name="haul_truck", wheelbase=4.3, track=3.3, wheel_radius=1.2, tyre_width=1.0, front=6.6, rear=2.8,
        width=4.2, height=4.4, mass_empty=32000.0, payload=40000.0, max_steer=0.6, max_steer_rate=0.35,
        steer_tau=0.25, delay=0.4, accel_tau=0.6, max_acc=2.0, max_dec=2.5, power=400e3,
        color="0.93 0.66 0.10 1", lidar_top=Lidar("top", (6.3, 0.0, 4.75)), gnss=(4.6, -1.6, 4.4)),
    "container_agv": Spec(
        name="container_agv", wheelbase=7.0, track=2.6, wheel_radius=0.55, tyre_width=0.5, front=10.0,
        rear=3.0, width=3.0, height=4.2, mass_empty=25000.0, payload=30000.0, max_steer=0.55,
        max_steer_rate=0.4, steer_tau=0.2, delay=0.25, accel_tau=0.4, max_acc=1.5, max_dec=2.0,
        power=250e3, color="0.15 0.42 0.75 1", lidar_top=Lidar("top", (9.85, 0.0, 5.05)),
        gnss=(9.85, 0.45, 5.3), chase=(-26.0, -8.0, 13.0, 0.0, 0.36, 0.3),
        corners=[
            Lidar("front_left", (10.05, 1.55, 1.95), (0.0, math.radians(10), math.radians(45)), 16, -15, 15, 900, 80),
            Lidar("front_right", (10.05, -1.55, 1.95), (0.0, math.radians(10), math.radians(-45)), 16, -15, 15, 900, 80),
            Lidar("rear_left", (-3.05, 1.55, 1.95), (0.0, math.radians(10), math.radians(135)), 16, -15, 15, 900, 80),
            Lidar("rear_right", (-3.05, -1.55, 1.95), (0.0, math.radians(10), math.radians(-135)), 16, -15, 15, 900, 80),
        ]),
}


def _f(*v):
    return " ".join(f"{x:.4f}" for x in v)


def _box_inertia(m, x, y, z):
    return (f"<inertial><mass>{m:.1f}</mass><inertia><ixx>{m * (y * y + z * z) / 12:.1f}</ixx>"
            f"<iyy>{m * (x * x + z * z) / 12:.1f}</iyy><izz>{m * (x * x + y * y) / 12:.1f}</izz>"
            "<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>")


def _vbox(name, xyz, size, color, retro=45, rpy=(0, 0, 0)):
    return (f'<visual name="{name}"><pose relative_to="__model__">{_f(*xyz, *rpy)}</pose>'
            f"<laser_retro>{retro}</laser_retro><geometry><box><size>{_f(*size)}</size></box></geometry>"
            f"<material><ambient>{color}</ambient><diffuse>{color}</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual>")


def _lidar_sensor(l: Lidar, topic):
    return (f'<sensor name="lidar_{l.name}" type="gpu_lidar"><pose relative_to="__model__">{_f(*l.xyz, *l.rpy)}</pose>'
            f"<topic>{topic}</topic><gz_frame_id>lidar_{l.name}</gz_frame_id><update_rate>{l.rate}</update_rate>"
            "<always_on>true</always_on><visualize>false</visualize><lidar><scan>"
            f"<horizontal><samples>{l.hsamples}</samples><resolution>1</resolution><min_angle>{-math.pi:.6f}</min_angle><max_angle>{math.pi:.6f}</max_angle></horizontal>"
            f"<vertical><samples>{l.channels}</samples><resolution>1</resolution><min_angle>{math.radians(l.vmin):.6f}</min_angle><max_angle>{math.radians(l.vmax):.6f}</max_angle></vertical>"
            f"</scan><range><min>1.0</min><max>{l.range_max}</max><resolution>0.01</resolution></range>"
            "<noise><type>gaussian</type><mean>0</mean><stddev>0.02</stddev></noise></lidar></sensor>")


def _noise(sd, bias=0.0):
    return (f'<noise type="gaussian"><mean>0</mean><stddev>{sd}</stddev>'
            f"<bias_mean>{bias}</bias_mean><bias_stddev>0</bias_stddev></noise>")


def vehicle_sdf(vehicle="haul_truck", world="quarry", loaded=False, corner_lidars=False, cameras=False,
                ego_dust=False, gt_tf=False, actuator_delay=None, odom_scale_error=0.015, model_name="ego"):
    s = SPECS[vehicle]
    m_total = s.mass_empty + (s.payload if loaded else 0.0)
    R, L, T = s.wheel_radius, s.wheelbase, s.track
    m_wheel = 900.0 if vehicle == "haul_truck" else 300.0
    m_small = 60.0
    m_chassis = m_total - 4 * m_wheel - 6 * m_small
    xc, zc = 0.45 * L, (1.9 if vehicle == "haul_truck" else 1.25) + (0.4 if loaded and vehicle == "haul_truck" else 0.0)
    sag = 0.07
    k = m_chassis * 9.81 / 4 / sag
    c = 2 * 0.45 * math.sqrt(k * m_chassis / 4)
    delay = s.delay if actuator_delay is None else actuator_delay
    hold = 0.3 * m_total * s.max_dec * R

    out = [f'<?xml version="1.0"?><sdf version="1.10"><model name="{model_name}"><self_collide>false</self_collide>']
    # ---- chassis (sprung mass): one collision box, visuals in the model (base_link) frame
    bx, bl = (s.front - s.rear) / 2, s.front + s.rear
    out.append(f'<link name="chassis"><pose relative_to="__model__">{_f(xc, 0, zc, 0, 0, 0)}</pose>')
    out.append(_box_inertia(m_chassis, bl * 0.8, s.width * 0.8, 2.4))
    z0 = R + 0.35
    out.append(f'<collision name="body"><pose relative_to="__model__">{_f(bx, 0, z0 + (s.height - z0) / 2, 0, 0, 0)}</pose>'
               f"<geometry><box><size>{_f(bl, s.width, s.height - z0)}</size></box></geometry></collision>")
    dark, steel = "0.14 0.14 0.15 1", "0.45 0.46 0.48 1"
    if vehicle == "haul_truck":
        out.append(_vbox("frame", (bx, 0, R + 0.25), (bl - 0.4, 1.6, 0.7), dark))
        # dump body: floor, sides, front wall + canopy over the cab
        out.append(_vbox("bed_floor", (2.0, 0, R + 1.05), (7.6, s.width, 0.35), s.color, 55, (0, -0.06, 0)))
        out.append(_vbox("bed_left", (2.0, s.width / 2 - 0.12, R + 1.9), (7.6, 0.24, 1.7), s.color, 55))
        out.append(_vbox("bed_right", (2.0, -s.width / 2 + 0.12, R + 1.9), (7.6, 0.24, 1.7), s.color, 55))
        out.append(_vbox("bed_front", (5.75, 0, R + 1.95), (0.3, s.width, 1.9), s.color, 55))
        out.append(_vbox("canopy", (6.25, 0, R + 2.95), (1.4, s.width, 0.18), s.color, 55))
        out.append(_vbox("cab", (6.25, 1.3, R + 1.75), (1.3, 1.5, 1.5), "0.85 0.85 0.82 1", 50))
        out.append(_vbox("cab_glass", (6.92, 1.3, R + 2.0), (0.04, 1.3, 0.8), "0.12 0.2 0.28 1", 15))
        out.append(_vbox("engine", (6.2, -0.5, R + 0.95), (1.6, 2.4, 1.5), dark))
        out.append(_vbox("bumper", (s.front - 0.3, 0, R - 0.1), (0.6, s.width - 0.4, 0.5), steel))
        out.append(_vbox("lidar_mast", (6.3, 0, 4.42), (0.2, 0.2, 0.36), dark))
        if loaded:
            out.append(_vbox("payload", (1.8, 0, R + 2.4), (6.8, s.width - 0.6, 1.4), "0.55 0.5 0.45 1", 35))
    else:
        out.append(_vbox("deck", (bx, 0, R + 0.55), (bl, s.width, 0.75), s.color, 50))
        out.append(_vbox("deck_rail_l", (bx, s.width / 2 - 0.1, R + 1.05), (bl, 0.2, 0.25), "0.95 0.75 0.1 1", 70))
        out.append(_vbox("deck_rail_r", (bx, -s.width / 2 + 0.1, R + 1.05), (bl, 0.2, 0.25), "0.95 0.75 0.1 1", 70))
        out.append(_vbox("container", (bx, 0, R + 2.3), (12.2, 2.44, 2.6), "0.70 0.18 0.14 1", 40))
        out.append(_vbox("mast", (9.85, 0, 3.55), (0.25, 0.25, 3.0), dark))
        for nm, (x, y) in {"bl": (-3.05, 1.55), "br": (-3.05, -1.55), "fl": (10.05, 1.55), "fr": (10.05, -1.55)}.items():
            out.append(_vbox(f"corner_pod_{nm}", (x, y, 1.75), (0.35, 0.35, 0.3), dark))
    if ego_dust:
        out.append(f'<particle_emitter name="ego_dust" type="box"><emitting>false</emitting>'
                   f"<pose>{_f(-s.rear - 1.5 - xc, 0, 0.9 - zc, 0, 0, 0)}</pose><size>2.5 {s.width:.1f} 1.2</size>"
                   "<particle_size>1.1 1.1 1.1</particle_size><lifetime>4.5</lifetime><rate>22</rate>"
                   "<min_velocity>0.2</min_velocity><max_velocity>0.7</max_velocity><scale_rate>0.9</scale_rate>"
                   f"<material><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular><pbr><metal><albedo_map>model://{world}/meshes/dust.png"
                   f"</albedo_map></metal></pbr></material><color_range_image>model://{world}/meshes/dust_colors.png</color_range_image>"
                   "<particle_scatter_ratio>0.06</particle_scatter_ratio></particle_emitter>")
    # ---- sensors (true extrinsics; the TF tree publishes the nominal ones)
    out.append(_lidar_sensor(s.lidar_top, "/iad/lidar_top"))
    if corner_lidars:
        for l in s.corners:
            out.append(_lidar_sensor(l, f"/iad/lidar_{l.name}"))
    out.append(f'<sensor name="imu" type="imu"><pose relative_to="__model__">{_f(*s.imu, 0, 0, 0)}</pose>'
               "<topic>/iad/imu</topic><gz_frame_id>imu_link</gz_frame_id><update_rate>100</update_rate><always_on>true</always_on>"
               "<imu><angular_velocity>"
               f"<x>{_noise(0.002, 0.0015)}</x><y>{_noise(0.002, 0.001)}</y><z>{_noise(0.002, 0.0035)}</z>"
               "</angular_velocity><linear_acceleration>"
               f"<x>{_noise(0.04, 0.02)}</x><y>{_noise(0.04, 0.03)}</y><z>{_noise(0.04, 0.01)}</z>"
               "</linear_acceleration></imu></sensor>")
    out.append(f'<sensor name="gnss" type="navsat"><pose relative_to="__model__">{_f(*s.gnss, 0, 0, 0)}</pose>'
               "<topic>/iad/gnss</topic><gz_frame_id>gnss_link</gz_frame_id><update_rate>10</update_rate><always_on>true</always_on>"
               # Ideal fix: gz applies horizontal NavSat noise in *degrees*; receiver errors come from gnss_effects.
               "<navsat><position_sensing><vertical><noise type=\"gaussian\"><mean>0</mean><stddev>0.01</stddev></noise>"
               "</vertical></position_sensing></navsat></sensor>")
    if cameras:
        out.append(f'<sensor name="chase_cam" type="camera"><pose relative_to="__model__">{_f(*s.chase)}</pose>'
                   "<topic>/iad/chase_cam</topic><update_rate>30</update_rate><always_on>true</always_on>"
                   "<camera><horizontal_fov>1.05</horizontal_fov><image><width>1920</width><height>1080</height></image>"
                   "<clip><near>0.5</near><far>1500</far></clip></camera>"
                   '<plugin filename="gz-sim-camera-video-recorder-system" name="gz::sim::systems::CameraVideoRecorder">'
                   "<service>/iad/chase_cam/record</service><use_sim_time>true</use_sim_time><fps>30</fps>"
                   "<bitrate>12000000</bitrate></plugin></sensor>")
    out.append("</link>")

    # ---- suspension + steering + wheels
    corners = {"fl": (L, T / 2, True), "fr": (L, -T / 2, True), "rl": (0.0, T / 2, False), "rr": (0.0, -T / 2, False)}
    for n, (x, y, steered) in corners.items():
        p = _f(x, y, R, 0, 0, 0)
        small = '<inertial><mass>60</mass><inertia><ixx>20</ixx><iyy>20</iyy><izz>20</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
        out.append(f'<link name="susp_{n}"><pose relative_to="__model__">{p}</pose>{small}</link>')
        out.append(f'<joint name="susp_{n}_joint" type="prismatic"><parent>chassis</parent><child>susp_{n}</child>'
                   '<axis><xyz expressed_in="__model__">0 0 1</xyz><limit><lower>-0.25</lower><upper>0.25</upper>'
                   f"<effort>1e8</effort></limit><dynamics><damping>{c:.0f}</damping><spring_stiffness>{k:.0f}</spring_stiffness>"
                   f"<spring_reference>{-sag}</spring_reference></dynamics></axis></joint>")
        parent = f"susp_{n}"
        if steered:
            out.append(f'<link name="steer_{n}"><pose relative_to="__model__">{p}</pose>{small}</link>')
            out.append(f'<joint name="steer_{n}_joint" type="revolute"><parent>susp_{n}</parent><child>steer_{n}</child>'
                       f'<axis><xyz expressed_in="__model__">0 0 1</xyz><limit><lower>{-s.max_steer - 0.2:.3f}</lower>'
                       f"<upper>{s.max_steer + 0.2:.3f}</upper><effort>1e7</effort><velocity>3</velocity></limit></axis></joint>")
            parent = f"steer_{n}"
        iyy = 0.5 * m_wheel * R * R
        ixx = m_wheel * (3 * R * R + s.tyre_width ** 2) / 12
        out.append(f'<link name="wheel_{n}"><pose relative_to="__model__">{p}</pose>'
                   f"<inertial><mass>{m_wheel}</mass><inertia><ixx>{ixx:.1f}</ixx><iyy>{iyy:.1f}</iyy><izz>{ixx:.1f}</izz>"
                   "<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
                   f'<collision name="tyre"><geometry><sphere><radius>{R}</radius></sphere></geometry>'
                   "<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>"
                   f'<visual name="tyre"><pose>0 0 0 1.5708 0 0</pose><laser_retro>8</laser_retro><geometry><cylinder>'
                   f"<radius>{R}</radius><length>{s.tyre_width}</length></cylinder></geometry>"
                   "<material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.08 0.08 0.08 1</diffuse></material></visual>"
                   f'<visual name="hub"><pose>0 {0.02 if y > 0 else -0.02} 0 1.5708 0 0</pose><geometry><cylinder>'
                   f"<radius>{0.55 * R:.3f}</radius><length>{s.tyre_width + 0.04:.3f}</length></cylinder></geometry>"
                   "<material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual></link>")
        out.append(f'<joint name="wheel_{n}_joint" type="revolute"><parent>{parent}</parent><child>wheel_{n}</child>'
                   f'<axis><xyz expressed_in="__model__">0 1 0</xyz><limit><effort>{hold:.0f}</effort></limit></axis></joint>')

    nominal_mass = s.mass_empty + (0.85 * s.payload if loaded else 0.0)
    out.append('<plugin filename="iad_gz_vehicle" name="iad_gz_sim::VehicleDrive">'
               "<rear_left_wheel>wheel_rl_joint</rear_left_wheel><rear_right_wheel>wheel_rr_joint</rear_right_wheel>"
               "<front_left_wheel>wheel_fl_joint</front_left_wheel><front_right_wheel>wheel_fr_joint</front_right_wheel>"
               "<front_left_steer>steer_fl_joint</front_left_steer><front_right_steer>steer_fr_joint</front_right_steer>"
               f"<wheelbase>{L}</wheelbase><track>{T}</track><wheel_radius>{R}</wheel_radius><nominal_mass>{nominal_mass:.0f}</nominal_mass>"
               f"<max_steer>{s.max_steer}</max_steer><max_steer_rate>{s.max_steer_rate}</max_steer_rate><steer_tau>{s.steer_tau}</steer_tau>"
               f"<delay>{delay}</delay><accel_tau>{s.accel_tau}</accel_tau><max_acc>{s.max_acc}</max_acc><max_dec>{s.max_dec}</max_dec>"
               f"<power>{s.power:.0f}</power><terrain_grid>{world}/terrain.grid</terrain_grid>"
               f"<odom_scale_error>{odom_scale_error}</odom_scale_error><publish_gt_tf>{1 if gt_tf else 0}</publish_gt_tf>"
               + (f"<dust_emitter_topic>/model/{model_name}/link/chassis/particle_emitter/ego_dust/cmd</dust_emitter_topic>" if ego_dust else "")
               + "</plugin>")
    out.append("</model></sdf>")
    return "\n".join(out)


def _quat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2), math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def sensor_frames(vehicle="haul_truck", corner_lidars=False, extrinsic_error_deg=0.0, extrinsic_error_m=0.0, seed=7):
    """(child_frame, xyz, rpy) of the *nominal* extrinsics (what the TF tree claims) and of the true ones.
    With extrinsic errors, every corner LiDAR gets roll/pitch/yaw errors of 0.5..1 x deg and x/y/z errors of
    0.5..1 x m with random signs (the top LiDAR is the reference)."""
    s = SPECS[vehicle]
    rng = random.Random(seed + 99)
    nominal, truth = [], []
    lidars = [s.lidar_top] + (s.corners if corner_lidars else [])
    for l in lidars:
        truth.append((f"lidar_{l.name}", l.xyz, l.rpy))
        if l.name == "top" or (extrinsic_error_deg <= 0 and extrinsic_error_m <= 0):
            nominal.append((f"lidar_{l.name}", l.xyz, l.rpy))
            continue
        e = [(1 if rng.random() < 0.5 else -1) * rng.uniform(0.5, 1.0) for _ in range(6)]
        rpy = tuple(l.rpy[i] + math.radians(extrinsic_error_deg) * e[i] for i in range(3))
        xyz = tuple(l.xyz[i] + extrinsic_error_m * e[3 + i] for i in range(3))
        nominal.append((f"lidar_{l.name}", xyz, rpy))
    for n, xyz in (("imu_link", s.imu), ("gnss_link", s.gnss)):
        nominal.append((n, xyz, (0.0, 0.0, 0.0)))
        truth.append((n, xyz, (0.0, 0.0, 0.0)))
    return nominal, truth


def bridge_config(world="quarry", vehicle="haul_truck", corner_lidars=False):
    def b(ros, gz, rt, gt, d="GZ_TO_ROS"):
        return (f'- ros_topic_name: "{ros}"\n  gz_topic_name: "{gz}"\n  ros_type_name: "{rt}"\n'
                f'  gz_type_name: "{gt}"\n  direction: {d}\n')
    out = [b("/clock", "/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock"),
           b("/sensing/lidar/top/points", "/iad/lidar_top/points", "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked"),
           b("/sensing/imu/data", "/iad/imu", "sensor_msgs/msg/Imu", "gz.msgs.IMU"),
           b("/sim/gnss/raw", "/iad/gnss", "sensor_msgs/msg/NavSatFix", "gz.msgs.NavSat"),
           b("/sim/traffic/poses", "/iad/traffic/poses", "tf2_msgs/msg/TFMessage", "gz.msgs.Pose_V")]
    if corner_lidars:
        for l in SPECS[vehicle].corners:
            out.append(b(f"/sensing/lidar/{l.name}/points", f"/iad/lidar_{l.name}/points",
                         "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked"))
    return "".join(out)
