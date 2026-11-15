"""Launch test: the localization node alone with synthetic standard messages (no simulator, no other iad
package). A container-yard map is written to a PCD file; the vehicle stands still at an unknown pose and
sees that yard; GNSS is a SINGLE fix 1.2 m off. The node must initialise itself (GNSS + NDT yaw search)
and converge to the true pose."""
import math
import os
import struct
import tempfile
import time
import unittest

import os
os.environ["ROS_DOMAIN_ID"] = "42"  # isolated from other packages' tests running in parallel
import launch
import launch_ros.actions
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster

LAT0, LON0 = 35.6206, 139.79
TRUE = (12.0, -5.0, 0.6)  # x, y, yaw of base_link
LIDAR = (9.85, 0.0, 5.05)
ANT = (9.85, 0.45, 5.3)


def yard(rng):
    pts = []
    gx, gy = np.meshgrid(np.arange(-80, 80, 1.0), np.arange(-80, 80, 1.0))
    pts.append(np.stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)], 1))
    boxes = [(-30, 20, 0.0), (5, 30, 0.3), (40, 10, 1.2), (30, -35, 0.0), (-20, -25, 0.8), (-45, -5, 1.57), (55, 40, 0.4)]
    for cx, cy, yaw in boxes:
        for _ in range(1):
            u = rng.uniform(-6.1, 6.1, 3000)
            v = rng.uniform(-1.22, 1.22, 3000)
            z = rng.uniform(0, 5.2, 3000)
            side = rng.integers(0, 4, 3000)
            lx = np.where(side == 0, 6.1, np.where(side == 1, -6.1, u))
            ly = np.where(side == 2, 1.22, np.where(side == 3, -1.22, v))
            c, s = math.cos(yaw), math.sin(yaw)
            pts.append(np.stack([cx + c * lx - s * ly, cy + s * lx + c * ly, z], 1))
    return np.concatenate(pts).astype(np.float32)


def write_pcd(path, p):
    with open(path, "wb") as f:
        f.write(("VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
                 f"WIDTH {len(p)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(p)}\nDATA binary\n").encode())
        f.write(np.concatenate([p, np.full((len(p), 1), 50, np.float32)], 1).tobytes())


TMP = tempfile.mkdtemp()
MAP = yard(np.random.default_rng(1))
write_pcd(os.path.join(TMP, "map.pcd"), MAP)


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package="iad_localization", executable="localization_node", name="localization",
        parameters=[{"map_file": os.path.join(TMP, "map.pcd"), "map_origin.latitude": LAT0,
                     "map_origin.longitude": LON0, "use_sim_time": False}])
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


def scan_msg(stamp):
    x, y, yaw = TRUE
    c, s = math.cos(yaw), math.sin(yaw)
    lx, ly = x + c * LIDAR[0] - s * LIDAR[1], y + s * LIDAR[0] + c * LIDAR[1]
    d = MAP - np.array([lx, ly, LIDAR[2]], np.float32)
    keep = np.linalg.norm(d, axis=1) < 60
    d = d[keep]
    local = np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], d[:, 2]], 1).astype(np.float32)
    pts = np.concatenate([local, np.full((len(local), 1), 50, np.float32)], 1)
    m = PointCloud2()
    m.header.frame_id, m.header.stamp = "lidar_top", stamp
    m.height, m.width, m.point_step, m.row_step = 1, len(pts), 16, 16 * len(pts)
    m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
    m.fields.append(PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1))
    m.data = pts.tobytes()
    return m


class TestLocalizationNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_global_init_and_convergence(self):
        node = rclpy.create_node("localization_tester")
        last = {}
        node.create_subscription(Odometry, "/localization/kinematic_state", lambda m: last.__setitem__("odom", m), 10)
        pubs = {"cloud": node.create_publisher(PointCloud2, "/sensing/lidar/top/points", 5),
                "imu": node.create_publisher(Imu, "/sensing/imu/data", 50),
                "gnss": node.create_publisher(NavSatFix, "/sensing/gnss/fix", 5),
                "vel": node.create_publisher(TwistWithCovarianceStamped, "/vehicle/status/velocity", 10)}
        stf = StaticTransformBroadcaster(node)
        tfs = []
        for child, xyz in (("lidar_top", LIDAR), ("gnss_link", ANT)):
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = "base_link", child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
            t.transform.rotation.w = 1.0
            tfs.append(t)
        stf.sendTransform(tfs)
        x, y, yaw = TRUE
        c, s = math.cos(yaw), math.sin(yaw)
        ax, ay = x + c * ANT[0] - s * ANT[1] + 0.9, y + s * ANT[0] + c * ANT[1] - 0.8  # 1.2 m GNSS error
        end, k = time.time() + 40, 0
        while time.time() < end:
            now = node.get_clock().now().to_msg()
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = now, "imu_link"
            imu.linear_acceleration.z = 9.81
            pubs["imu"].publish(imu)
            v = TwistWithCovarianceStamped()
            v.header.stamp = now
            pubs["vel"].publish(v)
            if k % 10 == 0:
                f = NavSatFix()
                f.header.stamp, f.header.frame_id = now, "gnss_link"
                f.status.status = NavSatStatus.STATUS_FIX
                f.latitude = LAT0 + ay / 111132.954
                f.longitude = LON0 + ax / (111319.49 * math.cos(math.radians(LAT0)))
                f.altitude = ANT[2]
                f.position_covariance = [1.5, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 4.0]
                pubs["gnss"].publish(f)
                pubs["cloud"].publish(scan_msg(now))
            k += 1
            rclpy.spin_once(node, timeout_sec=0.01)
            if "odom" in last and k > 300:
                p = last["odom"].pose.pose
                if math.hypot(p.position.x - x, p.position.y - y) < 0.15:
                    break
        self.assertIn("odom", last, "localization never initialised")
        p = last["odom"].pose.pose
        q = p.orientation
        est_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        err = math.hypot(p.position.x - x, p.position.y - y)
        self.assertLess(err, 0.3, f"position error {err:.2f} m")
        self.assertLess(abs(math.remainder(est_yaw - yaw, 2 * math.pi)), math.radians(2.0))
        node.destroy_node()
