"""Launch test: the perception node alone, fed with synthetic standard messages (no simulator, no other
iad package). A flat ground plane with a 3 m box ahead and a speckle of dust: the box must end up in the
occupancy grid and in DetectedObjects, the ground in the ground cloud, the dust out of the grid."""
import math
import struct
import time
import unittest

import os
os.environ["ROS_DOMAIN_ID"] = "41"  # isolated from other packages' tests running in parallel
import launch
import launch_ros.actions
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from autoware_perception_msgs.msg import DetectedObjects
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(package="iad_perception", executable="perception_node", name="perception",
                                   parameters=[{"use_sim_time": False}])
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


def make_scan(rng, stamp):
    rings, cols, h = 32, 1200, 4.15
    el = np.radians(np.linspace(-25, 15, rings))[:, None]
    az = (np.arange(cols) + 0.5) / cols * 2 * math.pi - math.pi
    az = az[None, :]
    dx, dy, dz = np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.broadcast_to(np.sin(el), (rings, cols))
    r = np.full((rings, cols), np.inf)
    down = dz < -1e-3
    r[down] = h / -dz[down]  # flat ground 4.15 m below the sensor
    # box: x in [20, 23], y in [-3, 3], z up to 3 m above ground (sensor frame z in [-4.15, -1.15])
    with np.errstate(divide="ignore", invalid="ignore"):
        t = 20.0 / dx
        y, z = t * dy, t * dz
        box = (dx > 0) & (np.abs(y) < 3) & (z > -h) & (z < -h + 3) & (t < r)
    r[box] = t[box]
    r[r > 80] = np.inf
    dust = (rng.random((rings, cols)) < 0.5) & (np.abs(az) < 0.5) & (dz > -0.05)
    r[dust] = rng.uniform(5, 12, dust.sum())
    pts = np.stack([r * dx, r * dy, r * dz, np.full_like(r, 40.0)], axis=-1).astype(np.float32)
    msg = PointCloud2()
    msg.header.frame_id, msg.header.stamp = "lidar_top", stamp
    msg.height, msg.width, msg.point_step, msg.row_step = rings, cols, 16, 16 * cols
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(("x", "y", "z", "intensity"))]
    msg.is_dense = False
    msg.data = pts.tobytes()
    return msg


class TestPerceptionNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_obstacle_reaches_grid_and_objects(self):
        node = rclpy.create_node("perception_tester")
        got = {}
        node.create_subscription(OccupancyGrid, "/perception/occupancy_grid", lambda m: got.__setitem__("grid", m), 5)
        node.create_subscription(DetectedObjects, "/perception/objects", lambda m: got.__setitem__("objs", m), 5)
        node.create_subscription(PointCloud2, "/perception/points/ground", lambda m: got.__setitem__("ground", m),
                                 qos_profile_sensor_data)
        pub = node.create_publisher(PointCloud2, "/sensing/lidar/top/points", 5)
        stf, tfb = StaticTransformBroadcaster(node), TransformBroadcaster(node)
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = "base_link", "lidar_top"
        t.transform.translation.x, t.transform.translation.z, t.transform.rotation.w = 5.2, 4.15, 1.0
        stf.sendTransform(t)
        rng = np.random.default_rng(3)
        end = time.time() + 25
        frames = 0
        while time.time() < end and not (frames > 12 and all(k in got for k in ("grid", "objs", "ground"))):
            now = node.get_clock().now().to_msg()
            m = TransformStamped()
            m.header.frame_id, m.header.stamp, m.child_frame_id = "map", now, "base_link"
            m.transform.rotation.w = 1.0
            tfb.sendTransform(m)
            time.sleep(0.02)
            pub.publish(make_scan(rng, now))
            frames += 1
            rclpy.spin_once(node, timeout_sec=0.1)
        self.assertIn("grid", got, "no occupancy grid")
        self.assertIn("objs", got, "no DetectedObjects")
        g = got["grid"]
        data = np.array(g.data, dtype=np.int8).reshape(g.info.height, g.info.width)

        def cell(x, y):
            i = int((x - g.info.origin.position.x) / g.info.resolution)
            j = int((y - g.info.origin.position.y) / g.info.resolution)
            return data[j, i]
        # box face at x = 5.2 + 20 in base/map frame
        occupied = max(cell(25.3, y) for y in np.arange(-2.5, 2.5, 0.5))
        self.assertEqual(occupied, 100, "box face not occupied")
        # dust region (8..17 m ahead, sensor height): must not be lethal
        dust_cells = [cell(x, y) for x in np.arange(11, 21, 0.5) for y in np.arange(-3, 3, 0.5)]
        self.assertLess(sum(c == 100 for c in dust_cells), 3, "dust leaked into the grid")
        self.assertGreater(got["ground"].width, 5000)
        self.assertTrue(any(abs(o.kinematics.pose_with_covariance.pose.position.x - 26.7) < 3.0
                            for o in got["objs"].objects), "box not among DetectedObjects")
        node.destroy_node()
