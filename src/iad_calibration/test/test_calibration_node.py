"""Calibrate one corner LiDAR using only synthetic clouds, odometry and nominal static TF."""
import json
import math
import os
import time
import unittest

os.environ["ROS_DOMAIN_ID"] = "45"  # isolated from other packages' launch tests
import launch
import launch_ros.actions
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster


@pytest.mark.launch_test
def generate_test_description():
    calibration = launch_ros.actions.Node(
        package="iad_calibration", executable="calibration_node", name="calibration",
        parameters=[{
            "lidars": ["front_left"], "reference": "top", "ground_scans": 3,
            "keyframe_spacing": 2.0, "max_keyframes": 15, "max_iterations": 25,
            "max_iterations_6dof": 15, "iteration_period": 0.05,
            "odom_topic": "/odom_test", "use_sim_time": False,
            "vehicle.front": 2.0, "vehicle.rear": 1.0,
            "vehicle.width": 2.0, "vehicle.height": 2.0,
        }])
    return launch.LaunchDescription([calibration, launch_testing.actions.ReadyToTest()])


def rotation(rpy):
    roll, pitch, yaw = rpy
    cr, cp, cy = np.cos([roll, pitch, yaw])
    sr, sp, sy = np.sin([roll, pitch, yaw])
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def static_transform(frame, xyz, rpy, stamp):
    tf = TransformStamped()
    tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, "base_link", frame
    tf.transform.translation.x = float(xyz[0])
    tf.transform.translation.y = float(xyz[1])
    tf.transform.translation.z = float(xyz[2])
    cr, cp, cy = np.cos(0.5 * np.asarray(rpy))
    sr, sp, sy = np.sin(0.5 * np.asarray(rpy))
    q = tf.transform.rotation
    q.x, q.y = float(sr * cp * cy - cr * sp * sy), float(cr * sp * cy + sr * cp * sy)
    q.z, q.w = float(cr * cp * sy - sr * sp * cy), float(cr * cp * cy + sr * sp * sy)
    return tf


def scene_points():
    # Dense planar patches give each one-metre map voxel enough points for a surfel.
    x, y = np.meshgrid(np.arange(-15.1, 75.0, 0.4), np.arange(-20.1, 20.0, 0.4))
    patches = [np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))]

    def wall(start, end):
        length = np.linalg.norm(np.asarray(end) - start)
        s, z = np.meshgrid(np.arange(0.0, length, 0.3), np.arange(1.2, 6.0, 0.3))
        xy = np.asarray(start) + s.ravel()[:, None] * (np.asarray(end) - start) / length
        patches.append(np.column_stack((xy, z.ravel())))

    wall((-15.0, 12.3), (75.0, 12.3))
    wall((-15.0, -12.3), (75.0, -12.3))
    for x_wall in (12.3, 28.3, 44.3, 60.3):
        wall((x_wall, 5.0), (x_wall, 10.0))
    wall((18.0, -5.0), (23.0, -10.0))
    wall((38.0, 5.0), (43.0, 10.0))
    wall((54.0, -5.0), (59.0, -10.0))
    return np.vstack(patches)


def cloud(world, vehicle_x, xyz, rpy, frame, stamp, forward_only=False):
    # Row-vector form of inverse(T_map_base * T_base_sensor), using the TRUE extrinsic.
    pts = (world - np.array([vehicle_x, 0.0, 0.0]) - xyz) @ rotation(rpy)
    ranges = np.linalg.norm(pts, axis=1)
    keep = (ranges > 3.0) & (ranges < 38.0)
    if forward_only:
        keep &= pts[:, 0] > 0.0
    pts = np.asarray(pts[keep], dtype="<f4")
    msg = PointCloud2()
    msg.header.stamp, msg.header.frame_id = stamp, frame
    msg.height, msg.width = 1, len(pts)
    msg.fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(("x", "y", "z"))]
    msg.is_bigendian, msg.is_dense = False, True
    msg.point_step, msg.row_step = 12, 12 * len(pts)
    msg.data = pts.tobytes()
    return msg


class TestCalibrationNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_recovers_extrinsic(self):
        node = rclpy.create_node("calibration_tester")
        try:
            latest, states = {}, set()

            def on_metrics(msg):
                latest["metrics"] = json.loads(msg.data)
                states.add(latest["metrics"]["state"])

            node.create_subscription(String, "/calibration/metrics", on_metrics, 20)
            odom_pub = node.create_publisher(Odometry, "/odom_test", 20)
            top_pub = node.create_publisher(
                PointCloud2, "/sensing/lidar/top/points", qos_profile_sensor_data)
            corner_pub = node.create_publisher(
                PointCloud2, "/sensing/lidar/front_left/points", qos_profile_sensor_data)
            broadcaster = StaticTransformBroadcaster(node)
            xyz_true, rpy_true = np.array([2.0, 1.0, 1.9]), np.deg2rad([3.0, 8.0, 40.0])
            top_xyz, top_rpy = np.array([0.0, 0.0, 3.0]), np.zeros(3)
            broadcaster.sendTransform([
                static_transform("lidar_top", top_xyz, top_rpy, node.get_clock().now().to_msg()),
                static_transform("lidar_front_left", xyz_true + [0.18, -0.10, 0.06],
                                 rpy_true + np.deg2rad([0.6, -0.4, 2.0]),
                                 node.get_clock().now().to_msg()),
            ])
            world = scene_points()
            deadline = time.monotonic() + 100.0

            def spin_for(seconds):
                end = min(time.monotonic() + seconds, deadline)
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=max(0.0, min(0.02, end - time.monotonic())))

            def publish_odom(x, velocity):
                msg = Odometry()
                msg.header.stamp, msg.header.frame_id = node.get_clock().now().to_msg(), "map"
                msg.child_frame_id = "base_link"
                msg.pose.pose.position.x, msg.pose.pose.orientation.w = float(x), 1.0
                msg.twist.twist.linear.x = float(velocity)
                odom_pub.publish(msg)

            def scan_pair(x):
                stamp = node.get_clock().now().to_msg()
                return (cloud(world, x, top_xyz, top_rpy, "lidar_top", stamp),
                        cloud(world, x, xyz_true, rpy_true, "lidar_front_left", stamp, True))

            # Start parked, allowing DDS discovery and the ground-plane stage to finish.
            parked = scan_pair(0.0)
            parked_until = time.monotonic() + 2.0
            ground_deadline = time.monotonic() + 15.0
            while (time.monotonic() < parked_until or
                   latest.get("metrics", {}).get("state") != "collect"):
                self.assertLess(time.monotonic(), ground_deadline,
                                f"ground stage did not finish: {latest}")
                publish_odom(0.0, 0.0)
                top_pub.publish(parked[0])
                corner_pub.publish(parked[1])
                spin_for(0.1)

            # At 3 m/s, snapshots 0.7 s apart are 2.1 m apart. Preload both scans before
            # advancing odometry: the node stores latest messages rather than synchronizing them.
            for index in range(1, 16):
                x = 2.1 * index
                scans = scan_pair(x)
                sample_start = time.monotonic()
                for _ in range(3):
                    top_pub.publish(scans[0])
                    corner_pub.publish(scans[1])
                    spin_for(0.05)
                while (time.monotonic() - sample_start < 0.7 or
                       latest.get("metrics", {}).get("keyframes", 0) < index):
                    self.assertLess(time.monotonic(), deadline,
                                    f"keyframe {index} was not collected: {latest}")
                    publish_odom(x, 3.0)
                    top_pub.publish(scans[0])
                    corner_pub.publish(scans[1])
                    spin_for(0.1)

            while (time.monotonic() < deadline and
                   latest.get("metrics", {}).get("state") != "done"):
                publish_odom(31.5, 0.0)
                spin_for(0.1)

            metrics = latest.get("metrics")
            self.assertIsNotNone(metrics, "no calibration metrics")
            self.assertEqual(metrics["state"], "done", metrics)
            self.assertTrue({"register", "refine"}.issubset(states), states)
            self.assertEqual(metrics["keyframes"], 15, metrics)
            estimate = metrics["lidars"]["front_left"]
            self.assertGreater(estimate["matched"], 50, metrics)
            self.assertLess(np.linalg.norm(np.asarray(estimate["xyz"]) - xyz_true), 0.05, metrics)
            delta_rotation = rotation(estimate["rpy"]).T @ rotation(rpy_true)
            angle = math.acos(float(np.clip((np.trace(delta_rotation) - 1.0) / 2.0, -1.0, 1.0)))
            self.assertLess(math.degrees(angle), 0.5, metrics)
        finally:
            node.destroy_node()
