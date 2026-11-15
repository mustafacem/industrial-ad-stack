"""Launch test: the planner alone with a synthetic site map file and standard messages (no simulator, no
other iad package). A 120 x 80 m yard with a wall; the truck must get a collision-free trajectory around
it to a goal, and a reverse-parking goal must produce reverse (negative-velocity) points."""
import math
import os
import tempfile
import time
import unittest

import os
os.environ["ROS_DOMAIN_ID"] = "43"  # isolated from other packages' tests running in parallel
import launch
import launch_ros.actions
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from autoware_planning_msgs.msg import Trajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

TMP = tempfile.mkdtemp()
RES, W, H, OX, OY = 0.5, 240, 160, -20.0, -40.0
grid = np.zeros((H, W), np.uint8)  # grid[j, i], j = y index (bottom-up)
for j in range(H):
    y = OY + (j + 0.5) * RES
    for i in range(W):
        x = OX + (i + 0.5) * RES
        if 38 < x < 41 and -40 < y < 18:
            grid[j, i] = 100  # wall with a gap at the top
        if y > 38 or y < -38 or x < -18 or x > 98:
            grid[j, i] = 100  # boundary
with open(os.path.join(TMP, "costmap.pgm"), "wb") as f:
    f.write(f"P5\n{W} {H}\n255\n".encode() + grid[::-1].tobytes())
with open(os.path.join(TMP, "costmap.yaml"), "w") as f:
    f.write(f"image: costmap.pgm\nmode: raw\nresolution: {RES}\norigin: [{OX}, {OY}, 0.0]\nnegate: 0\n")


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(package="iad_planning", executable="planner_node", name="planner",
                                   parameters=[{"costmap_file": os.path.join(TMP, "costmap.yaml"), "use_sim_time": False}])
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


def occupied(x, y):
    i, j = int((x - OX) / RES), int((y - OY) / RES)
    return not (0 <= i < W and 0 <= j < H) or grid[j, i] >= 100


class TestPlannerNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def run_goal(self, node, got, start, goal):
        odom_pub = node.create_publisher(Odometry, "/localization/kinematic_state", 10)
        goal_pub = node.create_publisher(PoseStamped, "/planning/goal", 5)
        got.clear()
        end, sent = time.time() + 20, False
        while time.time() < end and "traj" not in got:
            o = Odometry()
            o.header.frame_id, o.header.stamp = "map", node.get_clock().now().to_msg()
            o.pose.pose.position.x, o.pose.pose.position.y = start[0], start[1]
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(start[2] / 2), math.cos(start[2] / 2)
            odom_pub.publish(o)
            if not sent and time.time() > end - 18:
                g = PoseStamped()
                g.header.frame_id = "map"
                g.pose.position.x, g.pose.position.y = goal[0], goal[1]
                g.pose.orientation.z, g.pose.orientation.w = math.sin(goal[2] / 2), math.cos(goal[2] / 2)
                goal_pub.publish(g)
                sent = True
            rclpy.spin_once(node, timeout_sec=0.1)
        self.assertIn("traj", got, "no trajectory")
        return got["traj"]

    def test_plans_around_wall_and_reverses(self):
        node = rclpy.create_node("planner_tester")
        got = {}
        node.create_subscription(Trajectory, "/planning/trajectory",
                                 lambda m: got.__setitem__("traj", m) if len(m.points) > 5 else None, 5)
        t = self.run_goal(node, got, (0.0, -25.0, 0.0), (75.0, -25.0, 0.0))
        pts = [(p.pose.position.x, p.pose.position.y) for p in t.points]
        last = t.points[-1].pose.position
        self.assertLess(math.hypot(last.x - 75.0, last.y + 25.0), 1.0)
        self.assertFalse(any(occupied(x, y) for x, y in pts), "path crosses an obstacle")
        self.assertGreater(max(y for _, y in pts), 18.0, "path must go through the gap above the wall")
        # Reverse into a bay: goal facing away from the approach direction
        t2 = self.run_goal(node, got, (75.0, -25.0, 0.0), (60.0, -25.0, 0.0))
        self.assertTrue(any(p.longitudinal_velocity_mps < -0.05 for p in t2.points), "no reverse segment")
        node.destroy_node()
