"""Launch test: the controller alone, closed loop on the package's software-in-the-loop plant (0.4 s actuator
dead time, rate-limited steering), no simulator and no other iad package. It must drive the demo S-curve,
stop, change gear and reverse into the bay with small tracking errors."""
import json
import time
import unittest

import os
os.environ["ROS_DOMAIN_ID"] = "44"  # isolated from other packages' tests running in parallel
import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from std_msgs.msg import String


@pytest.mark.launch_test
def generate_test_description():
    ctl = launch_ros.actions.Node(package="iad_control", executable="controller_node", name="controller",
                                  parameters=[{"use_sim_time": False}])
    plant = launch_ros.actions.Node(package="iad_control", executable="sil_plant",
                                    parameters=[{"publish_demo_trajectory": True}])
    return launch.LaunchDescription([ctl, plant, launch_testing.actions.ReadyToTest()])


class TestControllerSil(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_tracks_s_curve_and_reverse_park(self):
        node = rclpy.create_node("control_tester")
        last, seen_reverse = {}, []
        def on_m(m):
            j = json.loads(m.data)
            last["m"] = j
            if j["state"] == "reverse":
                seen_reverse.append(1)
        node.create_subscription(String, "/control/metrics", on_m, 20)
        end = time.time() + 90
        while time.time() < end and not (last.get("m", {}).get("docked")):
            rclpy.spin_once(node, timeout_sec=0.1)
        m = last.get("m")
        self.assertIsNotNone(m, "no controller metrics")
        self.assertTrue(m["docked"], f"did not finish: {m}")
        self.assertTrue(seen_reverse, "never drove the reverse segment")
        self.assertLess(m["lat_rms"], 0.3, m)
        self.assertLess(abs(m["dock_lat"]), 0.5, m)
        self.assertLess(abs(m["dock_long"]), 1.0, m)
        node.destroy_node()
