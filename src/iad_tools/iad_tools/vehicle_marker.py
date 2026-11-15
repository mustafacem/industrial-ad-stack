"""Ego body for RViz (/viz/ego): a box in base_link (rear axle on the ground) plus the roof-LiDAR puck,
frame-locked so it moves with whatever publishes map -> base_link."""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


def main():
    rclpy.init()
    n = Node('vehicle_marker')
    front, rear = n.declare_parameter('front', 6.6).value, n.declare_parameter('rear', 2.8).value
    width, height = n.declare_parameter('width', 4.2).value, n.declare_parameter('height', 4.4).value
    color = n.declare_parameter('color', [0.95, 0.68, 0.12]).value
    lidar = n.declare_parameter('lidar', [6.3, 0.0, 4.75]).value
    pub = n.create_publisher(MarkerArray, '/viz/ego', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    body = Marker()
    body.header.frame_id, body.ns, body.id, body.type, body.frame_locked = 'base_link', 'ego', 0, Marker.CUBE, True
    body.pose.position.x, body.pose.position.z = (front - rear) / 2, 0.9 + (height - 0.9) / 2
    body.pose.orientation.w = 1.0
    body.scale.x, body.scale.y, body.scale.z = front + rear, width, height - 0.9
    body.color.r, body.color.g, body.color.b, body.color.a = float(color[0]), float(color[1]), float(color[2]), 0.85
    puck = Marker()
    puck.header.frame_id, puck.ns, puck.id, puck.type, puck.frame_locked = 'base_link', 'ego', 1, Marker.CYLINDER, True
    puck.pose.position.x, puck.pose.position.y, puck.pose.position.z = float(lidar[0]), float(lidar[1]), float(lidar[2])
    puck.pose.orientation.w = 1.0
    puck.scale.x = puck.scale.y = 0.6
    puck.scale.z = 0.4
    puck.color.r = puck.color.g = puck.color.b = 0.15
    puck.color.a = 1.0
    pub.publish(MarkerArray(markers=[body, puck]))
    rclpy.spin(n)


if __name__ == '__main__':
    main()
