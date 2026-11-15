"""Mission sequencer: drives a list of named site goals through the planner, with dwell times.

Goals and routes come from the site file (named poses + surveyed routes). A goal entry "route:<name>" sends
that route as nav_msgs/Path on /planning/route (route mode, e.g. the haul road); any other entry is a named
pose sent as geometry_msgs/PoseStamped on /planning/goal. Publishes /mission/status (JSON) for the HUD."""
import json
import math

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class Mission(Node):
    def __init__(self):
        super().__init__('mission')
        site = yaml.safe_load(open(self.declare_parameter('site_file', '').value))
        self.poses, self.routes = site.get('poses', {}), site.get('routes', {})
        self.goals = [g for g in str(self.declare_parameter('goals', 'dump').value).split(',') if g]
        self.dwell = self.declare_parameter('dwell', 4.0).value
        self.start_delay = self.declare_parameter('start_delay', 3.0).value
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.planner, self.idx, self.state, self.t_state = {}, -1, 'waiting', None
        self.create_subscription(String, '/planning/metrics', lambda m: self.planner.update(json.loads(m.data)), 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/planning/goal', 5)
        self.route_pub = self.create_publisher(Path, '/planning/route', latched)
        self.status_pub = self.create_publisher(String, '/mission/status', 5)
        self.create_timer(0.2, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def send(self, name):
        stamp = self.get_clock().now().to_msg()
        if name.startswith('route:'):
            # "route:<name>" or "route:<name>><pose>": the route, cut at the point nearest a named pose
            rname, _, until = name[6:].partition('>')
            pts = self.routes[rname]['points']
            if until:
                gx, gy = self.poses[until]['x'], self.poses[until]['y']
                k = min(range(len(pts)), key=lambda i: (pts[i][0] - gx) ** 2 + (pts[i][1] - gy) ** 2)
                pts = pts[:k + 1]
            p = Path()
            p.header.frame_id, p.header.stamp = 'map', stamp
            for i, (x, y) in enumerate(pts):
                j = min(i + 1, len(pts) - 1)
                k = max(j - 1, 0)
                yaw = math.atan2(pts[j][1] - pts[k][1], pts[j][0] - pts[k][0])
                ps = PoseStamped()
                ps.header = p.header
                ps.pose.position.x, ps.pose.position.y = float(x), float(y)
                ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                p.poses.append(ps)
            self.route_pub.publish(p)
        else:
            g = self.poses[name]
            m = PoseStamped()
            m.header.frame_id, m.header.stamp = 'map', stamp
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(g['x']), float(g['y']), float(g.get('z', 0.0))
            m.pose.orientation.z, m.pose.orientation.w = math.sin(g['yaw'] / 2), math.cos(g['yaw'] / 2)
            self.goal_pub.publish(m)
        self.get_logger().info(f'goal {self.idx + 1}/{len(self.goals)}: {name}')

    def tick(self):
        t = self.now()
        if self.t_state is None:
            self.t_state = t
        st = self.planner.get('state', 'idle')
        if self.state == 'waiting' and t - self.t_state > self.start_delay:
            self.idx, self.state, self.t_state = 0, 'driving', t
            self.send(self.goals[0])
        elif self.state == 'driving' and st == 'arrived' and t - self.t_state > 1.0:
            self.state, self.t_state = 'dwell', t
        elif self.state == 'dwell' and t - self.t_state > self.dwell:
            self.idx += 1
            if self.idx >= len(self.goals):
                self.state = 'done'
            else:
                self.state, self.t_state = 'driving', t
                self.send(self.goals[self.idx])
        name = self.goals[self.idx] if 0 <= self.idx < len(self.goals) else ''
        self.status_pub.publish(String(data=json.dumps({'state': self.state, 'goal': name.replace('route:', '').split('>')[0], 'index': self.idx,
                                                        'count': len(self.goals), 'planner': st, 'elapsed': t - self.t_state})))


def main():
    rclpy.init()
    rclpy.spin(Mission())


if __name__ == '__main__':
    main()
