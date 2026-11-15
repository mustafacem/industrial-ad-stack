"""Log JSON metric topics (std_msgs/String) to one JSONL file: {"topic": ..., "stamp": ..., "data": {...}}."""
import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')
        topics = self.declare_parameter('topics', ['/sim/debug']).value
        path = self.declare_parameter('path', '/ws/output/logs/run.jsonl').value
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.f = open(path, 'w')
        for t in topics:
            self.create_subscription(String, t, lambda m, t=t: self.on(t, m), 50)
        self.get_logger().info(f'logging {len(topics)} topics to {path}')

    def on(self, topic, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self.f.write(json.dumps({'topic': topic, 'stamp': now, 'data': data}) + '\n')
        self.f.flush()


def main():
    rclpy.init()
    rclpy.spin(MetricsLogger())


if __name__ == '__main__':
    main()
