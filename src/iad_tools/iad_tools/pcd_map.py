"""Publishes a site's point-cloud map (PCD, x y z [intensity]) latched on /map/pointcloud_map for RViz context
in demos that run without iad_localization (which publishes the map itself). Voxel-thinned: RViz renders in
software when recording."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField


def read_pcd(path):
    with open(path, 'rb') as f:
        fields, n = [], 0
        while True:
            line = f.readline().decode().strip()
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('POINTS'):
                n = int(line.split()[1])
            elif line.startswith('DATA'):
                assert line.split()[1] == 'binary', 'binary PCD only'
                break
        data = np.frombuffer(f.read(n * 4 * len(fields)), dtype=np.float32).reshape(n, len(fields))
    return data[:, [fields.index(c) for c in ('x', 'y', 'z')]]


def main():
    rclpy.init()
    node = Node('pcd_map')
    pts = read_pcd(node.declare_parameter('pcd_file', '').value)
    voxel = node.declare_parameter('voxel', 1.0).value
    if voxel > 0:
        _, keep = np.unique(np.floor(pts / voxel).astype(np.int64), axis=0, return_index=True)
        pts = pts[np.sort(keep)]
    m = PointCloud2()
    m.header.frame_id = 'map'
    m.height, m.width, m.point_step, m.row_step, m.is_dense = 1, len(pts), 12, 12 * len(pts), True
    m.fields = [PointField(name=c, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, c in enumerate('xyz')]
    m.data = np.ascontiguousarray(pts, dtype=np.float32).tobytes()
    pub = node.create_publisher(PointCloud2, '/map/pointcloud_map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pub.publish(m)
    node.get_logger().info(f'published {len(pts)} map points')
    rclpy.spin(node)


if __name__ == '__main__':
    main()
