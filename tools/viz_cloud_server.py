#!/usr/bin/env python3
"""Publica EN LOCAL la nube 3D analitica del circuito (maps/road_circuit_bonxai.pcd) en /viz_cloud (frame map).

Sirve para orientarse en un RViz remoto (navigation_3d_remote.rviz): no cruza el tunel SSH ni depende de la
simulacion. QoS transient-local: se publica una vez. Ctrl+C para parar.

Uso:  viz_cloud_server.py [ruta.pcd] [--max-z 6]
"""
import argparse
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def load_pcd_xyz(path):
    with open(path, 'rb') as f:
        fields = []
        while True:
            line = f.readline().decode().strip()
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('DATA'):
                if line.split()[1] != 'binary' or fields[:3] != ['x', 'y', 'z']:
                    raise SystemExit('Solo PCD binario con campos x y z')
                break
        data = f.read()
    return np.frombuffer(data, dtype=np.float32).reshape(-1, len(fields))[:, :3].copy()


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd', nargs='?', default=os.path.join(here, '..', 'maps', 'road_circuit_bonxai.pcd'))
    ap.add_argument('--max-z', type=float, default=6.0, help='descarta puntos por encima (cielo/bordes del demo)')
    a, ros_args = ap.parse_known_args()

    pts = load_pcd_xyz(a.pcd)
    pts = pts[pts[:, 2] <= a.max_z]

    rclpy.init(args=ros_args)
    node = Node('viz_cloud_server')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(PointCloud2, 'viz_cloud', qos)
    msg = PointCloud2()
    msg.header.frame_id = 'map'
    msg.height, msg.width = 1, len(pts)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate('xyz')]
    msg.point_step, msg.row_step = 12, 12 * len(pts)
    msg.is_dense = True
    msg.data = pts.tobytes()
    pub.publish(msg)
    node.get_logger().info('/viz_cloud: %d puntos de %s' % (len(pts), os.path.basename(a.pcd)))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
