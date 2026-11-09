#!/usr/bin/env python3
"""Adapta EN LOCAL el URDF que llega del laboratorio para que RViz encuentre las mallas.

El URDF trae rutas absolutas del equipo donde corre la simulacion (file:///home/rafael/.../install/<pkg>/...).
Este nodo escucha /caddy_ai2/robot_description_rviz (puente zenoh), sustituye el prefijo ".../install/" por el
install local y republica en /caddy_ai2/robot_description_local (transient-local). RViz usa este ultimo.
Ctrl+C para parar.
"""
import os
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def local_install():
    for p in os.environ.get('AMENT_PREFIX_PATH', '').split(':'):
        if os.path.basename(os.path.dirname(p)) == 'install':
            return os.path.dirname(p)
    raise SystemExit('Haz source del install local antes de lanzar (AMENT_PREFIX_PATH sin install/)')


def main():
    rclpy.init()
    node = Node('viz_robot_model')
    inst = local_install()
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(String, '/caddy_ai2/robot_description_local', qos)

    def cb(msg):
        out = String()
        out.data = re.sub(r'(file://)?/[^"\s<>]*?/install/', lambda m: (m.group(1) or '') + inst + '/', msg.data)
        pub.publish(out)
        node.get_logger().info('URDF republicado con install local %s' % inst)

    node.create_subscription(String, '/caddy_ai2/robot_description_rviz', cb, qos)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
