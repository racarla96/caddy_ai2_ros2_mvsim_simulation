#!/usr/bin/env python3
"""
Conduce el Caddy por el circuito de MVSim (world:=road_circuit) con pure pursuit, usando SOLO la interfaz
unificada con Gazebo: lee /caddy_ai2/ground_truth/odometry y publica TwistStamped en
/caddy_ai2/bicycle_steering_controller/reference (funciona igual en Gazebo si el mundo es el mismo).

Ruta: 1) vuelta completa a las calzadas (en sentido antihorario, pasando por debajo del puente) y
2) subida por la colina interior hasta el tablero del puente (z = 4 m) y cruce por encima de la calzada
norte. Las esquinas de las calzadas (4 m de ancho) son demasiado cerradas para el radio de giro del Caddy
(~3,9 m): las toma con arcos de 6 m, saliendose un poco de la calzada.

    ros2 run caddy_ai2_ros2_mvsim_simulation drive_route.py          (o: python3 tools/drive_route.py)
    drive_route.py --route loop | bridge | all  --speed 2.5
"""
import argparse
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def arc(cx, cy, r, a0, a1, n=12):
    return [(cx + r * math.cos(math.radians(a)), cy + r * math.sin(math.radians(a)))
            for a in np.linspace(a0, a1, n)]


def line(p0, p1, step=1.0):
    n = max(2, int(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / step) + 1)
    return [(p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t) for t in np.linspace(0, 1, n)]


def build_route(which):
    R = 6.0
    pts = []
    if which in ('loop', 'all'):
        pts += line((0, 0), (20, 0))
        pts += arc(20, R, R, -90, 0)                    # esquina SE
        pts += line((26, R), (26, 36 - R))
        pts += arc(20, 36 - R, R, 0, 90)                # esquina NE
        pts += line((20, 36), (-20, 36))                # calzada norte: pasa por debajo del puente (x 2..6)
        pts += arc(-20, 36 - R, R, 90, 180)             # esquina NO
        pts += line((-26, 36 - R), (-26, R))
        pts += arc(-20, R, R, 180, 270)                 # esquina SO
        pts += line((-20, 0), (-2, 0))
    if which in ('bridge', 'all'):
        if which == 'bridge':
            pts += line((-8, 0), (-2, 0))
        pts += arc(-2, R, R, -90, 0)                    # giro al norte hacia x = 4
        pts += line((4, R), (4, 43))                    # colina -> tablero del puente (4 m) -> sobre la calzada norte
    # quitar duplicados consecutivos
    out = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > 0.05:
            out.append(p)
    return np.array(out)


class Driver(Node):
    def __init__(self, route, speed, hill_speed):
        super().__init__('drive_route')
        self.route, self.v0, self.v_hill = route, speed, hill_speed
        self.seg = np.r_[0, np.cumsum(np.hypot(*np.diff(route, axis=0).T))]
        self.idx, self.pose, self.done, self.t_last = 0, None, False, 0.0
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Odometry, '/caddy_ai2/ground_truth/odometry', self.on_odom, qos)
        self.pub = self.create_publisher(TwistStamped, '/caddy_ai2/bicycle_steering_controller/reference', 10)
        self.create_timer(0.05, self.step)
        self.get_logger().info('Ruta de %d puntos, %.0f m' % (len(route), self.seg[-1]))

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def step(self):
        if self.pose is None or self.done:
            return
        x, y, yaw = self.pose
        # punto mas cercano, solo hacia delante
        win = self.route[self.idx:self.idx + 60]
        self.idx += int(np.argmin(np.hypot(win[:, 0] - x, win[:, 1] - y)))
        s_here = self.seg[self.idx]
        on_hill = self.route[self.idx, 0] > -6 and self.route[self.idx, 1] > 3 and abs(self.route[self.idx, 0] - 4) < 3
        v = self.v_hill if on_hill else self.v0
        ld = float(np.clip(1.5 + 1.0 * v, 3.0, 7.0))
        j = int(np.searchsorted(self.seg, s_here + ld))
        j = min(j, len(self.route) - 1)
        tx, ty = self.route[j]
        if self.idx >= len(self.route) - 3 or (j == len(self.route) - 1 and math.hypot(tx - x, ty - y) < 1.2):
            self.pub.publish(self._msg(0.0, 0.0))
            self.done = True
            self.get_logger().info('Fin de ruta en (%.1f, %.1f)' % (x, y))
            return
        alpha = math.atan2(ty - y, tx - x) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        kappa = float(np.clip(2 * math.sin(alpha) / max(ld, 0.1), -0.22, 0.22))  # limite del volante (~0.256)
        v = min(v, 0.6 + 1.0 / (1.0 + 6 * abs(kappa)) * (v - 0.6)) if abs(kappa) > 0.08 else v
        self.pub.publish(self._msg(v, v * kappa))
        t = self.get_clock().now().nanoseconds * 1e-9
        if t - self.t_last > 3.0:
            self.t_last = t
            self.get_logger().info('(%.1f, %.1f) rumbo %.0f deg | progreso %.0f%% | v=%.1f'
                                   % (x, y, math.degrees(yaw), 100 * s_here / self.seg[-1], v))

    def _msg(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x, m.twist.angular.z = float(v), float(w)
        return m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--route', choices=('loop', 'bridge', 'all'), default='all')
    ap.add_argument('--speed', type=float, default=2.5, help='m/s en llano')
    ap.add_argument('--hill-speed', type=float, default=1.6, help='m/s en la colina y el puente')
    args = ap.parse_args()
    rclpy.init()
    node = Driver(build_route(args.route), args.speed, args.hill_speed)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(10):                      # unos mensajes de parada
            node.pub.publish(node._msg(0.0, 0.0))
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.pub.publish(node._msg(0.0, 0.0))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
