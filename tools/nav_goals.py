#!/usr/bin/env python3
"""
Manda a EasyNav una secuencia de objetivos en el circuito (world:=road_circuit) y espera a que llegue a cada uno.

Solo usa la interfaz unificada y EasyNav: /initialpose y /goal_pose (entrada), /planner_node/astar/path
(para saber si hay camino) y /caddy_ai2/ground_truth/odometry (para saber si ha llegado).

    nav_goals.py [--start-at 200]        # segundos de simulacion a partir de los cuales manda el primero

Un objetivo se da por conseguido a menos de --arrive m (en x, y); se abandona si A* no da camino durante
--no-path-s s seguidos o si se supera --timeout s.
"""
import argparse
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

# (x, y, z, descripcion). z solo importa donde hay varias capas (el tablero del puente).
GOALS = [
    (26.0, 12.0, 0.0, 'calzada este'),
    (26.0, 32.0, 0.0, 'esquina noreste'),
    (8.0, 36.0, 0.0, 'calzada norte, antes del puente'),
    (-24.0, 32.0, 0.0, 'pasa por debajo del puente hasta la esquina noroeste'),
    (-26.0, 8.0, 0.0, 'calzada oeste'),
    (-4.0, 1.0, 0.0, 'calzada sur, al pie de la colina'),
    (4.0, 24.0, 3.75, 'sube la colina'),
    (4.0, 42.0, 4.0, 'cruza el tablero del puente'),
]


def yaw_q(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


class Goals(Node):
    def __init__(self, args):
        super().__init__('nav_goals')
        self.args, self.clock, self.pos, self.yaw, self.path_len, self.path_t = args, 0.0, None, 0.0, -1, 0.0
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Clock, '/clock', lambda m: setattr(self, 'clock', m.clock.sec + m.clock.nanosec * 1e-9), be)
        self.create_subscription(Odometry, '/caddy_ai2/ground_truth/odometry', self.on_gt, be)
        self.create_subscription(Path, '/planner_node/astar/path', self.on_path, 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.buf = Buffer()
        TransformListener(self.buf, self)

    def on_gt(self, m):
        self.pos = (m.pose.pose.position.x, m.pose.pose.position.y)
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_path(self, m):
        self.path_len, self.path_t = len(m.poses), self.clock

    def spin_for(self, secs):
        end = self.clock + secs
        t0 = self.get_clock().now().nanoseconds
        while rclpy.ok() and self.clock < end and (self.get_clock().now().nanoseconds - t0) < secs * 3e9:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for(self, cond, what, timeout_wall=900):
        import time
        t0 = time.time()
        while rclpy.ok() and not cond():
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - t0 > timeout_wall:
                raise RuntimeError('tiempo agotado esperando: ' + what)

    def sim_t(self):
        """Segundos de simulacion desde el arranque del mundo (el /clock de MVSim empieza en la hora real)."""
        return self.clock - self.args.clock_origin

    def map_ready(self):
        """map -> odom publicado y RECIENTE (no un TF antiguo): EasyNav ya funciona."""
        try:
            t = self.buf.lookup_transform('map', 'odom', Time())
            stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            return self.clock - stamp < 3.0
        except Exception:
            return False

    def run(self):
        log = self.get_logger().info
        self.wait_for(lambda: self.pos is not None and self.clock > 0, 'simulador')
        self.wait_for(self.map_ready, 'TF map -> odom (EasyNav tarda un par de minutos en inflar el mapa)')
        log('EasyNav listo (map->odom) en t_sim=%.1f' % self.sim_t())
        self.wait_for(lambda: self.sim_t() >= self.args.start_at, 'tiempo de inicio')
        for _ in range(3):
            # Pose REAL del robot (verdad de terreno), no la de arranque: asi AMCL queda bien sea cual sea el
            # punto donde esté el Caddy (el mapa y el mundo comparten origen).
            m = PoseWithCovarianceStamped()
            m.header.frame_id = 'map'
            m.pose.pose.position.x, m.pose.pose.position.y = self.pos
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = yaw_q(self.yaw)
            m.pose.covariance[0] = m.pose.covariance[7] = 0.25
            m.pose.covariance[35] = 0.07
            self.pub_init.publish(m)
            self.spin_for(0.5)
        self.spin_for(3.0)
        for i, (gx, gy, gz, what) in enumerate(GOALS):
            g = PoseStamped()
            g.header.frame_id = 'map'
            g.pose.position.x, g.pose.position.y, g.pose.position.z = gx, gy, gz
            g.pose.orientation.z, g.pose.orientation.w = yaw_q(math.atan2(gy - self.pos[1], gx - self.pos[0]))
            t_goal = self.clock
            self.path_len = -1
            self.pub_goal.publish(g)
            log('[t_sim=%.1f] OBJETIVO %d/%d (%.0f, %.0f, z=%.2f): %s' % (self.sim_t(), i + 1, len(GOALS), gx, gy, gz, what))
            no_path_since = None
            result = 'tiempo agotado'
            while rclpy.ok() and self.clock - t_goal < self.args.timeout:
                rclpy.spin_once(self, timeout_sec=0.2)
                if math.hypot(self.pos[0] - gx, self.pos[1] - gy) < self.args.arrive:
                    result = 'LLEGADO'
                    break
                if self.path_len == 0 and self.clock - t_goal > 3:
                    no_path_since = no_path_since or self.clock
                    if self.clock - no_path_since > self.args.no_path_s:
                        result = 'SIN CAMINO (A* no encuentra ruta)'
                        break
                elif self.path_len > 0:
                    no_path_since = None
            log('[t_sim=%.1f] objetivo %d: %s | robot en (%.1f, %.1f) tras %.0f s'
                % (self.sim_t(), i + 1, result, self.pos[0], self.pos[1], self.clock - t_goal))
            self.spin_for(2.0)
        log('Secuencia terminada en t_sim=%.1f' % self.sim_t())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--clock-origin', type=float, default=0.0, help='valor de /clock al arrancar el mundo (s)')
    ap.add_argument('--start-at', type=float, default=0.0, help='t_sim minimo (desde el arranque del mundo) para el primer objetivo')
    ap.add_argument('--arrive', type=float, default=1.2, help='distancia (m) para dar el objetivo por alcanzado')
    ap.add_argument('--timeout', type=float, default=70.0, help='segundos de simulacion maximos por objetivo')
    ap.add_argument('--no-path-s', type=float, default=10.0, help='segundos sin camino antes de abandonar el objetivo')
    args = ap.parse_args()
    rclpy.init()
    node = Goals(args)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
