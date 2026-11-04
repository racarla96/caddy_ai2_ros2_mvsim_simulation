#!/usr/bin/env python3
"""
Valida un vehiculo MVSim con ros2_control: odometria del controlador frente a la verdad del simulador, en recta y en
curva, y reparto de velocidades y angulos entre ruedas.

Se usa con cualquier cadena de controladores: solo hay que dar los topics.

  # demo de referencia de MVSim (ackermann_steering_controller)
  validate_ros2_control.py --reference /ackermann_steering_controller/reference \\
      --odom /ackermann_steering_controller/odometry --gt /base_pose_ground_truth --joint-states /joint_states

  # Caddy (bicycle_steering_controller -> adaptadores)
  validate_ros2_control.py --ns /caddy_ai2 --reference bicycle_steering_controller/reference \\
      --odom bicycle_steering_controller/odometry --gt ground_truth/odometry --joint-states joint_states \\
      --left-traction rear_left_wheel_joint --right-traction rear_right_wheel_joint \\
      --left-steering front_left_wheel_steer_joint --right-steering front_right_wheel_steer_joint

Prueba A (recta): v constante durante --t-straight s; compara la distancia de la odometria con la de la verdad.
Prueba B (curva a izquierdas): v y w constantes durante --t-curve s; compara distancia, desplazamiento y giro, y mira en
regimen estacionario cual rueda gira mas (traccion) y cual dirige mas (direccion). En una curva a izquierdas la rueda
interior es la izquierda: dirige MAS que la exterior y, al ir por un radio menor, gira MENOS.
"""
import argparse
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Validator(Node):
    def __init__(self, a):
        super().__init__('validate_ros2_control')
        self.a = a
        ns = a.ns.rstrip('/')
        full = lambda t: t if t.startswith('/') else ns + '/' + t
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.odom = self.gt = None
        self.js = {}
        self.create_subscription(Odometry, full(a.odom), lambda m: setattr(self, 'odom', m), be)
        self.create_subscription(Odometry, full(a.gt), lambda m: setattr(self, 'gt', m), be)
        self.create_subscription(JointState, full(a.joint_states), self.on_js, be)
        self.pub = self.create_publisher(TwistStamped, full(a.reference), 10)

    def on_js(self, m):
        for n, p, v in zip(m.name, m.position, m.velocity):
            self.js[n] = (p, v)

    def spin(self, secs, v=None, w=None, sample=None):
        """Publica (v, w) durante secs; devuelve las muestras de sample() en la segunda mitad."""
        out, t0 = [], time.time()
        while time.time() - t0 < secs:
            if v is not None:
                m = TwistStamped()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = 'base_link'
                m.twist.linear.x, m.twist.angular.z = float(v), float(w)
                self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.02)
            if sample and time.time() - t0 > secs / 2:
                out.append(sample())
        return out

    def pose(self, m, off=0.0):
        """Pose (x, y, yaw) de m, trasladada `off` m a lo largo del eje del vehiculo (por defecto, la propia)."""
        yaw = yaw_of(m.pose.pose.orientation)
        return (m.pose.pose.position.x + off * math.cos(yaw), m.pose.pose.position.y + off * math.sin(yaw), yaw)

    def run(self):
        a = self.a
        t0 = time.time()
        while (self.odom is None or self.gt is None or not self.js) and time.time() - t0 < 20:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom is None or self.gt is None or not self.js:
            raise SystemExit('sin datos: odom=%s gt=%s joint_states=%s' % (self.odom is not None, self.gt is not None, bool(self.js)))
        self.spin(1.0)                       # reposo
        res = {}
        for name, v, w, t in (('A recta', a.v, 0.0, a.t_straight), ('B curva izq', a.v, a.w, a.t_curve)):
            o0, g0 = self.pose(self.odom), self.pose(self.gt, a.gt_offset_x)
            lt, rt, ls, rs = a.left_traction, a.right_traction, a.left_steering, a.right_steering
            samp = self.spin(t, v, w, sample=lambda: (self.js.get(lt, (0, 0)), self.js.get(rt, (0, 0)),
                                                       self.js.get(ls, (0, 0)), self.js.get(rs, (0, 0))))
            self.spin(2.5, 0.0, 0.0)         # parar y dejar que se asiente
            o1, g1 = self.pose(self.odom), self.pose(self.gt, a.gt_offset_x)
            d_o = math.hypot(o1[0] - o0[0], o1[1] - o0[1]); d_g = math.hypot(g1[0] - g0[0], g1[1] - g0[1])
            yaw_o, yaw_g = wrap(o1[2] - o0[2]), wrap(g1[2] - g0[2])
            # desplazamiento en el sistema local del arranque, para poder comparar aunque odom y mundo difieran
            def local(p0, p1):
                dx, dy = p1[0] - p0[0], p1[1] - p0[1]
                c, s = math.cos(-p0[2]), math.sin(-p0[2])
                return dx * c - dy * s, dx * s + dy * c
            lo, lg = local(o0, o1), local(g0, g1)
            err = math.hypot(lo[0] - lg[0], lo[1] - lg[1])
            print('\n== Prueba %s (v=%.2f m/s, w=%.2f rad/s, %.0f s)' % (name, v, w, t))
            print('   distancia recorrida:  odometria %.4f m | verdad %.4f m | diferencia %+.4f m (%+.2f %%)'
                  % (d_o, d_g, d_o - d_g, 100 * (d_o - d_g) / max(d_g, 1e-9)))
            print('   desplazamiento local: odometria (%.3f, %.3f) | verdad (%.3f, %.3f) | error de posicion %.3f m'
                  % (lo[0], lo[1], lg[0], lg[1], err))
            print('   giro (yaw):           odometria %.2f deg | verdad %.2f deg | diferencia %+.2f deg'
                  % (math.degrees(yaw_o), math.degrees(yaw_g), math.degrees(yaw_o - yaw_g)))
            if samp:
                m_ = np.mean(np.array([[x[0][1], x[1][1], x[2][0], x[3][0]] for x in samp]), axis=0)
                print('   regimen estacionario: velocidad rueda trasera izq %.3f | der %.3f rad/s ; direccion izq %.2f | der %.2f deg'
                      % (m_[0], m_[1], math.degrees(m_[2]), math.degrees(m_[3])))
                res[name] = dict(vl=m_[0], vr=m_[1], sl=m_[2], sr=m_[3], err=err, dd=d_o - d_g)
        if 'B curva izq' in res:
            r = res['B curva izq']
            print('\n== Reparto entre ruedas en la curva a izquierdas (rueda interior = izquierda)')
            print('   direccion: |izq| %.2f deg %s |der| %.2f deg  -> %s' % (math.degrees(abs(r['sl'])), '>' if abs(r['sl']) > abs(r['sr']) else '<=',
                  math.degrees(abs(r['sr'])), 'OK: la interior dirige mas' if abs(r['sl']) > abs(r['sr']) else 'FALLA'))
            print('   traccion:  |izq| %.3f rad/s %s |der| %.3f  -> %s' % (abs(r['vl']), '<' if abs(r['vl']) < abs(r['vr']) else '>=',
                  abs(r['vr']), 'OK: la interior gira menos' if abs(r['vl']) < abs(r['vr']) else 'FALLA'))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ns', default='')
    ap.add_argument('--reference', default='/ackermann_steering_controller/reference')
    ap.add_argument('--odom', default='/ackermann_steering_controller/odometry')
    ap.add_argument('--gt', default='/base_pose_ground_truth')
    ap.add_argument('--joint-states', default='/joint_states')
    ap.add_argument('--left-traction', default='rl_wheel_joint')
    ap.add_argument('--right-traction', default='rr_wheel_joint')
    ap.add_argument('--left-steering', default='fl_steering_joint')
    ap.add_argument('--right-steering', default='fr_steering_joint')
    ap.add_argument('--gt-offset-x', type=float, default=0.0,
                    help='metros a lo largo del vehiculo desde el origen de la verdad hasta el punto que sigue la odometria del '
                         'controlador. bicycle_steering_controller sigue el EJE TRASERO: en el Caddy (origen a mitad de la distancia '
                         'entre ejes) son -wheelbase/2 = -0.825. Sin esto, en una curva el error sale ~wheelbase/2 * |e^(i*giro) - 1|.')
    ap.add_argument('--v', type=float, default=1.0)
    ap.add_argument('--w', type=float, default=0.3)
    ap.add_argument('--t-straight', type=float, default=6.0)
    ap.add_argument('--t-curve', type=float, default=8.0)
    a = ap.parse_args()
    rclpy.init()
    n = Validator(a)
    try:
        n.run()
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
