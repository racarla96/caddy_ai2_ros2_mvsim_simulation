#!/usr/bin/env python3
"""
Adaptador minimo MVSim -> contrato de la simulacion Gazebo.

Casi todo el contrato se consigue SIN tocar los mensajes (el launch lo hace con remapeos de ROS
sobre mvsim_node, MVSim >= 2.0 con publish_sensor_tf:=False y profundidad 32FC1):
  - nombres de topics: remapeos.
  - arbol TF base_footprint -> base_link -> enlaces de sensores: robot_state_publisher con la URDF
    de Gazebo (sin plugins).
Este nodo hace solo lo que los remapeos no pueden, y solo con mensajes pequenos:

  1. TF estaticos de IDENTIDAD entre cada enlace de la URDF y el frame que MVSim pone en sus
     mensajes (MVSim usa como frame_id la etiqueta interna del sensor, p. ej. "rs_helios_5515_points",
     y no se puede cambiar). Asi los frames de los mensajes se resuelven en el arbol TF sin copiar
     ni reescribir ningun mensaje. Los frame_id de los mensajes NO son identicos a los de Gazebo;
     lo que se unifica es el arbol TF.
  2. odom (MVSim) -> bicycle_steering_controller/odometry y TF odom -> base_footprint (en MVSim,
     base_link esta en el suelo: es el base_footprint de Gazebo).
  3. bicycle_steering_controller/reference (TwistStamped) -> cmd_vel (Twist, MVSim).

Parametros:
  frame_aliases: lista de "padre->hijo" (enlace URDF -> frame de MVSim), la genera el launch.
  namespace, prefix.
"""
import rclpy
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

SENSOR_QOS_IN = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
OUT_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)
STATIC_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        history=HistoryPolicy.KEEP_LAST, depth=100)


class GazeboCompatAdapter(Node):
    def __init__(self):
        super().__init__('gazebo_compat_adapter')
        self.declare_parameter('namespace', 'caddy_ai2')
        self.declare_parameter('prefix', '')
        self.declare_parameter('frame_aliases', [''])
        ns = self.get_parameter('namespace').value
        self.prefix = self.get_parameter('prefix').value
        self.ns = '/' + ns + '/' if ns else '/'

        self._publish_aliases(self.get_parameter('frame_aliases').value)

        self.pub_odom = self.create_publisher(Odometry, self.ns + 'bicycle_steering_controller/odometry', OUT_QOS)
        self.pub_tf = self.create_publisher(TFMessage, '/tf', 100)
        self.create_subscription(Odometry, self.ns + 'odom', self._on_odom, SENSOR_QOS_IN)

        self.pub_cmd = self.create_publisher(Twist, self.ns + 'cmd_vel', 10)
        self.create_subscription(TwistStamped, self.ns + 'bicycle_steering_controller/reference',
                                 lambda m: self.pub_cmd.publish(m.twist), 10)
        self.get_logger().info('Adaptador listo: odometria, mando y %d alias de frame' % self.n_alias)

    def _publish_aliases(self, aliases):
        msgs = []
        stamp = self.get_clock().now().to_msg()
        for a in aliases:
            if '->' not in a:
                continue
            parent, child = (x.strip() for x in a.split('->', 1))
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            msgs.append(t)
        self.n_alias = len(msgs)
        self.pub_static = self.create_publisher(TFMessage, '/tf_static', STATIC_QOS)
        self.pub_static.publish(TFMessage(transforms=msgs))

    def _on_odom(self, msg):
        msg.header.frame_id = self.prefix + 'odom'
        msg.child_frame_id = self.prefix + 'base_footprint'
        self.pub_odom.publish(msg)
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.rotation = msg.pose.pose.orientation
        self.pub_tf.publish(TFMessage(transforms=[t]))


def main():
    rclpy.init()
    node = GazeboCompatAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
