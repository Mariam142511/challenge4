#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import String
from math import atan2, sqrt, pi, fmod

# Función para mantener ángulos en el rango [-π, π]
def wrap_to_pi(angle):
    result = fmod(angle + pi, 2 * pi)
    if result < 0:
        result += 2 * pi
    return result - pi

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # Subscripciones
        self.create_subscription(Pose2D, 'odom', self.pose_callback, 10)
        self.create_subscription(Pose2D, 'point', self.point_callback, 10)
        self.create_subscription(String, 'semaforo', self.semaforo_callback, 10)

        # Publicadores
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.signal_pub = self.create_publisher(Twist, 'signal', 10)

        # Estado del robot y metas
        self.pose = Pose2D()
        self.goal = Pose2D()
        self.new_goal = False

        # Ganancias base
        self.linear_k = 0.5
        self.angular_k = 1.5

        # Color inicial (sin detección)
        self.color = 'green'

        # Timer principal
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Controller with point-to-point and semáforo started.')

    def pose_callback(self, msg):
        self.pose = msg

    def point_callback(self, msg):
        self.goal = msg
        self.new_goal = True

    def semaforo_callback(self, msg):
        self.color = msg.data

    def control_loop(self):
        if not self.new_goal:
            return

        # Cálculo de errores
        ex = self.goal.x - self.pose.x
        ey = self.goal.y - self.pose.y
        ed = sqrt(ex**2 + ey**2)
        etheta = wrap_to_pi(atan2(ey, ex) - self.pose.theta)

        # Inicializar velocidades
        v = 0.0
        w = 0.0

        # Modificar ganancias según color del semáforo
        if self.color.lower() == 'red':
            v = 0.0
            w = 0.0
        elif self.color.lower() == 'yellow':
            v = 0.2 * ed
            w = 0.4 * etheta
        elif self.color.lower() == 'green':
            v = self.linear_k * ed
            w = self.angular_k * etheta
        else:
            self.get_logger().warn(f"Color desconocido: {self.color}")

        # Imprimir información de control
        self.get_logger().info(f"[DEBUG] ed = {ed:.3f}, etheta = {etheta:.3f}, v = {v:.3f}, w = {w:.3f}")

        # Publicar velocidades
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        # Revisar si ya se alcanzó el punto
        if ed < 0.1:
            self.get_logger().info('Objetivo alcanzado.')
            self.signal_pub.publish(Twist())
            self.new_goal = False

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
