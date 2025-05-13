#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

class PathPublisher(Node):
    def __init__(self):
        super().__init__('Generator_node')

        # Declarar y leer parámetro
        self.declare_parameter('figura', 'triangulo')
        self.figura = self.get_parameter('figura').get_parameter_value().string_value
        self.get_logger().info(f'Figura seleccionada desde YAML: {self.figura}')

        # Crear publicador
        self.publisher = self.create_publisher(Pose2D, 'point', 10)

        # Definir trayectorias
        self.trayectorias = {
            'triangulo': [(0.5, 0.0), (0.25, 0.5), (0.0, 0.0)],
            'cuadrado': [(0.5, 0.5), (0.0, 1.0), (-0.5, 0.5), (0.0, 0.0)],
            'pentagono': [(-0.1, 0.25), (0.125, 0.45), (0.35, 0.25), (0.25, 0.0), (0.0, 0.0)],
            'hexagono': [(-0.15, 0.25), (0.0, 0.5), (0.25, 0.5), (0.4, 0.25), (0.2, 0.0), (0.0, 0.0)]
        }

        # Validar figura
        if self.figura not in self.trayectorias:
            self.get_logger().warn(f"Figura '{self.figura}' no válida. Usando triangulo por defecto.")
            self.trayectoria = self.trayectorias['triangulo']
        else:
            self.trayectoria = self.trayectorias[self.figura]

        self.index = 0
        self.timer = self.create_timer(4.0, self.publish_next_point)
        self.get_logger().info('Generador de trayectoria iniciado.')

    def publish_next_point(self):
        if self.index < len(self.trayectoria):
            x, y = self.trayectoria[self.index]
            point = Pose2D()
            point.x = x
            point.y = y
            point.theta = 0.0
            self.publisher.publish(point)
            self.get_logger().info(f'Objetivo publicado: ({x:.2f}, {y:.2f})')
            self.index += 1
        else:
            self.get_logger().info('Todas las trayectorias completadas.')
            self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
