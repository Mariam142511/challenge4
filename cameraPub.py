import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPub(Node):
    def __init__(self):
        super().__init__('cameraPub')

        self.publisher_ = self.create_publisher(Image, 'video_source/raw', 10)
        self.br = CvBridge()

        # Inicializa la cámara de la Jetson (por lo general index 0 funciona)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir la cámara.')
            return

        self.get_logger().info('Cámara iniciada correctamente.')
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 fps

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warning('No se pudo leer un frame de la cámara.')

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
