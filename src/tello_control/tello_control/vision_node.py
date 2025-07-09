# vision_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

# Importa os seus módulos de projeto
from . import vision_module as vision
from . import display_module as display
from . import config

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()

        # Publishers
        self.qr_publisher = self.create_publisher(String, '/qr_data', 10)
        self.debug_image_publisher = self.create_publisher(Image, '/debug_image', 10)

        # Subscriber
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        self.get_logger().info('Nó de Visão iniciado.')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Falha ao converter imagem: {e}')
            return

        # Redimensiona o frame como no seu código original
        frame = cv2.resize(frame, (config.FRAME_WIDTH, config.FRAME_HEIGHT))
        
        # Usa a sua lógica de deteção original
        qr_data, qr_rect = vision.detect_and_draw_qr(frame)

        if qr_data:
            # Publica os dados do QR code detetado
            qr_msg = String()
            qr_msg.data = qr_data
            self.qr_publisher.publish(qr_msg)

        # Publica a imagem de depuração com as anotações
        debug_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.debug_image_publisher.publish(debug_image_msg)

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()