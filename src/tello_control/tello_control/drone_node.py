# drone_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, BatteryState
from std_srvs.srv import Empty
from cv_bridge import CvBridge
from djitellopy import Tello
import cv2
import threading

# Importe as interfaces de serviço que vamos criar
# (Será necessário criar ficheiros .srv para isto, mas por simplicidade vamos usar Empty por agora
# e adaptar a lógica no mission_controller)

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        self.get_logger().info('Iniciando o Nó do Drone Tello...')

        # Inicialização do Drone
        self.tello = Tello()
        self.tello.connect()
        self.tello.streamon()
        self.frame_reader = self.tello.get_frame_read()

        # Publishers
        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.battery_publisher = self.create_publisher(BatteryState, '/battery_state', 10)
        
        self.bridge = CvBridge()

        # Timers para publicação contínua
        self.video_timer = self.create_timer(1.0 / 30.0, self.publish_video_frame) # 30 FPS
        self.battery_timer = self.create_timer(5.0, self.publish_battery_state) # A cada 5 segundos

        # Serviços para Comandos
        self.create_service(Empty, 'takeoff', self.takeoff_callback)
        self.create_service(Empty, 'land', self.land_callback)
        # Para comandos com argumentos como 'go' ou 'move_up', o ideal seria criar
        # serviços customizados. Por agora, o mission_controller usará RC control.

        # Subscriber para comandos RC (Roll, Pitch, Throttle, Yaw)
        # O mission_controller irá publicar aqui para movimentos finos.
        self.rc_subscription = self.create_subscription(
            rclpy.qos.QoSProfile(depth=1),
            'tello/rc_control', # Usaremos uma mensagem customizada ou um Twist
            self.rc_control_callback,
            10)
            
        self.get_logger().info('Nó do Drone Tello pronto.')

    def rc_control_callback(self, msg):
        # Aqui receberíamos uma mensagem (ex: geometry_msgs/Twist) e traduziríamos
        # self.tello.send_rc_control(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        # Por simplicidade, o mission_controller irá chamar a função diretamente por agora.
        pass

    def publish_video_frame(self):
        frame = self.frame_reader.frame
        if frame is not None:
            # Converte a imagem OpenCV para uma mensagem ROS e publica
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_frame"
            self.image_publisher.publish(ros_image)

    def publish_battery_state(self):
        battery_level = self.tello.get_battery()
        battery_msg = BatteryState()
        battery_msg.percentage = float(battery_level)
        self.battery_publisher.publish(battery_msg)
        self.get_logger().info(f'Bateria: {battery_level}%')

    def takeoff_callback(self, request, response):
        self.get_logger().info('Serviço de decolagem chamado...')
        self.tello.takeoff()
        return response

    def land_callback(self, request, response):
        self.get_logger().info('Serviço de aterragem chamado...')
        self.tello.land()
        return response

    def on_shutdown(self):
        self.get_logger().info('Desligando o nó do drone...')
        self.tello.land()
        self.tello.streamoff()
        self.tello.end()

def main(args=None):
    rclpy.init(args=args)
    drone_node = DroneNode()
    try:
        rclpy.spin(drone_node)
    except KeyboardInterrupt:
        pass
    finally:
        drone_node.on_shutdown()
        drone_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()