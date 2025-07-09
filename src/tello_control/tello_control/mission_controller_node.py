# mission_controller_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty

# Importa a sua classe MissionController (que será ligeiramente adaptada)
from .mission_controller import MissionController

class MissionControllerNode(Node):
    def __init__(self):
        super().__init__('mission_controller_node')
        
        # Instancia a sua classe de lógica de missão
        # Passamos o nó para que a classe possa usar os seus recursos (logger, etc.)
        self.mission_logic = MissionController(self)

        # Subscribers
        self.qr_subscription = self.create_subscription(
            String,
            '/qr_data',
            self.mission_logic.update,  # O método update da sua classe será o callback
            10)
            
        # Publisher de estado
        self.status_publisher = self.create_publisher(String, '/mission_status', 10)

        # Clientes de Serviço para chamar o drone_node
        self.takeoff_client = self.create_client(Empty, 'takeoff')
        self.land_client = self.create_client(Empty, 'land')
        
        # Espera que os serviços estejam disponíveis
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço de decolagem não disponível, a esperar...')
        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço de aterragem não disponível, a esperar...')

        # Inicia a missão (ex: chama o serviço de decolagem)
        self.start_mission()

    def start_mission(self):
        self.get_logger().info("A iniciar a missão... A enviar pedido de decolagem.")
        self.takeoff_client.call_async(Empty.Request())
        self.mission_logic.mission_state = "SEARCHING" # Atualiza o estado inicial

def main(args=None):
    rclpy.init(args=args)
    mission_controller_node = MissionControllerNode()
    rclpy.spin(mission_controller_node)
    mission_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()