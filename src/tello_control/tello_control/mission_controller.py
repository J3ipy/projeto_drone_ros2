import json
import os
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Empty # Importa o tipo de serviço Empty

class MissionController:
    def __init__(self, node):
        # Armazena a referência ao nó que a está a usar
        self.node = node
        self.mission_plan = {}
        self.mission_sequence = []
        self._get_mission_plan()

        self.mission_state = "IDLE"
        self.current_step_index = 0

    def _get_mission_plan(self):
        """Carrega o plano de voo de um arquivo mission.json instalado."""
        try:
            # Encontra o caminho para o diretório de partilha do nosso pacote
            package_share_directory = get_package_share_directory('tello_control')
            # Constrói o caminho completo para o ficheiro mission.json
            mission_file_path = os.path.join(package_share_directory, 'mission.json')

            self.node.get_logger().info(f"A carregar plano de missão de: {mission_file_path}")
            with open(mission_file_path, 'r') as f:
                mission_data = json.load(f)
                self.mission_plan = mission_data['plan']
                self.mission_sequence = mission_data['sequence']
                self.node.get_logger().info("Plano de missão carregado com sucesso.")
        except Exception as e:
            self.node.get_logger().error(f"ERRO ao carregar mission.json: {e}")
            # Plano de emergência
            self.mission_plan = {'A': {'action': 'land', 'value': '0'}}
            self.mission_sequence = ['A']

    @property
    def target_qr(self) -> str:
        if self.current_step_index < len(self.mission_sequence):
            return self.mission_sequence[self.current_step_index]
        return "DONE"

    def update(self, qr_msg):
        """Este método é o callback para o tópico /qr_data."""
        qr_data = qr_msg.data
        self.node.get_logger().info(f'Estado: {self.mission_state}, Alvo: {self.target_qr}, QR Visto: {qr_data}')

        if self.mission_state == "SEARCHING":
            if qr_data and qr_data == self.target_qr:
                self.node.get_logger().info(f"Alvo '{self.target_qr}' encontrado! A executar ação.")
                
                # Para de se mover (assumindo que estava a procurar)
                # idealmente, o nó do drone faria isto ao receber um novo comando.

                # Pega a ação do plano de missão
                step = self.mission_plan[self.target_qr]
                action = step['action']
                value = step['value']

                if action == 'go':
                    # No ROS, em vez de 'go', iriamos publicar para o /cmd_vel ou usar um serviço.
                    # Por agora, apenas avançamos o índice.
                    self.node.get_logger().info(f"Ação 'go' para {value}. (Simulado)")
                    self.current_step_index += 1
                    if self.target_qr == "DONE":
                        self.mission_state = "LANDING"
                        self.node.get_logger().info("Missão concluída. A pedir para aterrar.")
                        self.node.land_client.call_async(Empty.Request())
                    else:
                        self.mission_state = "SEARCHING"
                        
                elif action == 'land':
                    self.mission_state = "LANDING"
                    self.node.get_logger().info("Ação 'land'. A pedir para aterrar.")
                    self.node.land_client.call_async(Empty.Request())
            else:
                # Lógica para procurar (ex: rodar)
                # Aqui você publicaria no /tello/rc_control
                pass