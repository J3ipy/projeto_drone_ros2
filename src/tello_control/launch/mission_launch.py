# mission_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nó do Driver do Drone
        Node(
            package='tello_control',
            executable='drone_node',
            name='drone_node',
            output='screen'
        ),
        # Nó de Visão
        Node(
            package='tello_control',
            executable='vision_node',
            name='vision_node',
            output='screen'
        ),
        # Nó de Controlo da Missão
        Node(
            package='tello_control',
            executable='mission_controller_node',
            name='mission_controller_node',
            output='screen'
        ),
    ])