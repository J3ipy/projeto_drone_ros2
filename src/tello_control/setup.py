# Caminho: tello_ros_ws/src/tello_control/setup.py

import os
from glob import glob
from setuptools import setup

package_name = 'tello_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Garante que os ficheiros de lançamento sejam instalados
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Garante que o ficheiro mission.json seja instalado (com a vírgula)
        (os.path.join('share', package_name), ['tello_control/mission.json'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS 2 control package for DJI Tello drone.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_node = tello_control.drone_node:main',
            'vision_node = tello_control.vision_node:main',
            'mission_controller_node = tello_control.mission_controller_node:main',
        ],
    },
)