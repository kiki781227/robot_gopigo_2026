from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_gopigo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='Robot GoPiGo3 - Vision, Controle, Logique',
    license='MIT',
    tests_require=['pytest'],
    
    # Même si mon script "odometry.py et autre) est physiquement dans le dossier "control", c'est le nom du paquet (celui qui contient le fichier "package.xml") qui doit être utilisé dans la commande "ros2 run ...". Mon fichier setup.py qui fait le travail de "traducteur". C'est lui qui dit à ROS2 : "Quand l'utilisateur tape "ros
    entry_points={
        'console_scripts': [
			'gopigo_driver_node = robot_gopigo.control.gopigo_driver_node:main',
            'camera_node = robot_gopigo.vision.camera_node:main',
            'cube_detector_node = robot_gopigo.vision.cube_detector_node:main',
            'mission_node = robot_gopigo.logic.mission_node:main',
            'debug_server = robot_gopigo.vision.debug_server:main',
            'aruco_detector_node = robot_gopigo.vision.aruco_detector_node:main'
        ],
    },
)
