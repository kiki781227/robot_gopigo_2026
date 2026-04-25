from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_gopigo'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),

    data_files=[
        # Registre ROS2 du package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # Fichiers de configuration YAML
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='Robot GoPiGo3 - Trieur de cubes colorés avec ArUco',
    license='MIT',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            # ── Driver unifié (moteurs + odométrie + servo) ──
            'gopigo_driver_node = robot_gopigo.control.gopigo_driver_node:main',

            # ── Vision ──
            'camera_node         = robot_gopigo.vision.camera_node:main',
            'aruco_detector_node = robot_gopigo.vision.aruco_detector_node:main',
            'cube_detector_node  = robot_gopigo.vision.cube_detector_node:main',
            'debug_server        = robot_gopigo.vision.debug_server:main',

            # ── Mission ──
            'mission_node = robot_gopigo.logic.mission_node:main',
        ],
    },
)
