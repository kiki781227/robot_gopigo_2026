"""
robot.launch.py
───────────────
Lance tous les nodes du robot avec la config YAML.

Usage :
  ros2 launch robot_gopigo robot.launch.py

Options (arguments en ligne de commande) :
  mission:=false       → lance tout sauf le mission_node (utile pour tests manuels)
  debug:=false         → lance tout sauf le debug_server (moins de charge CPU)
  config:=<chemin>     → utilise un autre fichier de config

Exemples :
  ros2 launch robot_gopigo robot.launch.py
  ros2 launch robot_gopigo robot.launch.py mission:=false
  ros2 launch robot_gopigo robot.launch.py debug:=false mission:=false
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Arguments de lancement ──────────────────────────────────────────────
    default_config = os.path.join(
        get_package_share_directory('robot_gopigo'),
        'config',
        'robot_config.yaml'
    )

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Chemin vers le fichier YAML de configuration'
    )

    mission_arg = DeclareLaunchArgument(
        'mission',
        default_value='true',
        description='Lancer le mission_node (true/false)'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Lancer le debug_server web (true/false)'
    )

    config_file = LaunchConfiguration('config')

    # ── Nodes ───────────────────────────────────────────────────────────────

    # Driver unifié : moteurs + odo + servo
    driver_node = Node(
        package='robot_gopigo',
        executable='gopigo_driver_node',
        name='gopigo_driver_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    # Caméra
    camera_node = Node(
        package='robot_gopigo',
        executable='camera_node',
        name='camera_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    # Détecteur ArUco
    aruco_node = Node(
        package='robot_gopigo',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    # Détecteur cubes colorés
    cube_node = Node(
        package='robot_gopigo',
        executable='cube_detector_node',
        name='cube_detector_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    # Debug server (conditionnel)
    debug_node = Node(
        package='robot_gopigo',
        executable='debug_server',
        name='debug_server',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('debug')),
    )

    # Mission (conditionnel)
    mission_node = Node(
        package='robot_gopigo',
        executable='mission_node',
        name='mission_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('mission')),
    )

    return LaunchDescription([
        config_arg,
        mission_arg,
        debug_arg,
        driver_node,
        camera_node,
        aruco_node,
        cube_node,
        debug_node,
        mission_node,
    ])
