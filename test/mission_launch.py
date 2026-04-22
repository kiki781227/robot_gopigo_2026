"""
launch/mission_launch.py
Lance tous les nœuds en un seul terminal avec la config YAML.
Usage :
    ros2 launch robot_gopigo mission_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('robot_gopigo')
    config = os.path.join(pkg, 'config', 'mission_params.yaml')

    return LaunchDescription([

        # ── Vision ─────────────────────────────────────────
        Node(
            package='robot_gopigo',
            executable='camera_node',
            name='camera_node',
            output='screen',
        ),
        Node(
            package='robot_gopigo',
            executable='cube_detector_node',
            name='cube_detector_node',
            output='screen',
        ),
        Node(
            package='robot_gopigo',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen',
        ),

        # ── Contrôle ───────────────────────────────────────
        Node(
            package='robot_gopigo',
            executable='cmd_vel_node',
            name='cmd_vel_node',
            output='screen',
        ),
        Node(
            package='robot_gopigo',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
        ),
        Node(
            package='robot_gopigo',
            executable='ultrasonic_node',
            name='ultrasonic_node',
            output='screen',
        ),

        # ── Logique mission ────────────────────────────────
        Node(
            package='robot_gopigo',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[config],   # <── charge le YAML
        ),
    ])
