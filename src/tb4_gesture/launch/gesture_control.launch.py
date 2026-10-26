"""Bài 06 - Phần 2 + 3: gesture_recognition_node (webcam) + robot_behavior_node.

Chạy sau gesture_sim.launch.py (hoặc dùng sim:=true để bật luôn mô phỏng).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('tb4_gesture')
    params = LaunchConfiguration('params')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=os.path.join(pkg, 'config', 'gesture_params.yaml')),
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('sim', default_value='false', choices=['true', 'false'],
                              description='Bật luôn Gazebo + TurtleBot 4 (gesture_sim.launch.py)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gesture_sim.launch.py')),
            condition=IfCondition(LaunchConfiguration('sim'))),
        Node(package='tb4_gesture', executable='gesture_recognition_node', output='screen',
             parameters=[params, {'camera_index': ParameterValue(
                 LaunchConfiguration('camera_index'), value_type=int)}]),
        Node(package='tb4_gesture', executable='robot_behavior_node', output='screen',
             parameters=[params]),
    ])
