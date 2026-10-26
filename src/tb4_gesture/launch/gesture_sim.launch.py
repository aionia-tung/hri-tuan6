"""Bài 06 - Phần 1: Ignition Gazebo (arena 8x8 m) + TurtleBot 4.

Tương đương turtlebot4_ignition.launch.py nhưng:
  - dùng world gesture_arena.sdf (sàn trống, tường thấp) để robot chuyển động không va chạm;
  - dock được đặt ở góc arena (dock_x, dock_y) thay vì ngay trước mũi robot, robot
    xuất phát ở (0, 0) đã rời dock nên tiến/lùi/quay tự do được;
  - không chạy Nav2 / bản đồ (không bắt buộc ở bài này);
  - sửa lỗi GPU-lidar Ogre1 trên GPU Intel/Mesa (overlay irobot_create_description ogre2),
    giống bài 05.
"""

import os
import shutil
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from irobot_create_common_bringup.namespace import GetNamespacedName
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CACHE = Path.home() / '.cache' / 'tb4_gesture'


def make_ogre2_overlay():
    """Bản sao irobot_create_description với <render_engine>ogre2</render_engine>."""
    source = Path(get_package_share_directory('irobot_create_description'))
    prefix = CACHE / 'ogre2_overlay'
    target = prefix / 'share' / 'irobot_create_description'
    if source == target:
        return
    if not target.exists():
        shutil.copytree(source, target)
    text = (source / 'urdf' / 'create3.urdf.xacro').read_text()
    (target / 'urdf' / 'create3.urdf.xacro').write_text(
        text.replace('<render_engine>ogre</render_engine>', '<render_engine>ogre2</render_engine>'))
    index = prefix / 'share' / 'ament_index' / 'resource_index' / 'packages'
    index.mkdir(parents=True, exist_ok=True)
    (index / 'irobot_create_description').touch()
    os.environ['AMENT_PREFIX_PATH'] = f'{prefix}:{os.environ.get("AMENT_PREFIX_PATH", "")}'


def launch_setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    if value('lidar_ogre2') == 'true':
        make_ogre2_overlay()

    pkg_gesture = get_package_share_directory('tb4_gesture')
    pkg_tb4_ign = get_package_share_directory('turtlebot4_ignition_bringup')
    pkg_tb4_gui_plugins = get_package_share_directory('turtlebot4_ignition_gui_plugins')
    pkg_tb4_description = get_package_share_directory('turtlebot4_description')
    pkg_create_common = get_package_share_directory('irobot_create_common_bringup')
    pkg_create_description = get_package_share_directory('irobot_create_description')
    pkg_create_ign = get_package_share_directory('irobot_create_ignition_bringup')
    pkg_create_ign_plugins = get_package_share_directory('irobot_create_ignition_plugins')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    launch_file = lambda pkg, name: PythonLaunchDescriptionSource(  # noqa: E731
        os.path.join(pkg, 'launch', name))

    env = [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ':'.join([
            os.path.join(pkg_gesture, 'worlds'),
            os.path.join(pkg_tb4_ign, 'worlds'),
            os.path.join(pkg_create_ign, 'worlds'),
            str(Path(pkg_tb4_description).parent.resolve()),
            str(Path(pkg_create_description).parent.resolve()),
        ])),
        SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH', ':'.join([
            os.path.join(pkg_tb4_gui_plugins, 'lib'),
            os.path.join(pkg_create_ign_plugins, 'lib'),
        ])),
    ]

    gazebo = IncludeLaunchDescription(
        launch_file(pkg_ros_ign_gazebo, 'ign_gazebo.launch.py'),
        launch_arguments=[('ign_args', ' '.join(
            [value('world') + '.sdf', '-r', '-v 2', '--gui-config',
             os.path.join(pkg_tb4_ign, 'gui', value('model'), 'gui.config')]
            + ([] if value('gui') == 'true' else ['-s'])))])

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        output='screen', arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'])

    namespace = ''
    robot_name = GetNamespacedName(namespace, 'turtlebot4')
    dock_name = GetNamespacedName(namespace, 'standard_dock')

    robot = GroupAction([
        IncludeLaunchDescription(
            launch_file(pkg_tb4_description, 'robot_description.launch.py'),
            launch_arguments=[('model', value('model')), ('use_sim_time', 'true'),
                              ('namespace', namespace)]),
        IncludeLaunchDescription(
            launch_file(pkg_create_common, 'dock_description.launch.py'),
            launch_arguments=[('gazebo', 'ignition')]),
        Node(package='ros_ign_gazebo', executable='create', output='screen',
             arguments=['-name', robot_name, '-x', value('x'), '-y', value('y'), '-z', '0.0',
                        '-Y', value('yaw'), '-topic', 'robot_description']),
        # Dock đặt ở góc arena, quay mặt vào trong, cách xa vùng robot chuyển động
        Node(package='ros_ign_gazebo', executable='create', output='screen',
             arguments=['-name', dock_name, '-x', value('dock_x'), '-y', value('dock_y'),
                        '-z', '0.0', '-Y', '0.785', '-topic', 'standard_dock_description']),
        IncludeLaunchDescription(
            launch_file(pkg_tb4_ign, 'ros_ign_bridge.launch.py'),
            launch_arguments=[('model', value('model')), ('robot_name', robot_name),
                              ('dock_name', dock_name), ('namespace', namespace),
                              ('world', value('world'))]),
        IncludeLaunchDescription(
            launch_file(pkg_tb4_ign, 'turtlebot4_nodes.launch.py'),
            launch_arguments=[('model', value('model'))]),
        IncludeLaunchDescription(
            launch_file(pkg_create_common, 'create3_nodes.launch.py'),
            launch_arguments=[('namespace', namespace)]),
        IncludeLaunchDescription(
            launch_file(pkg_create_ign, 'create3_ignition_nodes.launch.py'),
            launch_arguments=[('robot_name', robot_name), ('dock_name', dock_name)]),
        Node(name='rplidar_stf', package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0.0', 'rplidar_link',
                        [robot_name, '/rplidar_link/rplidar']]),
    ])

    return env + [gazebo, clock_bridge, robot]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='gesture_arena',
                              description='World Ignition (gesture_arena, depot, maze, warehouse)'),
        DeclareLaunchArgument('model', default_value='standard', choices=['standard', 'lite']),
        # Các launch con (robot_description, spawner controller) đọc 'namespace' lúc chạy;
        # nếu không khai báo, robot_description lấy mặc định = robot_name -> sai namespace
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'],
                              description='Mở cửa sổ Gazebo'),
        DeclareLaunchArgument('lidar_ogre2', default_value='true', choices=['true', 'false'],
                              description='Render cảm biến bằng ogre2 (sửa lỗi GPU Intel/Mesa)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('dock_x', default_value='-3.0'),
        DeclareLaunchArgument('dock_y', default_value='-3.0'),
        OpaqueFunction(function=launch_setup),
    ])
