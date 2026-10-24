import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('rabbits_sim'), 'config', 'gamma.yaml')

    config_arg = DeclareLaunchArgument(
        'config', default_value=default_config,
        description='gamma の偽ファームの定数 YAML（gamma_turret_fw の robot.hpp と対で更新）')
    link_path_arg = DeclareLaunchArgument(
        'link_path', default_value='/tmp/gamma_fake',
        description='robot_bridge の device に渡すシンボリックリンクのパス')

    fake_node = Node(
        package='rabbits_sim',
        executable='gamma_fake_node',
        name='gamma_fake_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {'link_path': LaunchConfiguration('link_path')},
        ],
    )

    return LaunchDescription([config_arg, link_path_arg, fake_node])
