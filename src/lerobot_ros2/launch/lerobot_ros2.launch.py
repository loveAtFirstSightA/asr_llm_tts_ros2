import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lerobot_ros2',
            executable='lerobot_ros2',
            parameters=[os.path.join(get_package_share_directory('lerobot_ros2'),
                'config', 'param.yaml')],
            output='screen'),
    ])
