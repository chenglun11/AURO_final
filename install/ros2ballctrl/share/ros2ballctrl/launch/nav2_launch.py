# src/my_nav2_package/launch/nav2_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 使用自定义参数文件路径
    params_file = LaunchConfiguration('params_file', default='config/nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            output='screen',
            parameters=[params_file]
        )
    ])
