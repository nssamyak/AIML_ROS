"""Frontier exploration (needs SLAM and Nav2 running).

    ros2 launch hb_navigation explore.launch.py use_sim_time:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('hb_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'explorer.yaml')),
        Node(package='hb_navigation', executable='frontier_explorer',
             parameters=[LaunchConfiguration('params_file'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}],
             output='screen'),
    ])
