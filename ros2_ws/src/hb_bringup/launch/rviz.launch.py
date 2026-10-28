"""RViz for watching the real robot from a laptop (same ROS_DOMAIN_ID as the Pi).

    ros2 launch hb_bringup rviz.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('hb_bringup'), 'rviz', 'hb_bot.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', config],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
