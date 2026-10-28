"""Real robot exploring on its own: base + SLAM + Nav2 + frontier exploration.
Run on the Raspberry Pi. Watch it from a laptop with:  ros2 launch hb_bringup rviz.launch.py

    ros2 launch hb_bringup robot_autonomy.launch.py
    ros2 launch hb_bringup robot_autonomy.launch.py explore:=false   # send goals from RViz
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package, launch_file, **args):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=args.items())


def generate_launch_description():
    passthrough = ['esp32_port', 'lidar', 'lidar_port', 'lidar_baud', 'lidar_yaw']
    defaults = {'esp32_port': '/dev/ttyUSB0', 'lidar': 'rplidar', 'lidar_port': '/dev/ttyUSB1',
                'lidar_baud': '115200', 'lidar_yaw': '0.0'}

    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=defaults[name]) for name in passthrough] + [
            DeclareLaunchArgument('explore', default_value='true'),

            include('hb_bringup', 'robot.launch.py',
                    **{name: LaunchConfiguration(name) for name in passthrough}),
            TimerAction(period=3.0, actions=[include('hb_navigation', 'slam.launch.py')]),
            TimerAction(period=6.0, actions=[include('hb_navigation', 'navigation.launch.py')]),
            TimerAction(period=10.0, actions=[include('hb_navigation', 'explore.launch.py')],
                        condition=IfCondition(LaunchConfiguration('explore'))),
        ])
