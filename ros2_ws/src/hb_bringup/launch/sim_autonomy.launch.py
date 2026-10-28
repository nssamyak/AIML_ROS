"""Everything in simulation: Gazebo + SLAM + Nav2 + frontier exploration + RViz.

    ros2 launch hb_bringup sim_autonomy.launch.py
    ros2 launch hb_bringup sim_autonomy.launch.py explore:=false   # click goals yourself
    ros2 launch hb_bringup sim_autonomy.launch.py headless:=true rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, launch_file, **args):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=args.items())


def generate_launch_description():
    sim_time = {'use_sim_time': 'true'}
    rviz_config = os.path.join(get_package_share_directory('hb_bringup'), 'rviz', 'hb_bot.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('explore', default_value='true'),
        DeclareLaunchArgument('use_meshes', default_value='false'),

        include('hb_gazebo', 'sim.launch.py',
                headless=LaunchConfiguration('headless'),
                use_meshes=LaunchConfiguration('use_meshes')),

        # Give Gazebo a few seconds to spawn the robot and start the controllers
        TimerAction(period=5.0, actions=[include('hb_navigation', 'slam.launch.py', **sim_time)]),
        TimerAction(period=8.0, actions=[include('hb_navigation', 'navigation.launch.py', **sim_time)]),
        TimerAction(period=12.0, actions=[include('hb_navigation', 'explore.launch.py', **sim_time)],
                    condition=IfCondition(LaunchConfiguration('explore'))),

        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
