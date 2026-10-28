"""View the robot model in RViz and move the wheels with sliders.

    ros2 launch hb_description display.launch.py
    ros2 launch hb_description display.launch.py use_meshes:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('hb_description')
    use_meshes = LaunchConfiguration('use_meshes')
    gui = LaunchConfiguration('gui')

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg, 'urdf', 'hb_bot.urdf.xacro']),
                 ' sim_mode:=false use_meshes:=', use_meshes]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_meshes', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Show joint sliders (needs joint_state_publisher_gui)'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'display.rviz'])]),
    ])
