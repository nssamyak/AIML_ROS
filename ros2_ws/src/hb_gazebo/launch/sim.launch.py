"""Start Gazebo with the HB bot, ready to drive with /cmd_vel.

    ros2 launch hb_gazebo sim.launch.py
    ros2 launch hb_gazebo sim.launch.py headless:=true      # no Gazebo window
    ros2 launch hb_gazebo sim.launch.py world:=/path/to/world.sdf x:=0 y:=0

Then drive it:  ros2 run teleop_twist_keyboard teleop_twist_keyboard
(hold Shift for sideways moves: the robot is holonomic)

What runs:
  gz sim                 physics + lidar
  robot_state_publisher  URDF -> TF
  create                 spawns the robot from /robot_description
  parameter_bridge       /clock and /scan from Gazebo to ROS
  spawners               joint_state_broadcaster + wheel_velocity_controller
  kiwi_drive             cmd_vel -> wheel speeds, wheel feedback -> odom
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = get_package_share_directory('hb_description')
    gazebo_share = get_package_share_directory('hb_gazebo')
    control_share = get_package_share_directory('hb_control')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_meshes = LaunchConfiguration('use_meshes')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(description_share, 'urdf', 'hb_bot.urdf.xacro'),
                 ' sim_mode:=true',
                 ' use_meshes:=', use_meshes,
                 ' controllers_file:=', os.path.join(description_share, 'config', 'controllers.yaml')]),
        value_type=str)

    gz_launch = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': ['-r -v 2 ', world], 'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(headless))
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': ['-r -s --headless-rendering -v 2 ', world],
                          'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(headless))

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'hb_bot',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.01', '-Y', LaunchConfiguration('yaw')],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(gazebo_share, 'config', 'gz_bridge.yaml'),
                     'use_sim_time': True}],
        output='screen')

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '60'],
                    output='screen')

    # Controllers can only load once the robot (and its gz_ros2_control plugin) exists
    start_controllers = RegisterEventHandler(OnProcessExit(
        target_action=spawn,
        on_exit=[spawner('joint_state_broadcaster'), spawner('wheel_velocity_controller')]))

    kiwi_drive = Node(
        package='hb_control', executable='kiwi_drive',
        parameters=[os.path.join(control_share, 'config', 'kiwi_drive.yaml'),
                    {'use_sim_time': True}],
        output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(gazebo_share, 'rviz', 'sim.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(gazebo_share, 'worlds', 'arena.sdf')),
        DeclareLaunchArgument('headless', default_value='false', description='Run without the Gazebo window'),
        DeclareLaunchArgument('use_meshes', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('x', default_value='-1.3'),
        DeclareLaunchArgument('y', default_value='-0.2'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        # Lets Gazebo find package://hb_description/meshes/...
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(description_share)),

        gz_gui, gz_server,
        robot_state_publisher, spawn, bridge, start_controllers, kiwi_drive, rviz,
    ])
