"""Real robot base, run on the Raspberry Pi.

    ros2 launch hb_bringup robot.launch.py
    ros2 launch hb_bringup robot.launch.py esp32_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB1
    ros2 launch hb_bringup robot.launch.py lidar:=none     # start your own lidar driver instead

What runs:
  robot_state_publisher  URDF -> TF (sim_mode:=false: no Gazebo plugins)
  rplidar_node           /scan from the lidar
  esp32_bridge           serial link to the ESP32 motor controller
  kiwi_drive             cmd_vel -> wheel speeds, wheel feedback -> odom
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = get_package_share_directory('hb_description')
    control = get_package_share_directory('hb_control')
    bringup = get_package_share_directory('hb_bringup')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(description, 'urdf', 'hb_bot.urdf.xacro'),
                 ' sim_mode:=false lidar_yaw:=', LaunchConfiguration('lidar_yaw')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('esp32_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar', default_value='rplidar', description='rplidar or none'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_baud', default_value='115200',
                              description='A1/A2M8: 115200, A2M12/A3: 256000, C1/S1: 460800'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0',
                              description='lidar mounting rotation in radians (e.g. 3.14159)'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),

        Node(package='rplidar_ros', executable='rplidar_node', name='rplidar',
             parameters=[os.path.join(bringup, 'config', 'rplidar.yaml'),
                         {'serial_port': LaunchConfiguration('lidar_port'),
                          'serial_baudrate': LaunchConfiguration('lidar_baud')}],
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration('lidar'), 'rplidar')),
             output='screen'),

        Node(package='hb_control', executable='esp32_bridge',
             parameters=[os.path.join(control, 'config', 'esp32_bridge.yaml'),
                         {'port': LaunchConfiguration('esp32_port')}],
             output='screen'),

        Node(package='hb_control', executable='kiwi_drive',
             parameters=[os.path.join(control, 'config', 'kiwi_drive.yaml')],
             output='screen'),
    ])
