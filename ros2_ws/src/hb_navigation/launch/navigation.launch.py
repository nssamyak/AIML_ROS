"""Nav2: plan and drive to goals on the SLAM map.

    ros2 launch hb_navigation navigation.launch.py use_sim_time:=true
Then click "2D Goal Pose" in RViz.

Only the servers this robot needs are started:
  planner_server     global path on the map (NavFn)
  controller_server  follows the path (MPPI, "Omni" motion model)
  behavior_server    recoveries: spin, back up, wait
  velocity_smoother  limits acceleration, publishes /cmd_vel
  bt_navigator       the behaviour tree tying it all together
cmd_vel flow:  controller / behaviors -> cmd_vel_nav -> velocity_smoother -> cmd_vel
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('hb_navigation')
    params = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    common = [params, {'use_sim_time': use_sim_time}]

    def nav2_node(package, executable, remappings=()):
        return Node(package=package, executable=executable, name=executable,
                    parameters=common, remappings=list(remappings), output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'nav2_params.yaml')),

        nav2_node('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_planner', 'planner_server'),
        nav2_node('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother',
                  [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        nav2_node('nav2_bt_navigator', 'bt_navigator'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', parameters=common, output='screen'),
    ])
