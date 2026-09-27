"""Nav2 navigation servers wired for the rover's manual/auto velocity chain.

  controller/behaviors --cmd_vel_nav_raw--> velocity_smoother --cmd_vel_smoothed-->
  collision_monitor --cmd_vel_nav--> rover_control/mode_manager --cmd_vel--> robot
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

LIFECYCLE_NODES = [
    'controller_server', 'smoother_server', 'planner_server', 'behavior_server',
    'velocity_smoother', 'collision_monitor', 'bt_navigator', 'waypoint_follower',
]


def generate_launch_description():
    pkg = get_package_share_directory('rover_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')
    to_raw = [('cmd_vel', 'cmd_vel_nav_raw')]

    def nav_node(package, executable, remappings=()):
        return Node(package=package, executable=executable, name=executable, output='screen',
                    parameters=[params_file], remappings=list(remappings),
                    arguments=['--ros-args', '--log-level', log_level])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('log_level', default_value='info'),
        GroupAction([
            SetParameter('use_sim_time', use_sim_time),
            nav_node('nav2_controller', 'controller_server', to_raw),
            nav_node('nav2_smoother', 'smoother_server'),
            nav_node('nav2_planner', 'planner_server'),
            nav_node('nav2_behaviors', 'behavior_server', to_raw),
            nav_node('nav2_bt_navigator', 'bt_navigator'),
            nav_node('nav2_waypoint_follower', 'waypoint_follower'),
            nav_node('nav2_velocity_smoother', 'velocity_smoother', to_raw),
            nav_node('nav2_collision_monitor', 'collision_monitor'),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 parameters=[{'autostart': True, 'node_names': LIFECYCLE_NODES}]),
        ]),
    ])
