"""Nav2 navigation servers wired for the rover's manual/auto velocity chain.

  controller/behaviors --cmd_vel_nav_raw--> velocity_smoother --cmd_vel_smoothed-->
  collision_monitor --cmd_vel_nav--> rover_control/mode_manager --cmd_vel--> robot

robot:=rover   (2WD diff drive)  nav2_params.yaml, Nav2's default behavior trees
robot:=rover4  (4WD Ackermann)   nav2_params_rover4.yaml, behavior trees without Spin
params_file:=... overrides the parameter file for either robot.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

LIFECYCLE_NODES = [
    'controller_server', 'smoother_server', 'planner_server', 'behavior_server',
    'velocity_smoother', 'collision_monitor', 'bt_navigator', 'waypoint_follower',
]
PARAMS = {'rover': 'nav2_params.yaml', 'rover4': 'nav2_params_rover4.yaml'}


def launch_nodes(context):
    pkg = get_package_share_directory('rover_navigation')
    robot = LaunchConfiguration('robot').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context) or \
        os.path.join(pkg, 'config', PARAMS[robot])
    log_level = LaunchConfiguration('log_level')
    to_raw = [('cmd_vel', 'cmd_vel_nav_raw')]

    bt_overrides = {}
    if robot == 'rover4':   # car-like: recoveries without Spin
        bt_dir = os.path.join(pkg, 'behavior_trees')
        bt_overrides = {
            'default_nav_to_pose_bt_xml': os.path.join(bt_dir, 'ackermann_navigate_to_pose.xml'),
            'default_nav_through_poses_bt_xml': os.path.join(bt_dir, 'ackermann_navigate_through_poses.xml'),
        }

    def nav_node(package, executable, remappings=(), extra_params=None):
        return Node(package=package, executable=executable, name=executable, output='screen',
                    parameters=[params_file] + ([extra_params] if extra_params else []),
                    remappings=list(remappings),
                    arguments=['--ros-args', '--log-level', log_level])

    return [GroupAction([
        SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
        nav_node('nav2_controller', 'controller_server', to_raw),
        nav_node('nav2_smoother', 'smoother_server'),
        nav_node('nav2_planner', 'planner_server'),
        nav_node('nav2_behaviors', 'behavior_server', to_raw),
        nav_node('nav2_bt_navigator', 'bt_navigator', extra_params=bt_overrides),
        nav_node('nav2_waypoint_follower', 'waypoint_follower'),
        nav_node('nav2_velocity_smoother', 'velocity_smoother', to_raw),
        nav_node('nav2_collision_monitor', 'collision_monitor'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': True, 'node_names': LIFECYCLE_NODES}]),
    ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot', default_value='rover', choices=list(PARAMS)),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Nav2 parameter file (default: chosen by `robot`)'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=launch_nodes),
    ])
