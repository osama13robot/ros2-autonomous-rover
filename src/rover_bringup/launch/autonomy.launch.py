"""Robot-agnostic autonomy stack (used by both the simulation and the real robot).

EKF + (SLAM or AMCL localization) + Nav2 + mode manager + frontier explorer + RViz.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory('rover_navigation')
    bringup_pkg = get_package_share_directory('rover_bringup')
    control_params = os.path.join(bringup_pkg, 'config', 'control.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    explore = LaunchConfiguration('explore')

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'ekf.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'slam.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(slam))
    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'localization.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'map': LaunchConfiguration('map')}.items(),
        condition=UnlessCondition(slam))
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'navigation.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    mode_manager = Node(
        package='rover_control', executable='mode_manager', output='screen',
        parameters=[control_params, {'use_sim_time': use_sim_time,
                                     'initial_mode': LaunchConfiguration('initial_mode')}])

    # exploration only makes sense while building a map
    explorer = Node(
        package='rover_control', executable='frontier_explorer', output='screen',
        parameters=[control_params, {'use_sim_time': use_sim_time, 'enabled_on_start': explore}],
        condition=IfCondition(slam))

    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', os.path.join(bringup_pkg, 'rviz', 'rover.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('slam', default_value='true',
                              description='true: build a map with SLAM; false: localize on `map` with AMCL'),
        DeclareLaunchArgument('map', default_value='',
                              description='Map yaml used when slam:=false'),
        DeclareLaunchArgument('explore', default_value='true',
                              description='Start frontier exploration automatically when in AUTO mode'),
        DeclareLaunchArgument('initial_mode', default_value='manual', description='manual | auto'),
        DeclareLaunchArgument('rviz', default_value='true'),
        ekf,
        slam_toolbox,
        amcl,
        mode_manager,
        rviz,
        # give TF / map a moment before Nav2 and the explorer start
        TimerAction(period=3.0, actions=[navigation, explorer]),
    ])
