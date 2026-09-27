"""Start Gazebo (Harmonic) with a world, spawn the rover and bridge its topics to ROS 2.

    ros2 launch rover_gazebo sim.launch.py
    ros2 launch rover_gazebo sim.launch.py world:=empty.sdf headless:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('rover_gazebo')
    pkg_description = get_package_share_directory('rover_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    x, y, yaw = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('yaw')

    args = [
        DeclareLaunchArgument('world', default_value='rover_world.sdf',
                              description='World file name inside rover_gazebo/worlds (or absolute path)'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run the Gazebo server only, without the GUI'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    # Let Gazebo resolve package://rover_description/meshes/... and the worlds folder
    resource_paths = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_description)),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg_gazebo, 'worlds')),
    ]

    world_path = PathJoinSubstitution([pkg_gazebo, 'worlds', world])

    gz_server_and_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v 2 ', world_path], 'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(headless),
    )
    gz_server_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s --headless-rendering -v 2 ', world_path],
                          'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(headless),
    )

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg_description, 'urdf', 'rover.urdf.xacro'), ' sim_gazebo:=true']),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'rover',
                   '-x', x, '-y', y, '-z', '0.01', '-Y', yaw],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': os.path.join(pkg_gazebo, 'config', 'gz_bridge.yaml'),
                     'use_sim_time': True}],
    )

    return LaunchDescription(args + resource_paths + [
        gz_server_and_gui, gz_server_only, robot_state_publisher, spawn, bridge,
    ])
