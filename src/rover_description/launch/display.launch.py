"""Inspect the robot model in RViz (no simulator): wheels can be turned with the GUI sliders.

    ros2 launch rover_description display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('rover_description')
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg, 'urdf', 'rover.urdf.xacro'),
                 ' sim_gazebo:=false use_meshes:=', LaunchConfiguration('use_meshes')]),
        value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('use_meshes', default_value='true',
                              description='Show the 3D-printed STL parts (false: simple shapes)'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'display.rviz')]),
    ])
