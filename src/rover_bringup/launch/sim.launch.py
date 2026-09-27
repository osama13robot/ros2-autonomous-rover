"""Everything for the simulated rover in one command:

    ros2 launch rover_bringup sim.launch.py

then, in a second terminal, drive / switch modes with the keyboard:

    ros2 run rover_control teleop_keyboard

Arguments:
    world:=rover_world.sdf | empty.sdf     headless:=false
    slam:=true (map:=... when false)       explore:=true
    initial_mode:=manual | auto            rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('rover_gazebo')
    bringup_pkg = get_package_share_directory('rover_bringup')
    control_params = os.path.join(bringup_pkg, 'config', 'control.yaml')

    arg_names = {
        'world': ('rover_world.sdf', 'World file in rover_gazebo/worlds'),
        'headless': ('false', 'Run Gazebo without its GUI'),
        'x': ('0.0', 'Spawn x'), 'y': ('0.0', 'Spawn y'), 'yaw': ('0.0', 'Spawn yaw'),
        'slam': ('true', 'Build a map with SLAM (false: AMCL on `map`)'),
        'map': ('', 'Map yaml for slam:=false'),
        'explore': ('true', 'Explore automatically when in AUTO mode'),
        'initial_mode': ('manual', 'manual | auto'),
        'rviz': ('true', 'Start RViz'),
    }
    args = [DeclareLaunchArgument(n, default_value=d, description=h) for n, (d, h) in arg_names.items()]
    cfg = {n: LaunchConfiguration(n) for n in arg_names}

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch', 'sim.launch.py')),
        launch_arguments={k: cfg[k] for k in ('world', 'headless', 'x', 'y', 'yaw')}.items())

    adapter = Node(package='rover_control', executable='sim_sensor_adapter', output='screen',
                   parameters=[control_params, {'use_sim_time': True}])

    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'autonomy.launch.py')),
        launch_arguments={'use_sim_time': 'true',
                          **{k: cfg[k] for k in ('slam', 'map', 'explore', 'initial_mode', 'rviz')}}.items())

    return LaunchDescription(args + [simulation, adapter, autonomy])
