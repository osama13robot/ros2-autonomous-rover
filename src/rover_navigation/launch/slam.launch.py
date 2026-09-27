"""slam_toolbox online SLAM: builds /map and publishes TF map -> odom."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    params = os.path.join(get_package_share_directory('rover_navigation'), 'config', 'slam_toolbox.yaml')
    slam_launch = os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('slam_params_file', default_value=params),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': LaunchConfiguration('slam_params_file'),
            }.items(),
        ),
    ])
