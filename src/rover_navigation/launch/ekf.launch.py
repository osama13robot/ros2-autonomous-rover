"""EKF (robot_localization): wheel odometry + IMU -> /odometry/filtered and TF odom -> base_footprint."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = os.path.join(get_package_share_directory('rover_navigation'), 'config', 'ekf.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
        ),
    ])
