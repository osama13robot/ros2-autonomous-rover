"""Bring up the REAL rover (run on the robot's Raspberry Pi).

    ros2 launch rover_bringup robot.launch.py lidar:=true camera:=true rviz:=false

The same autonomy stack as in simulation is used. The hardware drivers must provide the
same topics the simulator provides (see docs/11_real_robot.md):

    publish   /scan                     sensor_msgs/LaserScan     frame lidar_link
    publish   /camera/depth/points      sensor_msgs/PointCloud2   frame camera_link or an optical frame in TF
    publish   /imu/data                 sensor_msgs/Imu           frame imu_link   (with covariances)
    publish   /wheel/odometry_cov       nav_msgs/Odometry         odom -> base_footprint twist (with covariances)
    publish   /joint_states             sensor_msgs/JointState    left/right_wheel_joint
    subscribe /cmd_vel                  geometry_msgs/Twist

Lidar and camera driver packages are not part of this workspace; install them with
    sudo apt install ros-jazzy-sllidar-ros2 ros-jazzy-realsense2-camera

NOTE: the driver sections below are a starting TEMPLATE (not tested on hardware). Check
topic names and frame ids with `ros2 topic list` / `ros2 run tf2_tools view_frames` and
adjust the remappings; e.g. the RealSense cloud frame must exist in the URDF TF tree.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_pkg = get_package_share_directory('rover_description')
    bringup_pkg = get_package_share_directory('rover_bringup')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(description_pkg, 'urdf', 'rover.urdf.xacro'), ' sim_gazebo:=false']),
        value_type=str)

    args = [
        DeclareLaunchArgument('lidar', default_value='false', description='Start the RPLIDAR driver'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('camera', default_value='false', description='Start the RealSense driver'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('explore', default_value='false'),
        DeclareLaunchArgument('initial_mode', default_value='manual'),
        DeclareLaunchArgument('rviz', default_value='false', description='Usually RViz runs on a laptop'),
    ]

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description}])

    lidar = Node(
        package='sllidar_ros2', executable='sllidar_node', name='lidar', output='screen',
        parameters=[{'serial_port': LaunchConfiguration('lidar_port'), 'serial_baudrate': 460800,
                     'frame_id': 'lidar_link', 'angle_compensate': True, 'scan_mode': 'Standard'}],
        condition=IfCondition(LaunchConfiguration('lidar')))

    # RealSense: its own TF is disabled, the URDF already places camera_link.
    camera = Node(
        package='realsense2_camera', executable='realsense2_camera_node', name='camera',
        namespace='camera', output='screen',
        parameters=[{'publish_tf': False, 'base_frame_id': 'camera_link',
                     'pointcloud.enable': True, 'align_depth.enable': True,
                     'depth_module.depth_profile': '424x240x15', 'rgb_camera.color_profile': '424x240x15',
                     'decimation_filter.enable': True}],
        remappings=[('depth/color/points', '/camera/depth/points')],
        condition=IfCondition(LaunchConfiguration('camera')))

    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'autonomy.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            **{k: LaunchConfiguration(k) for k in ('slam', 'map', 'explore', 'initial_mode', 'rviz')},
        }.items())

    return LaunchDescription(args + [robot_state_publisher, lidar, camera, autonomy])
