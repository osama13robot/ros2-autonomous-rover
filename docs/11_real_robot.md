# 11 · Building the real robot

> This guide uses the 2WD **rover**; for **rover4** add `robot:=rover4` to the launch commands and see
> [14 Rover4](14_rover4_ackermann.md#real-robot-firmware-for-ackermann) for the steering firmware.

> **Status:** the mechanical design and the autonomy stack are complete and tested *in simulation*.
> The hardware bring-up described here is a **plan and a template**: the driver launch file has not
> been run on physical hardware yet. Contributions from people who build it are very welcome.

## 1. Build the hardware

Follow the [Hardware guide](../hardware/README.md): print the parts, buy the BOM, assemble and wire.
**Before printing**, measure your actual ball casters, motors and sensors, update
`dimensions.yaml`, and regenerate the parts.

## 2. Set up the computer (Raspberry Pi 5)

* Install **Ubuntu Server 24.04** (64-bit) and **ROS 2 Jazzy** (`ros-jazzy-ros-base` is enough on
  the robot; run RViz on your laptop).
* Clone this repository and build it (skip `rover_gazebo` to save time):
  `colcon build --symlink-install --packages-skip rover_gazebo`.
* Put the robot and the laptop on the same network with the same `ROS_DOMAIN_ID`.

## 3. The driver topic contract

The autonomy stack (`rover_bringup/autonomy.launch.py`) is exactly the same in simulation and on
the real robot. It expects these topics, which the drivers must provide:

| Direction | Topic | Type | Requirements |
|---|---|---|---|
| publish | `/scan` | `sensor_msgs/LaserScan` | `frame_id: lidar_link` |
| publish | `/camera/depth/points` | `sensor_msgs/PointCloud2` | frame must exist in the TF tree |
| publish | `/imu/data` | `sensor_msgs/Imu` | `frame_id: imu_link`, fill covariances; orientation covariance `[-1, …]` if there is no magnetometer |
| publish | `/wheel/odometry_cov` | `nav_msgs/Odometry` | twist from the encoders, `odom → base_footprint`, realistic twist covariance, **no TF** (the EKF publishes it) |
| publish | `/joint_states` | `sensor_msgs/JointState` | `left_wheel_joint`, `right_wheel_joint` positions in radians |
| subscribe | `/cmd_vel` | `geometry_msgs/Twist` | stop the motors if no message arrives for 0.5 s |

In simulation, Gazebo + `sim_sensor_adapter` provide exactly this contract.

## 4. Lidar and camera drivers

```bash
sudo apt install ros-jazzy-sllidar-ros2 ros-jazzy-realsense2-camera
ros2 launch rover_bringup robot.launch.py lidar:=true camera:=true
```

`robot.launch.py` configures:

* **sllidar_ros2** (RPLIDAR C1: 460800 baud, `frame_id: lidar_link`)
* **realsense2_camera** (424 × 240 at 15 Hz, point cloud on, its own TF off)

Check the frames the camera actually publishes: `ros2 topic echo --once /camera/depth/points --field header`.
If the frame (for example `camera_depth_optical_frame`) isn't in the URDF, either add it as a child
of `camera_link` in `rover.urdf.xacro`, or set the driver's frame parameters to match.

## 5. Motor controller firmware (to be written)

The microcontroller (ESP32 or Raspberry Pi Pico) needs to:

1. read both encoders (quadrature, interrupts or the PCNT peripheral on the ESP32),
2. run a **PID speed loop** per wheel (typically 50–100 Hz),
3. convert `/cmd_vel` → wheel speeds:
   `v_left = v − ω·L/2`, `v_right = v + ω·L/2` (L = 0.182 m, wheel radius 0.035 m),
4. integrate wheel travel into odometry and publish `/wheel/odometry_cov` and `/joint_states`,
5. apply the 0.5 s command timeout (safety).

Two common ways to connect it:

* **micro-ROS** on the ESP32: it becomes a ROS 2 node directly (USB serial or Wi-Fi via the
  `micro_ros_agent`).
* **A serial protocol** plus a small Python bridge node on the Pi: simpler to debug.

## 6. Bring-up checklist

| Step | Check |
|---|---|
| Wheels | `ros2 topic pub /cmd_vel …` with 0.1 m/s: both wheels turn forward |
| Encoders | push the robot by hand 1 m: `/wheel/odometry_cov` says about 1 m |
| Turning | command 1 rad/s for 6.28 s: the robot turns one full circle |
| IMU | rotate by hand: `/imu/data` `angular_velocity.z` is positive counter-clockwise |
| Lidar | RViz, fixed frame `base_footprint`: walls appear where they really are |
| TF | `ros2 run tf2_tools view_frames` matches [03 Architecture](03_architecture.md#tf-tree-coordinate-frames) |
| EKF | drive a square: `/odometry/filtered` returns close to the start |
| SLAM | map a room slowly in MANUAL |
| Nav2 | send short goals first, with `vx_max` reduced to 0.15 |
| Safety | lift the robot, then kill the teleop: the wheels must stop within 0.5 s |

## 7. Calibration

* **Wheel radius:** drive 2 m straight and measure; scale the radius by `actual / reported`.
* **Wheel separation:** spin 10 turns in place and compare with the reported angle; scale the
  separation the same way.
* **IMU:** keep the robot still at start-up for the gyro bias estimate (BNO055 does this internally).
* **Camera extrinsics:** make sure the camera pose in `dimensions.yaml` matches the real mount.
  Place a box in front of the robot and check that the point cloud and the lidar agree in RViz.
