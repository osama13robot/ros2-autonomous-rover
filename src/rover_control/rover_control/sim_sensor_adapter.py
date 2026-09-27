"""Make the raw Gazebo sensor streams look like real ROS drivers (simulation only).

* Wheel odometry  /wheel/odometry        -> /wheel/odometry_cov  (adds covariances)
* IMU             /imu                   -> /imu/data            (adds covariances)
* Depth cloud     /camera/depth/points_raw -> /camera/depth/points
    Gazebo's RGB-D sensor publishes the point cloud with x-forward (body) axes while
    stamping it with the optical frame id. The cloud is re-stamped with the body frame
    (camera_link) and down-sampled (every Nth pixel) to keep the costmaps fast.

On the real robot, the motor/IMU/camera drivers publish these topics directly.
"""
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


class SimSensorAdapter(Node):

    def __init__(self):
        super().__init__('sim_sensor_adapter')
        self.declare_parameter('cloud_frame', 'camera_link')
        self.declare_parameter('cloud_stride', 2)
        # 1-sigma values
        self.declare_parameter('odom_linear_sigma', 0.03)      # m/s
        self.declare_parameter('odom_angular_sigma', 0.10)     # rad/s
        self.declare_parameter('imu_gyro_sigma', 0.02)         # rad/s
        self.declare_parameter('imu_accel_sigma', 0.2)         # m/s^2

        self.cloud_frame = self.get_parameter('cloud_frame').value
        self.stride = max(1, int(self.get_parameter('cloud_stride').value))
        lin = self.get_parameter('odom_linear_sigma').value ** 2
        ang = self.get_parameter('odom_angular_sigma').value ** 2
        gyro = self.get_parameter('imu_gyro_sigma').value ** 2
        acc = self.get_parameter('imu_accel_sigma').value ** 2

        self.odom_twist_cov = np.diag([lin, lin, 1e3, 1e3, 1e3, ang]).flatten().tolist()
        self.odom_pose_cov = np.diag([0.01, 0.01, 1e3, 1e3, 1e3, 0.05]).flatten().tolist()
        self.gyro_cov = np.diag([gyro] * 3).flatten().tolist()
        self.accel_cov = np.diag([acc] * 3).flatten().tolist()

        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry_cov', 20)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, '/camera/depth/points', qos_profile_sensor_data)

        self.create_subscription(Odometry, '/wheel/odometry', self.on_odom, 20)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/camera/depth/points_raw', self.on_cloud,
                                 qos_profile_sensor_data)

    def on_odom(self, msg: Odometry):
        msg.pose.covariance = self.odom_pose_cov
        msg.twist.covariance = self.odom_twist_cov
        self.odom_pub.publish(msg)

    def on_imu(self, msg: Imu):
        msg.orientation_covariance = [-1.0] + [0.0] * 8        # no absolute orientation (no magnetometer)
        msg.angular_velocity_covariance = self.gyro_cov
        msg.linear_acceleration_covariance = self.accel_cov
        self.imu_pub.publish(msg)

    def on_cloud(self, msg: PointCloud2):
        msg.header.frame_id = self.cloud_frame
        s = self.stride
        if s > 1 and msg.height > 1:
            data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.row_step)
            data = data[::s, :msg.width * msg.point_step].reshape(-1, msg.width, msg.point_step)[:, ::s, :]
            msg.height, msg.width = data.shape[0], data.shape[1]
            msg.row_step = msg.width * msg.point_step
            msg.data = data.tobytes()
        self.cloud_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimSensorAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
