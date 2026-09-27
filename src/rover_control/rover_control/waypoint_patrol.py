"""Example: patrol a list of waypoints with the Nav2 Simple Commander API.

A small, readable starting point for writing your own autonomous behaviours.

    ros2 run rover_control waypoint_patrol
    ros2 run rover_control waypoint_patrol --ros-args -p loops:=0 \
        -p waypoints:="[1.0, 2.0, -4.5, 0.5, 4.5, -3.0]"      # x1, y1, x2, y2, ... (map frame)

It switches the robot to AUTO (via /mode_request), then visits each waypoint in turn.
loops:=0 patrols forever. Press a drive key in teleop_keyboard to take over at any time;
the patrol stops when its goal is cancelled.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String
from std_srvs.srv import SetBool

# a tour through the rooms of rover_world.sdf
DEFAULT_WAYPOINTS = [1.5, 2.0, -4.5, 0.5, -4.5, -3.8, 5.0, 3.0, 0.0, 0.0]


def make_pose(nav, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2)
    pose.pose.orientation.w = math.cos(yaw / 2)
    return pose


def main():
    rclpy.init()
    nav = BasicNavigator(node_name='waypoint_patrol')
    nav.declare_parameter('waypoints', DEFAULT_WAYPOINTS)
    nav.declare_parameter('loops', 1)
    flat = list(nav.get_parameter('waypoints').value)
    loops = int(nav.get_parameter('loops').value)
    if len(flat) < 2 or len(flat) % 2:
        nav.get_logger().error('waypoints must be a flat list: [x1, y1, x2, y2, ...]')
        return 1
    points = list(zip(flat[0::2], flat[1::2]))

    # SLAM or AMCL both work: only wait for the Nav2 navigation servers here
    nav.waitUntilNav2Active(localizer='robot_localization')

    # our own exploration would compete for Nav2: pause it, then take the robot to AUTO
    explore = nav.create_client(SetBool, '/explore/enable')
    if explore.wait_for_service(timeout_sec=1.0):
        explore.call_async(SetBool.Request(data=False))
    nav.create_publisher(String, '/mode_request', 10).publish(String(data='auto'))

    lap = 0
    try:
        while loops == 0 or lap < loops:
            lap += 1
            for i, (x, y) in enumerate(points):
                nx, ny = points[(i + 1) % len(points)]
                goal = make_pose(nav, x, y, math.atan2(ny - y, nx - x))
                nav.get_logger().info(f'Lap {lap}: waypoint {i + 1}/{len(points)} -> ({x:.1f}, {y:.1f})')
                nav.goToPose(goal)
                while not nav.isTaskComplete():
                    rclpy.spin_once(nav, timeout_sec=0.2)
                result = nav.getResult()
                if result == TaskResult.CANCELED:
                    nav.get_logger().info('Goal cancelled (manual take-over?): stopping the patrol')
                    return 0
                if result == TaskResult.FAILED:
                    nav.get_logger().warn(f'Could not reach ({x:.1f}, {y:.1f}), skipping it')
        nav.get_logger().info('Patrol finished')
    except KeyboardInterrupt:
        nav.cancelTask()
    finally:
        nav.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
