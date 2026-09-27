"""Manual / autonomous mode switch (velocity multiplexer).

                 /cmd_vel_teleop  (keyboard)  --.
                                                 +--> [mode_manager] --> /cmd_vel --> robot
    Nav2 --> collision_monitor --> /cmd_vel_nav --'

Only the source that matches the current mode reaches the robot. If the selected source
goes quiet, a zero command is sent (dead-man behaviour).

Interfaces
    /robot_mode      std_msgs/String  (latched)  current mode: "MANUAL" or "AUTO"
    /mode_request    std_msgs/String             "manual", "auto" or "toggle"
    /set_autonomous  std_srvs/SetBool            true = AUTO, false = MANUAL

When switching to MANUAL all Nav2 goals are cancelled so the robot does not resume an
old goal later by surprise.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

MANUAL, AUTO = 'MANUAL', 'AUTO'
NAV_ACTIONS = ['/navigate_to_pose', '/navigate_through_poses', '/follow_waypoints']


class ModeManager(Node):

    def __init__(self):
        super().__init__('mode_manager')
        self.declare_parameter('initial_mode', 'manual')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('teleop_timeout', 0.5)
        self.declare_parameter('nav_timeout', 0.5)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 2.5)
        self.declare_parameter('cancel_nav_on_manual', True)

        self.teleop_timeout = self.get_parameter('teleop_timeout').value
        self.nav_timeout = self.get_parameter('nav_timeout').value
        self.max_lin = self.get_parameter('max_linear').value
        self.max_ang = self.get_parameter('max_angular').value
        self.cancel_on_manual = self.get_parameter('cancel_nav_on_manual').value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.mode_pub = self.create_publisher(String, '/robot_mode', latched)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last = {MANUAL: (None, None), AUTO: (None, None)}   # (msg, time)
        self.create_subscription(Twist, '/cmd_vel_teleop', lambda m: self.on_cmd(MANUAL, m), 10)
        self.create_subscription(Twist, '/cmd_vel_nav', lambda m: self.on_cmd(AUTO, m), 10)
        self.create_subscription(String, '/mode_request', self.on_request, 10)
        self.create_service(SetBool, '/set_autonomous', self.on_set_autonomous)

        self.cancel_clients = [self.create_client(CancelGoal, f'{a}/_action/cancel_goal')
                               for a in NAV_ACTIONS]

        initial = str(self.get_parameter('initial_mode').value).upper()
        self.mode = None
        self.set_mode(AUTO if initial == AUTO else MANUAL)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)

    # ------------------------------------------------------------------
    def on_cmd(self, source, msg):
        self.last[source] = (msg, self.get_clock().now())

    def on_request(self, msg: String):
        req = msg.data.strip().lower()
        if req == 'toggle':
            self.set_mode(MANUAL if self.mode == AUTO else AUTO)
        elif req in ('auto', 'autonomous'):
            self.set_mode(AUTO)
        elif req == 'manual':
            self.set_mode(MANUAL)
        else:
            self.get_logger().warn(f'Unknown mode request "{msg.data}" (use manual/auto/toggle)')

    def on_set_autonomous(self, request, response):
        self.set_mode(AUTO if request.data else MANUAL)
        response.success = True
        response.message = self.mode
        return response

    def set_mode(self, mode):
        if mode == self.mode:
            self.mode_pub.publish(String(data=mode))
            return
        previous, self.mode = self.mode, mode
        # never carry a stale command across a switch
        self.last = {MANUAL: (None, None), AUTO: (None, None)}
        self.cmd_pub.publish(Twist())
        self.mode_pub.publish(String(data=mode))
        self.get_logger().info(f'Mode: {previous} -> {mode}')
        if mode == MANUAL and previous == AUTO and self.cancel_on_manual:
            self.cancel_all_nav_goals()

    def cancel_all_nav_goals(self):
        # An all-zero goal id and stamp cancels every goal on the action server
        for client in self.cancel_clients:
            if client.service_is_ready():
                client.call_async(CancelGoal.Request())

    # ------------------------------------------------------------------
    def tick(self):
        msg, stamp = self.last[self.mode]
        timeout = self.teleop_timeout if self.mode == MANUAL else self.nav_timeout
        fresh = stamp is not None and \
            (self.get_clock().now() - stamp).nanoseconds * 1e-9 < timeout
        out = Twist()   # a steady zero stream: a lost message never leaves the robot moving
        if fresh:
            out.linear.x = self.clamp(msg.linear.x, self.max_lin)
            out.angular.z = self.clamp(msg.angular.z, self.max_ang)
        self.cmd_pub.publish(out)

    @staticmethod
    def clamp(value, limit):
        if not math.isfinite(value):
            return 0.0
        return max(-limit, min(limit, value))


def main(args=None):
    rclpy.init(args=args)
    node = ModeManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # a callback that was still running when Ctrl-C shut ROS down may fail (for example
        # publishing on an invalidated context); that is a normal exit, not a crash
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
