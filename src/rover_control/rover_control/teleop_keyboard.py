"""Keyboard driving + mode switching. Run it in its own terminal:

    ros2 run rover_control teleop_keyboard

Publishes /cmd_vel_teleop (geometry_msgs/Twist) and /mode_request (std_msgs/String).
Hold a movement key to drive; releasing it stops the robot (a terminal cannot see key
releases, so motion stops `key_timeout` seconds after the last key repeat).
Set the parameter hold_to_drive:=false for classic "latched" behaviour instead.
"""
import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

HELP = r"""
 ---------------------------------------------------------------
   ROVER KEYBOARD CONTROL
 ---------------------------------------------------------------
   Drive (hold):        q   w   e          w/x : forward / back
                        a   s   d          a/d : rotate left / right
                        z   x   c          q/e/z/c : curves
   Stop:  s or SPACE    (SPACE also drops out of AUTO mode)

   Speed:  r / f  linear  +/- 10 %      t / g  angular +/- 10 %

   Mode:   m  toggle MANUAL <-> AUTO     1  MANUAL     2  AUTO
           o  start / pause frontier exploration (in AUTO)

   Arrow keys work too.  A drive key pressed in AUTO takes over (-> MANUAL).
   CTRL-C to quit
 ---------------------------------------------------------------
"""

# key: (linear sign, angular sign)
MOVES = {
    'w': (1, 0), 'x': (-1, 0), 'a': (0, 1), 'd': (0, -1),
    'q': (1, 1), 'e': (1, -1), 'z': (-1, -1), 'c': (-1, 1),
}

ARROWS = {'\x1b[A': 'w', '\x1b[B': 'x', '\x1b[C': 'd', '\x1b[D': 'a'}


class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('angular_speed', 1.2)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 2.5)
        self.declare_parameter('hold_to_drive', True)
        self.declare_parameter('key_timeout', 0.6)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('accel_linear', 1.0)     # m/s^2 ramp for smooth starts
        self.declare_parameter('accel_angular', 4.0)    # rad/s^2
        self.declare_parameter('take_over_auto', True)

        gp = lambda n: self.get_parameter(n).value
        self.lin, self.ang = gp('linear_speed'), gp('angular_speed')
        self.max_lin, self.max_ang = gp('max_linear'), gp('max_angular')
        self.hold, self.key_timeout = gp('hold_to_drive'), gp('key_timeout')
        self.dt = 1.0 / gp('rate')
        self.acc_lin, self.acc_ang = gp('accel_linear'), gp('accel_angular')
        self.take_over = gp('take_over_auto')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.mode_pub = self.create_publisher(String, '/mode_request', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.mode = '?'
        self.explore_state = '-'
        self.create_subscription(String, '/robot_mode', self.on_mode, latched)
        self.create_subscription(String, '/explore/status', self.on_explore, latched)
        self.explore_client = self.create_client(SetBool, '/explore/enable')

        self.target = (0, 0)          # (linear sign, angular sign)
        self.last_key_time = None
        self.v = 0.0
        self.w = 0.0

    # ---------------- subscriptions ----------------
    def on_mode(self, msg):
        self.mode = msg.data

    def on_explore(self, msg):
        self.explore_state = msg.data

    # ---------------- keys ----------------
    def handle_key(self, key):
        now = self.get_clock().now()
        if key in MOVES:
            if self.mode == 'AUTO' and self.take_over:
                self.request_mode('manual')
            self.target = MOVES[key]
            self.last_key_time = now
        elif key in ('s', ' '):
            self.target = (0, 0)
            self.v = self.w = 0.0
            if key == ' ' and self.mode == 'AUTO':
                self.request_mode('manual')
        elif key == 'r':
            self.lin = min(self.lin * 1.1, self.max_lin)
        elif key == 'f':
            self.lin = max(self.lin / 1.1, 0.02)
        elif key == 't':
            self.ang = min(self.ang * 1.1, self.max_ang)
        elif key == 'g':
            self.ang = max(self.ang / 1.1, 0.1)
        elif key == 'm':
            self.request_mode('toggle')
        elif key == '1':
            self.request_mode('manual')
        elif key == '2':
            self.request_mode('auto')
        elif key == 'o':
            self.toggle_exploration()

    def request_mode(self, what):
        self.target = (0, 0)
        self.v = self.w = 0.0
        self.mode_pub.publish(String(data=what))

    def toggle_exploration(self):
        if not self.explore_client.service_is_ready():
            self.explore_state = 'explorer not running'
            return
        enable = not self.explore_state.startswith(('EXPLORING', 'WAITING'))
        self.explore_client.call_async(SetBool.Request(data=enable))

    # ---------------- control loop ----------------
    def step(self):
        if self.hold and self.last_key_time is not None and self.target != (0, 0):
            if (self.get_clock().now() - self.last_key_time).nanoseconds * 1e-9 > self.key_timeout:
                self.target = (0, 0)
        v_goal = self.target[0] * self.lin
        w_goal = self.target[1] * self.ang
        self.v = self.ramp(self.v, v_goal, self.acc_lin * self.dt)
        self.w = self.ramp(self.w, w_goal, self.acc_ang * self.dt)
        if self.mode != 'AUTO':
            msg = Twist()
            msg.linear.x = self.v
            msg.angular.z = self.w
            self.cmd_pub.publish(msg)

    @staticmethod
    def ramp(current, goal, max_step):
        if goal == 0.0:          # stop immediately, no coasting
            return 0.0
        return current + max(-max_step, min(max_step, goal - current))

    def status_line(self):
        mode = self.mode
        colour = '\033[1;32m' if mode == 'AUTO' else '\033[1;33m'
        return (f'\r\033[K {colour}[{mode}]\033[0m  v={self.v:+.2f} m/s  w={self.w:+.2f} rad/s  '
                f'| speed {self.lin:.2f} m/s, {self.ang:.2f} rad/s  | explore: {self.explore_state}')


def main(args=None):
    if not sys.stdin.isatty():
        print('teleop_keyboard needs an interactive terminal: run it with `ros2 run`, not in a launch file.')
        return 1
    rclpy.init(args=args)
    node = TeleopKeyboard()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            ready, _, _ = select.select([sys.stdin], [], [], node.dt)
            if ready:
                keys = os.read(sys.stdin.fileno(), 64).decode(errors='ignore')
                # several bytes = auto-repeat burst or an arrow key; the last key wins
                for seq, key in ARROWS.items():
                    keys = keys.replace(seq, key)
                if keys:
                    node.handle_key(keys[-1].lower())
            node.step()
            sys.stdout.write(node.status_line())
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print('\nteleop stopped')
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
