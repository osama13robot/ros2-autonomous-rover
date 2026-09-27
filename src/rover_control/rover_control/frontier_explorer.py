"""Autonomous frontier exploration on top of SLAM + Nav2.

A *frontier* is the border between known free space and unknown space in the SLAM map.
The explorer repeatedly picks the most attractive frontier (big and close), sends it to
Nav2 (navigate_to_pose) and re-targets as soon as the frontier has been uncovered. When no
reachable frontier is left, the map is complete and the robot drives back to where it
started.

It only acts while the robot is in AUTO mode (see mode_manager). If someone sends their own
Nav2 goal (e.g. "2D Goal Pose" in RViz), exploration pauses and that goal is executed.

Interfaces
    /explore/enable     std_srvs/SetBool           start (true) / pause (false)
    /explore/status     std_msgs/String (latched)  human-readable state
    /explore/frontiers  visualization_msgs/MarkerArray
"""
import math
import uuid

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy import ndimage
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from unique_identifier_msgs.msg import UUID
from visualization_msgs.msg import Marker, MarkerArray

ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')
        p = self.declare_parameter
        p('enabled_on_start', True)
        p('global_frame', 'map')
        p('robot_frame', 'base_footprint')
        p('planner_period', 1.5)          # s between re-evaluations
        p('min_frontier_size', 0.40)      # m, smaller frontiers are ignored
        p('clearance', 0.30)              # m, goals closer to obstacles are skipped
        p('min_goal_distance', 0.5)       # m, ignore frontiers right under the robot
        p('distance_weight', 1.0)         # cost per metre of distance
        p('size_weight', 0.6)             # reward per metre of frontier length
        p('blacklist_radius', 0.5)        # m, around goals Nav2 failed to reach
        p('goal_timeout', 90.0)           # s before a goal is abandoned
        p('stuck_timeout', 20.0)          # s without real progress before a goal is abandoned
        p('reached_radius', 0.45)         # m, close enough: if still a frontier it is unobservable
        p('return_home', True)
        p('done_confirmations', 5)        # empty evaluations in a row before declaring the map done

        gp = lambda n: self.get_parameter(n).value
        self.enabled = gp('enabled_on_start')
        self.global_frame, self.robot_frame = gp('global_frame'), gp('robot_frame')
        self.min_size, self.clearance = gp('min_frontier_size'), gp('clearance')
        self.min_goal_dist = gp('min_goal_distance')
        self.w_dist, self.w_size = gp('distance_weight'), gp('size_weight')
        self.blacklist_radius, self.goal_timeout = gp('blacklist_radius'), gp('goal_timeout')
        self.return_home = gp('return_home')
        self.done_confirmations = gp('done_confirmations')
        self.stuck_timeout, self.reached_radius = gp('stuck_timeout'), gp('reached_radius')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status_pub = self.create_publisher(String, '/explore/status', latched)
        self.marker_pub = self.create_publisher(MarkerArray, '/explore/frontiers', 1)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(String, '/robot_mode', self.on_mode, latched)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 self.on_nav_status, latched)
        self.create_service(SetBool, '/explore/enable', self.on_enable)

        self.map = None
        self.mode = 'MANUAL'
        self.home = None
        self.target = None           # (x, y) of the current goal
        self.target_is_home = False
        self.goal_handle = None
        self.goal_sent = None
        self.my_goal_ids = set()
        self.blacklist = []
        self.status = ''
        self.empty_count = 0
        self.retry_after = None
        self.progress_pose = None    # (x, y, time) of the last significant movement
        self.create_timer(gp('planner_period'), self.plan)
        self.set_status('WAITING for map' if self.enabled else 'DISABLED')

    # ------------------------------------------------------------ callbacks
    def on_map(self, msg):
        self.map = msg

    def on_mode(self, msg):
        self.mode = msg.data
        if self.mode != 'AUTO':
            self.forget_goal()

    def on_enable(self, request, response):
        self.enabled = bool(request.data)
        if self.enabled:
            self.blacklist.clear()
            self.empty_count = 0
            self.set_status('EXPLORING (starting)')
        else:
            self.cancel_goal()
            self.set_status('DISABLED')
        response.success = True
        response.message = self.status
        return response

    def on_nav_status(self, msg: GoalStatusArray):
        """Pause exploration when a goal that is not ours becomes active."""
        for st in msg.status_list:
            gid = bytes(st.goal_info.goal_id.uuid)
            if st.status in ACTIVE and gid not in self.my_goal_ids:
                self.my_goal_ids.add(gid)          # react only once per foreign goal
                if self.enabled:
                    self.enabled = False
                    self.forget_goal()
                    self.set_status('PAUSED (user goal) - press "o" to resume')
                    self.get_logger().info('External navigation goal received: exploration paused')

    # ------------------------------------------------------------ main loop
    def plan(self):
        if not self.enabled:
            return
        if self.mode != 'AUTO':
            self.set_status('WAITING (MANUAL mode) - switch to AUTO')
            return
        if self.map is None:
            self.set_status('WAITING for map')
            return
        pose = self.robot_xy()
        if pose is None:
            return
        if self.home is None:
            self.home = pose
        if not self.nav.server_is_ready() or \
                (self.retry_after is not None and self.get_clock().now() < self.retry_after):
            self.set_status('WAITING for Nav2')
            return

        grid, info = self.map_array()
        frontiers, cells = self.find_frontiers(grid, info)
        self.publish_markers(cells, frontiers, info)

        # still busy with a goal?
        if self.target is not None:
            reason = self.goal_problem(pose)
            if reason is not None:
                self.get_logger().info(f'Abandoning ({self.target[0]:.1f}, {self.target[1]:.1f}): {reason}')
                self.blacklist.append(self.target)
                self.cancel_goal()
            elif self.target_is_home or self.still_frontier(grid, info, self.target):
                return
            # the frontier was uncovered on the way: pick the next one right away

        candidates = []
        for (gx, gy, size) in frontiers:
            d = math.hypot(gx - pose[0], gy - pose[1])
            if d < self.min_goal_dist or self.blacklisted(gx, gy):
                continue
            candidates.append((self.w_dist * d - self.w_size * size, gx, gy))

        if not candidates:
            self.empty_count += 1
            # confirm over several map updates: right after start-up the map is nearly empty
            if self.target is None and self.empty_count >= self.done_confirmations:
                self.finish(pose)
            return
        self.empty_count = 0

        _, gx, gy = min(candidates)
        if self.target is not None and math.hypot(gx - self.target[0], gy - self.target[1]) < 0.3:
            return
        self.send_goal(gx, gy, pose)

    def goal_problem(self, pose):
        now = self.get_clock().now()
        if (now - self.goal_sent).nanoseconds * 1e-9 > self.goal_timeout:
            return 'timeout'
        if self.target_is_home:
            return None
        if math.hypot(pose[0] - self.target[0], pose[1] - self.target[1]) < self.reached_radius:
            return 'reached but still unknown (occluded)'
        px, py, pt = self.progress_pose
        if math.hypot(pose[0] - px, pose[1] - py) > 0.25:
            self.progress_pose = (pose[0], pose[1], now)
        elif (now - pt).nanoseconds * 1e-9 > self.stuck_timeout:
            return 'no progress'
        return None

    def finish(self, pose):
        if self.return_home and self.home is not None and \
                math.hypot(pose[0] - self.home[0], pose[1] - self.home[1]) > 0.5:
            self.get_logger().info('Exploration complete: returning to the start position')
            self.send_goal(self.home[0], self.home[1], pose, home=True)
            self.set_status('RETURNING HOME (map complete)')
        else:
            self.get_logger().info('Exploration complete')
            self.enabled = False
            self.set_status('DONE - map complete (save it with the map_saver)')

    # ------------------------------------------------------------ frontiers
    def map_array(self):
        info = self.map.info
        grid = np.asarray(self.map.data, dtype=np.int8).reshape(info.height, info.width)
        return grid, info

    def find_frontiers(self, grid, info):
        res = info.resolution
        unknown = grid < 0
        free = (grid >= 0) & (grid < 25)
        occupied = grid >= 65
        # frontier = free cell touching unknown space (4-neighbourhood)
        frontier = free & ndimage.binary_dilation(unknown, structure=ndimage.generate_binary_structure(2, 1))
        # keep the robot away from walls
        if occupied.any():
            clearance = ndimage.distance_transform_edt(~occupied) * res
            frontier &= clearance > self.clearance
        labels, n = ndimage.label(frontier, structure=np.ones((3, 3)))
        result = []
        if n == 0:
            return result, np.empty((0, 2))
        idx = np.arange(1, n + 1)
        sizes = ndimage.sum(frontier, labels, idx)
        centroids = ndimage.center_of_mass(frontier, labels, idx)
        rows, cols = np.nonzero(frontier)
        cell_labels = labels[rows, cols]
        for lbl, size, (cr, cc) in zip(idx, sizes, centroids):
            if size * res < self.min_size:
                continue
            members = cell_labels == lbl
            r, c = rows[members], cols[members]
            k = np.argmin((r - cr) ** 2 + (c - cc) ** 2)   # frontier cell closest to the centroid
            x, y = self.cell_to_world(r[k], c[k], info)
            result.append((x, y, size * res))
        cells = np.column_stack([rows, cols])
        return result, cells

    def still_frontier(self, grid, info, xy, radius=0.35):
        r, c = self.world_to_cell(xy[0], xy[1], info)
        k = int(radius / info.resolution)
        window = grid[max(r - k, 0):r + k + 1, max(c - k, 0):c + k + 1]
        return int((window < 0).sum()) > 3

    def blacklisted(self, x, y):
        return any(math.hypot(x - bx, y - by) < self.blacklist_radius for bx, by in self.blacklist)

    @staticmethod
    def cell_to_world(r, c, info):
        # assumes an axis-aligned map (true for slam_toolbox and map_server)
        return (info.origin.position.x + (c + 0.5) * info.resolution,
                info.origin.position.y + (r + 0.5) * info.resolution)

    @staticmethod
    def world_to_cell(x, y, info):
        return (int((y - info.origin.position.y) / info.resolution),
                int((x - info.origin.position.x) / info.resolution))

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time(),
                                                timeout=Duration(seconds=0.1))
        except TransformException as ex:
            self.set_status(f'WAITING for TF ({self.global_frame} -> {self.robot_frame})')
            self.get_logger().debug(str(ex))
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    # ------------------------------------------------------------ Nav2
    def send_goal(self, x, y, pose, home=False):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        yaw = math.atan2(y - pose[1], x - pose[0])        # arrive facing the unknown
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        gid = uuid.uuid4().bytes
        self.my_goal_ids.add(gid)
        self.target = (x, y)
        self.target_is_home = home
        self.goal_sent = self.get_clock().now()
        self.progress_pose = (pose[0], pose[1], self.goal_sent)
        if not home:
            self.set_status(f'EXPLORING -> ({x:.1f}, {y:.1f})')
        future = self.nav.send_goal_async(goal, goal_uuid=UUID(uuid=list(gid)))
        future.add_done_callback(lambda f, target=(x, y): self.on_goal_response(f, target))

    def on_goal_response(self, future, target):
        handle = future.result()
        if not handle.accepted:
            # usually Nav2 is still activating: try again shortly
            self.get_logger().warn('Nav2 rejected the goal, retrying in 3 s')
            self.retry_after = self.get_clock().now() + Duration(seconds=3.0)
            if self.target == target:
                self.forget_goal()
            return
        if self.target != target:          # superseded while waiting for the answer
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, target))

    def on_result(self, future, target):
        status = future.result().status
        if self.target != target:          # preempted by a newer goal of ours: not a failure
            return
        if status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info(f'Could not reach ({target[0]:.1f}, {target[1]:.1f}), blacklisting it')
            self.blacklist.append(target)
        was_home = self.target_is_home
        self.forget_goal()
        if was_home and status == GoalStatus.STATUS_SUCCEEDED:
            self.enabled = False
            self.set_status('DONE - map complete, robot is home')
            self.get_logger().info('Back home. Exploration finished.')

    def cancel_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.forget_goal()

    def forget_goal(self):
        self.target = None
        self.target_is_home = False
        self.goal_handle = None

    # ------------------------------------------------------------ output
    def set_status(self, text):
        if text != self.status:
            self.status = text
            self.status_pub.publish(String(data=text))

    def publish_markers(self, cells, frontiers, info):
        stamp = self.get_clock().now().to_msg()
        pts = Marker()
        pts.header.frame_id = self.global_frame
        pts.header.stamp = stamp
        pts.ns, pts.id, pts.type = 'frontier_cells', 0, Marker.POINTS
        pts.scale.x = pts.scale.y = info.resolution
        pts.color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.8)
        for r, c in cells[::2]:
            x, y = self.cell_to_world(r, c, info)
            pts.points.append(Point(x=x, y=y, z=0.02))

        goals = Marker()
        goals.header = pts.header
        goals.ns, goals.id, goals.type = 'frontier_goals', 1, Marker.SPHERE_LIST
        goals.scale.x = goals.scale.y = goals.scale.z = 0.15
        goals.color = ColorRGBA(r=0.1, g=0.4, b=1.0, a=0.9)
        goals.points = [Point(x=x, y=y, z=0.05) for x, y, _ in frontiers]

        target = Marker()
        target.header = pts.header
        target.ns, target.id, target.type = 'target', 2, Marker.CYLINDER
        if self.target is not None:
            target.action = Marker.ADD
            target.pose.position.x, target.pose.position.y = self.target
            target.pose.position.z = 0.15
            target.pose.orientation.w = 1.0
            target.scale.x = target.scale.y = 0.25
            target.scale.z = 0.3
            target.color = ColorRGBA(r=1.0, g=0.2, b=0.1, a=0.8)
        else:
            target.action = Marker.DELETE
        self.marker_pub.publish(MarkerArray(markers=[pts, goals, target]))


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
