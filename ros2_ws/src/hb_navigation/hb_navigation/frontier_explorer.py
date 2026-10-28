"""frontier_explorer: drive around until the whole area is mapped.

Every update:
  1. read the SLAM map (/map) and the robot pose (TF map -> base_footprint)
  2. find frontiers (edges between free and unknown space)
  3. pick the best one: close and long is good; skip spots already tried
  4. send it to Nav2 (NavigateToPose) and watch the robot:
       - reached / failed       -> remember the spot, pick the next frontier
       - not getting closer     -> give up on it, pick the next
       - frontier disappeared   -> the lidar already saw it, pick the next
  5. no frontiers left -> exploration done, drive back to the start

Topics
  /map                        nav_msgs/OccupancyGrid   (input)
  /explore/frontiers          visualization_msgs/MarkerArray  (for RViz)
  navigate_to_pose            nav2_msgs/action/NavigateToPose (Nav2)
"""
import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from hb_navigation.frontier_search import find_frontiers

IDLE, NAVIGATING, RETURNING, DONE = 'idle', 'navigating', 'returning home', 'done'


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')
        p = self.declare_parameter
        self.global_frame = p('global_frame', 'map').value
        self.robot_frame = p('robot_frame', 'base_footprint').value
        rate = p('update_rate', 1.0).value
        self.min_frontier_size = p('min_frontier_size', 0.25).value
        self.clearance = p('clearance', 0.12).value
        self.distance_weight = p('distance_weight', 1.0).value
        self.size_weight = p('size_weight', 0.5).value
        self.blacklist_radius = p('blacklist_radius', 0.3).value
        self.progress_timeout = p('progress_timeout', 15.0).value
        self.frontier_lost_radius = p('frontier_lost_radius', 0.25).value
        self.return_to_start = p('return_to_start', True).value
        self.empty_checks_to_finish = p('empty_checks_to_finish', 3).value
        self.save_map_path = p('save_map_path', '').value

        self.map = None
        self.start_xy = None
        self.state = IDLE
        self.goal_xy = None
        self.goal_handle = None
        self.goal_seq = 0
        self.canceling = False
        self.best_dist = math.inf
        self.last_progress = None
        self.blacklist = []
        self.empty_checks = 0
        self.goals_sent = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, map_qos)
        self.marker_pub = self.create_publisher(MarkerArray, 'explore/frontiers', 1)
        self.timer = self.create_timer(1.0 / rate, self.update)
        self.get_logger().info('frontier explorer started, waiting for map and Nav2')

    # ------------------------------------------------------------------ inputs
    def on_map(self, msg):
        self.map = msg

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def current_frontiers(self):
        info = self.map.info
        grid = np.array(self.map.data, dtype=np.int16).reshape(info.height, info.width)
        return find_frontiers(grid, info.resolution, info.origin.position.x, info.origin.position.y,
                              clearance=self.clearance, min_size=self.min_frontier_size)

    # ------------------------------------------------------------------ main loop
    def update(self):
        if self.state == DONE:
            return
        if self.map is None:
            self.get_logger().info('waiting for /map ...', throttle_duration_sec=5.0)
            return
        robot = self.robot_xy()
        if robot is None:
            self.get_logger().info('waiting for TF map -> base_footprint ...', throttle_duration_sec=5.0)
            return
        if not self.nav.server_is_ready():
            self.get_logger().info('waiting for Nav2 (navigate_to_pose) ...', throttle_duration_sec=5.0)
            return
        if self.start_xy is None:
            self.start_xy = robot

        frontiers = self.current_frontiers()
        self.publish_markers(frontiers)

        if self.state == NAVIGATING:
            self.watch_progress(robot, frontiers)
            return
        if self.state == RETURNING:
            return

        candidates = [f for f in frontiers if not self.is_blacklisted(f.x, f.y)]
        if not candidates:
            self.empty_checks += 1
            if self.empty_checks >= self.empty_checks_to_finish:
                self.finish()
            return
        self.empty_checks = 0

        best = min(candidates, key=lambda f: self.score(f, robot))
        self.get_logger().info(
            f'{len(candidates)} frontiers; going to ({best.x:.2f}, {best.y:.2f}), '
            f'{best.size:.2f} m long, {math.dist(robot, (best.x, best.y)):.2f} m away')
        self.send_goal(best.x, best.y, robot, NAVIGATING)

    def score(self, f, robot):
        """Lower is better: prefer near frontiers, and long ones (more to discover)."""
        return self.distance_weight * math.dist(robot, (f.x, f.y)) - self.size_weight * f.size

    def watch_progress(self, robot, frontiers):
        if self.canceling:
            return
        now = self.get_clock().now()
        d = math.dist(robot, self.goal_xy)
        if d < self.best_dist - 0.05:
            self.best_dist, self.last_progress = d, now
        elif (now - self.last_progress).nanoseconds * 1e-9 > self.progress_timeout:
            self.get_logger().warn('no progress towards this frontier, skipping it')
            self.blacklist.append(self.goal_xy)
            self.cancel_goal()
            return

        # Is there still a frontier at the goal? If the lidar already mapped it, move on.
        cells = [f.cells for f in frontiers]
        if cells:
            nearest = np.min(np.hypot(*(np.vstack(cells) - self.goal_xy).T))
        else:
            nearest = math.inf
        if nearest > self.frontier_lost_radius:
            self.get_logger().info('frontier already explored, picking the next one')
            self.cancel_goal()

    def is_blacklisted(self, x, y):
        return any(math.dist((x, y), b) < self.blacklist_radius for b in self.blacklist)

    def finish(self):
        self.get_logger().info(f'exploration complete after {self.goals_sent} goals')
        self.save_map()
        robot = self.robot_xy()
        if self.return_to_start and self.start_xy is not None and robot is not None:
            self.get_logger().info('driving back to the start')
            self.send_goal(*self.start_xy, robot, RETURNING)
        else:
            self.state = DONE

    # ------------------------------------------------------------------ Nav2
    def send_goal(self, x, y, robot, state):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        yaw = math.atan2(y - robot[1], x - robot[0])    # face the way we're driving
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        self.goal_seq += 1
        seq = self.goal_seq
        self.state = state
        self.goal_xy = (x, y)
        self.goal_handle = None
        self.canceling = False
        self.best_dist = math.inf
        self.last_progress = self.get_clock().now()
        self.goals_sent += 1
        self.nav.send_goal_async(goal).add_done_callback(lambda f: self.on_goal_response(f, seq))

    def on_goal_response(self, future, seq):
        handle = future.result()
        if seq != self.goal_seq:
            return
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal')
            self.blacklist.append(self.goal_xy)
            self.state = IDLE
            return
        self.goal_handle = handle
        if self.canceling:                      # cancel was requested before acceptance
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, seq))

    def on_result(self, future, seq):
        if seq != self.goal_seq:
            return
        status = future.result().status
        if self.state == RETURNING:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('back at the start. Done!')
            else:
                self.get_logger().warn('could not drive back to the start, stopping here. Done!')
            self.state = DONE
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('reached frontier')
            self.blacklist.append(self.goal_xy)   # been there; don't come back
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Nav2 could not reach the frontier, skipping it')
            self.blacklist.append(self.goal_xy)
        self.state = IDLE

    def cancel_goal(self):
        self.canceling = True
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    # ------------------------------------------------------------------ extras
    def save_map(self):
        if not self.save_map_path:
            return
        try:
            from slam_toolbox.srv import SaveMap
            from std_msgs.msg import String
        except ImportError:
            self.get_logger().warn('slam_toolbox not installed, cannot save the map')
            return
        client = self.create_client(SaveMap, 'slam_toolbox/save_map')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('slam_toolbox/save_map service not available')
            return
        client.call_async(SaveMap.Request(name=String(data=self.save_map_path)))
        self.get_logger().info(f'saving map to {self.save_map_path}.yaml/.pgm')

    def publish_markers(self, frontiers):
        stamp = self.get_clock().now().to_msg()

        def marker(mid, mtype, scale, rgba):
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = stamp
            m.ns = 'explore'
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
            m.lifetime = Duration(seconds=5.0).to_msg()
            return m

        cells = marker(0, Marker.POINTS, 0.03, (0.0, 0.8, 1.0, 1.0))
        for f in frontiers:
            cells.points.extend(Point(x=float(x), y=float(y), z=0.02) for x, y in f.cells[:2000])

        candidates = marker(1, Marker.SPHERE_LIST, 0.08, (1.0, 0.6, 0.0, 1.0))
        candidates.points = [Point(x=f.x, y=f.y, z=0.05) for f in frontiers]

        tried = marker(2, Marker.SPHERE_LIST, 0.06, (1.0, 0.1, 0.1, 0.8))
        tried.points = [Point(x=x, y=y, z=0.05) for x, y in self.blacklist]

        goal = marker(3, Marker.SPHERE, 0.12, (0.1, 1.0, 0.2, 1.0))
        if self.goal_xy is not None and self.state in (NAVIGATING, RETURNING):
            goal.pose.position.x, goal.pose.position.y = self.goal_xy
        else:
            goal.action = Marker.DELETE

        self.marker_pub.publish(MarkerArray(markers=[cells, candidates, tried, goal]))


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
