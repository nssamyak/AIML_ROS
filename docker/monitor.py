"""Log ground-truth pose, odom and cmd_vel once per sim second (for debugging Nav2 runs)."""
import math

import rclpy
import rclpy.parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Monitor(Node):
    def __init__(self):
        super().__init__('monitor', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.gt = self.odom = None
        self.cmd = (0, 0, 0)
        self.cmd_n = 0
        self.idx = None
        self.create_subscription(TFMessage, 'gt_poses', self.on_gt, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_timer(1.0, self.report)
        print('t, gt_x, gt_y, gt_yaw, gt_z, roll, pitch, odom_x, odom_y, cmd_vx, cmd_vy, cmd_wz, cmd_msgs', flush=True)

    def on_gt(self, m):
        if self.idx is None:
            for i, t in enumerate(m.transforms):
                tr = t.transform.translation
                if abs(tr.x) + abs(tr.y) > 1e-6:   # first entity not at the origin = the robot
                    self.idx = i
                    break
            else:
                return
        t = m.transforms[self.idx].transform
        q = t.rotation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
        self.gt = (t.translation.x, t.translation.y, yaw(q), t.translation.z, roll, pitch)

    def on_odom(self, m):
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y)

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)
        self.cmd_n += 1

    def report(self):
        if self.gt is None or self.odom is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        g = self.gt
        print(f'{t:.0f}, {g[0]:.2f}, {g[1]:.2f}, {g[2]:.2f}, {g[3]:.3f}, {g[4]:.2f}, {g[5]:.2f}, '
              f'{self.odom[0]:.2f}, {self.odom[1]:.2f}, '
              f'{self.cmd[0]:.3f}, {self.cmd[1]:.3f}, {self.cmd[2]:.3f}, {self.cmd_n}', flush=True)
        self.cmd_n = 0


def main():
    rclpy.init()
    rclpy.spin(Monitor())


if __name__ == '__main__':
    main()
