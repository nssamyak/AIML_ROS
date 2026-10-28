"""Drive the simulated robot and compare wheel odometry with Gazebo ground truth.
Needs: sim running, and /gt_poses bridged from /world/arena/pose/info.
"""
import math
import time

import rclpy
import rclpy.parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Tester(Node):
    def __init__(self):
        super().__init__('drive_test', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.odom = None
        self.gt = None
        self.gt_index = None   # the bridge drops entity names, so find the robot by its spawn point
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(TFMessage, 'gt_poses', self.on_gt, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def on_odom(self, m):
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, yaw(p.orientation))

    def on_gt(self, m):
        if self.gt_index is None:
            for i, t in enumerate(m.transforms):
                if math.hypot(t.transform.translation.x + 1.3, t.transform.translation.y + 0.2) < 0.05:
                    self.gt_index = i
                    break
            else:
                return
        tr = m.transforms[self.gt_index].transform
        self.gt = (tr.translation.x, tr.translation.y, yaw(tr.rotation))

    def spin_for(self, seconds, twist=None):
        # durations are in SIM time, so a slow simulator doesn't skew the results
        while self.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(self, timeout_sec=0.05)
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        wall = time.time()
        while self.get_clock().now().nanoseconds < end:
            if twist is not None:
                self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.rtf = seconds / max(time.time() - wall, 1e-3)


def main():
    rclpy.init()
    n = Tester()
    n.spin_for(3.0)
    if n.odom is None or n.gt is None:
        print(f'FAIL: no data (odom={n.odom}, gt={n.gt})')
        return
    tests = [('forward  vx=+0.1', 0.1, 0.0, 0.0), ('strafe   vy=+0.1', 0.0, 0.1, 0.0),
             ('backward vx=-0.1', -0.1, 0.0, 0.0), ('rotate   wz=+0.5', 0.0, 0.0, 0.5),
             ('diagonal 0.07,0.07', 0.07, 0.07, 0.0)]
    for label, vx, vy, wz in tests:
        o0, g0 = n.odom, n.gt
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
        n.spin_for(3.0, t)
        n.spin_for(1.5, Twist())
        o1, g1 = n.odom, n.gt
        d = lambda a, b: (b[0] - a[0], b[1] - a[1], math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2])))
        do, dg = d(o0, o1), d(g0, g1)
        # express ground-truth motion in the robot frame at the start of the move
        c, s = math.cos(g0[2]), math.sin(g0[2])
        dg_body = (c * dg[0] + s * dg[1], -s * dg[0] + c * dg[1], dg[2])
        co, so = math.cos(o0[2]), math.sin(o0[2])
        do_body = (co * do[0] + so * do[1], -so * do[0] + co * do[1], do[2])
        print(f'{label}: requested ({3*vx:+.2f}, {3*vy:+.2f}, {3*wz:+.2f})  '
              f'truth ({dg_body[0]:+.3f}, {dg_body[1]:+.3f}, {dg_body[2]:+.3f})  '
              f'odom ({do_body[0]:+.3f}, {do_body[1]:+.3f}, {do_body[2]:+.3f})  RTF {n.rtf:.2f}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
