"""kiwi_drive: cmd_vel -> wheel speeds, and wheel feedback -> odometry.

Subscribes
  cmd_vel          geometry_msgs/Twist (or TwistStamped)   robot velocity request
  joint_states     sensor_msgs/JointState                  wheel positions/speeds
Publishes
  /wheel_velocity_controller/commands  std_msgs/Float64MultiArray  [left, right, back] rad/s
  odom             nav_msgs/Odometry
  TF               odom -> base_footprint

The node is identical in Gazebo and on the real robot. In Gazebo the wheel
topics are served by ros2_control; on the robot by esp32_bridge.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

from hb_control.kinematics import KiwiKinematics, integrate_pose, limit_wheel_speeds


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class KiwiDriveNode(Node):

    def __init__(self):
        super().__init__('kiwi_drive')

        p = self.declare_parameter
        wheel_radius = p('wheel_radius', 0.019).value
        base_radius = p('wheel_base_radius', 0.052).value
        angles_deg = p('wheel_angles_deg', [60.0, -60.0, 180.0]).value
        self.wheel_joints = list(p('wheel_joints',
                                   ['rim_left_joint', 'rim_right_joint', 'rim_back_joint']).value)
        self.max_wheel_speed = p('max_wheel_speed', 30.0).value
        self.cmd_timeout = p('cmd_timeout', 0.5).value
        rate = p('command_rate', 50.0).value
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.publish_tf = p('publish_tf', True).value
        wheel_cmd_topic = p('wheel_command_topic', '/wheel_velocity_controller/commands').value
        stamped = p('use_stamped_cmd_vel', False).value

        if len(angles_deg) != len(self.wheel_joints):
            raise ValueError('wheel_angles_deg and wheel_joints must have the same length')

        self.kin = KiwiKinematics(wheel_radius, base_radius, [math.radians(a) for a in angles_deg])

        # Command side
        self.cmd = (0.0, 0.0, 0.0)
        self.last_cmd_time = None
        self.stop_messages_left = 0
        self.wheel_pub = self.create_publisher(Float64MultiArray, wheel_cmd_topic, 10)
        if stamped:
            self.create_subscription(TwistStamped, 'cmd_vel', lambda m: self.on_cmd(m.twist), 10)
        else:
            self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_timer(1.0 / rate, self.send_wheel_command)

        # Odometry side
        self.x = self.y = self.theta = 0.0
        self.last_positions = None
        self.last_stamp = None
        self.joint_index = None
        self._names_len = 0
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 20)

        self.get_logger().info(
            f'kiwi drive ready: r={wheel_radius} m, L={base_radius} m, wheels at {angles_deg} deg')

    # ------------------------------------------------------------------ commands
    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def send_wheel_command(self):
        fresh = (self.last_cmd_time is not None and
                 (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 < self.cmd_timeout)
        if fresh:
            wheels = limit_wheel_speeds(self.kin.body_to_wheels(*self.cmd), self.max_wheel_speed)
            self.stop_messages_left = 3
        elif self.stop_messages_left > 0:
            # Command went stale: send a few zeros, then go quiet so other
            # tools (e.g. wheel_test) can drive the wheels directly.
            wheels = [0.0] * len(self.wheel_joints)
            self.stop_messages_left -= 1
        else:
            return
        self.wheel_pub.publish(Float64MultiArray(data=[float(w) for w in wheels]))

    # ------------------------------------------------------------------ odometry
    def on_joint_states(self, msg):
        if self.joint_index is None or len(msg.name) != self._names_len:
            try:
                self.joint_index = [msg.name.index(j) for j in self.wheel_joints]
                self._names_len = len(msg.name)
            except ValueError:
                return  # this message doesn't contain our wheels
        idx = self.joint_index
        if len(msg.position) <= max(idx):
            return

        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()
        positions = [msg.position[i] for i in idx]

        if self.last_positions is None:
            self.last_positions, self.last_stamp = positions, stamp
            return

        dt = (stamp - self.last_stamp).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        deltas = [p - q for p, q in zip(positions, self.last_positions)]
        self.last_positions, self.last_stamp = positions, stamp
        if dt > 1.0:
            return  # long gap (e.g. reconnect): don't integrate a huge jump

        # Wheel angle changes -> body displacement -> world pose
        dx, dy, dth = self.kin.wheels_to_body(deltas)
        self.x, self.y, self.theta = integrate_pose(self.x, self.y, self.theta, dx, dy, dth)

        if len(msg.velocity) > max(idx):
            vx, vy, wz = self.kin.wheels_to_body([msg.velocity[i] for i in idx])
        else:
            vx, vy, wz = dx / dt, dy / dt, dth / dt

        self.publish_odometry(stamp.to_msg(), vx, vy, wz)

    def publish_odometry(self, stamp, vx, vy, wz):
        qx, qy, qz, qw = yaw_to_quaternion(self.theta)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        # x, y, z, roll, pitch, yaw. Omni wheels slip, so don't trust odom too much.
        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = odom.pose.covariance[21] = odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = odom.twist.covariance[7] = 0.01
        odom.twist.covariance[14] = odom.twist.covariance[21] = odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 0.05
        self.odom_pub.publish(odom)

        if self.tf_pub is not None:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_pub.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = KiwiDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
