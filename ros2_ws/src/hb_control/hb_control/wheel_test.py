"""wheel_test: spin each wheel on its own, then run a few robot motions.

Use it to check the wiring before letting the robot drive itself:

    ros2 run hb_control wheel_test            # one wheel at a time
    ros2 run hb_control wheel_test --motions  # forward, sideways, rotate

With a POSITIVE speed, each wheel should turn CLOCKWISE when you look at it
from outside the robot, and its position in /joint_states should increase.
If not, flip MOTOR_DIR / ENC_DIR for that wheel in the firmware config.h.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

NAMES = ['left (front-left)', 'right (front-right)', 'back']


def main():
    rclpy.init()
    node = Node('wheel_test')
    node.declare_parameter('speed', 3.0)          # rad/s for single-wheel test
    speed = node.get_parameter('speed').value
    wheels = node.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)
    cmd_vel = node.create_publisher(Twist, 'cmd_vel', 10)

    def hold(publish, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            publish()
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        time.sleep(1.0)  # let the publishers connect
        if '--motions' in sys.argv:
            motions = [('forward', 0.1, 0.0, 0.0), ('backward', -0.1, 0.0, 0.0),
                       ('left (strafe)', 0.0, 0.1, 0.0), ('right (strafe)', 0.0, -0.1, 0.0),
                       ('rotate CCW', 0.0, 0.0, math.radians(60)),
                       ('rotate CW', 0.0, 0.0, -math.radians(60))]
            for label, vx, vy, wz in motions:
                node.get_logger().info(f'robot: {label}')
                t = Twist()
                t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
                hold(lambda: cmd_vel.publish(t), 2.0)
                hold(lambda: cmd_vel.publish(Twist()), 1.0)
        else:
            for i, name in enumerate(NAMES):
                node.get_logger().info(f'wheel {i} {name}: +{speed} rad/s (should turn clockwise seen from outside)')
                cmd = [0.0, 0.0, 0.0]
                cmd[i] = speed
                hold(lambda: wheels.publish(Float64MultiArray(data=cmd)), 2.0)
                hold(lambda: wheels.publish(Float64MultiArray(data=[0.0, 0.0, 0.0])), 1.0)
        node.get_logger().info('done')
    except KeyboardInterrupt:
        pass
    finally:
        wheels.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
