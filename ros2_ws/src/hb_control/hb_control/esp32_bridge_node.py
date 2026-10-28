"""esp32_bridge: talks to the ESP32 motor controller over USB serial.

It stands in for Gazebo's ros2_control, offering the same topics:
  subscribes  /wheel_velocity_controller/commands  Float64MultiArray  wheel targets, rad/s
  publishes   joint_states                         JointState         wheel position + speed

Serial protocol (text lines, 115200 baud):
  RPi -> ESP32   V,<w_left>,<w_right>,<w_back>      wheel speed targets, rad/s
  ESP32 -> RPi   S,<ms>,<p_l>,<p_r>,<p_b>,<v_l>,<v_r>,<v_b>
                                                    wheel angle (rad) and speed (rad/s)
                 #<text>                            log message from the firmware
"""
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class Esp32Bridge(Node):

    def __init__(self):
        super().__init__('esp32_bridge')
        p = self.declare_parameter
        self.port = p('port', '/dev/ttyUSB0').value
        self.baud = p('baud', 115200).value
        self.joints = list(p('wheel_joints',
                             ['rim_left_joint', 'rim_right_joint', 'rim_back_joint']).value)
        self.cmd_timeout = p('cmd_timeout', 0.5).value
        send_rate = p('send_rate', 20.0).value
        cmd_topic = p('command_topic', '/wheel_velocity_controller/commands').value

        self.ser = None
        self.rx_buffer = b''
        self.target = [0.0] * len(self.joints)
        self.last_cmd_time = None

        self.state_pub = self.create_publisher(JointState, 'joint_states', 20)
        self.create_subscription(Float64MultiArray, cmd_topic, self.on_command, 10)
        self.create_timer(1.0 / send_rate, self.send_target)
        self.create_timer(0.005, self.read_serial)
        self.create_timer(2.0, self.ensure_connected)
        self.ensure_connected()

    # ------------------------------------------------------------------ serial
    def ensure_connected(self):
        if self.ser is not None:
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.rx_buffer = b''
            self.get_logger().info(f'connected to ESP32 on {self.port}')
        except serial.SerialException as e:
            self.get_logger().warn(f'cannot open {self.port}: {e}', throttle_duration_sec=10.0)

    def drop_connection(self, err):
        self.get_logger().error(f'serial error, reconnecting: {err}')
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None

    def write_line(self, line):
        if self.ser is None:
            return
        try:
            self.ser.write((line + '\n').encode('ascii'))
        except (serial.SerialException, OSError) as e:
            self.drop_connection(e)

    # ------------------------------------------------------------------ commands
    def on_command(self, msg):
        if len(msg.data) != len(self.joints):
            self.get_logger().warn(
                f'expected {len(self.joints)} wheel speeds, got {len(msg.data)}',
                throttle_duration_sec=5.0)
            return
        self.target = list(msg.data)
        self.last_cmd_time = self.get_clock().now()
        self.send_target()

    def send_target(self):
        stale = (self.last_cmd_time is None or
                 (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout)
        target = [0.0] * len(self.joints) if stale else self.target
        self.write_line('V,' + ','.join(f'{w:.3f}' for w in target))

    # ------------------------------------------------------------------ feedback
    def read_serial(self):
        if self.ser is None:
            return
        try:
            waiting = self.ser.in_waiting
            if waiting:
                self.rx_buffer += self.ser.read(waiting)
        except (serial.SerialException, OSError) as e:
            self.drop_connection(e)
            return

        while b'\n' in self.rx_buffer:
            raw, self.rx_buffer = self.rx_buffer.split(b'\n', 1)
            line = raw.decode('ascii', errors='ignore').strip()
            if line.startswith('S,'):
                self.handle_state(line)
            elif line.startswith('#'):
                self.get_logger().info(f'[esp32] {line[1:].strip()}')
        if len(self.rx_buffer) > 4096:
            self.rx_buffer = b''  # garbage on the line, start over

    def handle_state(self, line):
        n = len(self.joints)
        parts = line.split(',')
        if len(parts) != 2 + 2 * n:
            return
        try:
            values = [float(v) for v in parts[2:]]
        except ValueError:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joints
        msg.position = values[:n]
        msg.velocity = values[n:]
        self.state_pub.publish(msg)

    def destroy_node(self):
        # Leave the robot stopped
        self.write_line('V,' + ','.join(['0'] * len(self.joints)))
        if self.ser is not None:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
