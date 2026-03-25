import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float64MultiArray
import serial
import struct
import threading


class RadioBridgeNode(Node):

    def __init__(self):
        super().__init__('radio_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        self.serial_conn = serial.Serial(
            port=serial_port,
            baudrate=baud_rate,
            timeout=0.1
        )
        self.get_logger().info(f'Serial connection opened: {serial_port}')

        self.create_subscription(Twist, '/cmd_vel', self.callback_cmd_vel, 10)
        self.create_subscription(
            Float64MultiArray, '/drive/pid_tuning', self.callback_pid_tuning, 10
        )
        self.create_subscription(Float32, '/weapon/speed', self.callback_weapon_speed, 10)

        threading.Thread(target=self.serial_rx_thread, daemon=True).start()

    def callback_cmd_vel(self, msg: Twist):
        data = struct.pack('<ff', msg.linear.x, msg.angular.z)
        self.get_logger().info(f'Sending cmd_vel: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')
        self.serial_conn.write(data)

    def callback_pid_tuning(self, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            data = struct.pack('<ffff', msg.data[0], msg.data[1], msg.data[2], msg.data[3])
            self.serial_conn.write(data)

    def callback_weapon_speed(self, msg: Float32):
        data = struct.pack('<f', msg.data)
        self.get_logger().info(f'Sending weapon speed: {msg.data:.2f}')
        self.serial_conn.write(data)

    def serial_rx_thread(self):
        while rclpy.ok():
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    self.get_logger().info(f'Received: {data.hex()}')
            except Exception as e:
                self.get_logger().warn(f'RX error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RadioBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()