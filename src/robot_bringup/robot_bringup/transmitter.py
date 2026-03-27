import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float64MultiArray
import serial
import threading

class RadioBridgeNode(Node):
    def __init__(self):
        super().__init__('radio_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        self.serial_conn = serial.Serial(port=serial_port, baudrate=baud_rate, timeout=0.1)
        self.get_logger().info(f'Serial connection opened: {serial_port}')

        self.create_subscription(Twist, '/cmd_vel', self.callback_cmd_vel, 10)
        self.create_subscription(Float64MultiArray, '/drive/pid_tuning', self.callback_pid_tuning, 10)
        self.create_subscription(Float32, '/weapon/speed', self.callback_weapon_speed, 10)

        threading.Thread(target=self.serial_rx_thread, daemon=True).start()

    def callback_cmd_vel(self, msg: Twist):
        # Format: CMD:linear,angular
        data = f"CMD:{msg.linear.x:.2f},{msg.angular.z:.2f}\n"
        self.serial_conn.write(data.encode('utf-8'))
        self.get_logger().info(f'Sending: {data.strip()}')

    def callback_pid_tuning(self, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            # Format: PID:p,i,d,f
            data = f"PID:{msg.data[0]:.2f},{msg.data[1]:.2f},{msg.data[2]:.2f},{msg.data[3]:.2f}\n"
            self.serial_conn.write(data.encode('utf-8'))

    def callback_weapon_speed(self, msg: Float32):
        # Format: WPN:speed
        data = f"WPN:{msg.data:.2f}\n"
        self.serial_conn.write(data.encode('utf-8'))
        self.get_logger().info(f'Sending: {data.strip()}')

    def serial_rx_thread(self):
        while rclpy.ok():
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.readline().decode('utf-8', errors='replace')
                    self.get_logger().info(f'Received from ESP32: {data.strip()}')
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