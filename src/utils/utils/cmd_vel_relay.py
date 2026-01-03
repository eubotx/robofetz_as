import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.sub = self.create_subscription(Twist, 'cmd_vel_smoothed', self.cmd_vel_callback, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Relaying cmd_vel_smoothed -> cmd_vel')

    def cmd_vel_callback(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
