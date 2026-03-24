#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageFormatConverter(Node):
    def __init__(self):
        super().__init__('image_format_converter')
        
        self.declare_parameter('input_topic', 'image')
        self.declare_parameter('output_topic', 'image_rgb')
        self.declare_parameter('output_encoding', 'rgb8')
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.output_encoding = self.get_parameter('output_encoding').value
        
        self.bridge = CvBridge()
        
        self.pub = self.create_publisher(Image, output_topic, 10)
        self.sub = self.create_subscription(
            Image,
            input_topic,
            self.callback,
            10
        )
        
        self.get_logger().info(f'Converting {input_topic} -> {output_topic} ({self.output_encoding})')

    def callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            if msg.encoding in ['yuv422_yuy2', 'YUYV']:
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_YUV2BGR_YUY2)
                if self.output_encoding == 'rgb8':
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding=self.output_encoding)
            out_msg.header = msg.header
            self.pub.publish(out_msg)
            
        except Exception as e:
            self.get_logger().error(f'Conversion error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageFormatConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
