#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class BgrToRgbNode(Node):
    def __init__(self):
        super().__init__('bgr_to_rgb_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            Image,
            '/image_rgb',
            10
        )

        self.get_logger().info(
            'BGR-to-RGB converter running: /image -> /image_rgb'
        )

    def image_callback(self, msg):
        # Convert the incoming ROS image into OpenCV BGR format.
        bgr_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        # Convert BGR pixels to RGB pixels.
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)

        # Convert the result back into a ROS image message.
        output_msg = self.bridge.cv2_to_imgmsg(
            rgb_image,
            encoding='rgb8'
        )

        output_msg.header = msg.header
        self.publisher.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BgrToRgbNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
