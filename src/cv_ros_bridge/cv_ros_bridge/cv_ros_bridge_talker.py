#!/usr/bin/env python3
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImagePublisher(Node):
    def __init__(self, image_path: str):
        super().__init__("opencv_image_talker")
        self.publisher_ = self.create_publisher(Image, "image_cv_bridge", 10)
        self.bridge = CvBridge()
        self.image_path = image_path
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        img_bgr = cv2.imread(self.image_path)
        if img_bgr is None:
            self.get_logger().error(f"Image file not found at {self.image_path}")
            return
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        msg = self.bridge.cv2_to_imgmsg(img_rgb, encoding="rgb8")
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published image from {self.image_path}")


def main(args=None):
    rclpy.init(args=args)
    image_path = os.path.expanduser("~/ros2_ws/pictures/kanye.jpg")
    node = ImagePublisher(image_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
