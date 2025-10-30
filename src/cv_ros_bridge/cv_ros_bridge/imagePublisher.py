#!/usr/bin/env python3
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImagePublisher(Node):
    def __init__(self, image_path: str):
        super().__init__("image_publisher")
        self.image_freq = 10
        self.image_pub = self.create_publisher(Image, "image/raw", self.image_freq)
        self.image_timer = self.create_timer(1 / self.image_freq, self.image_callback)

        self.bridge = CvBridge()
        self.cv_image = cv2.imread(image_path, cv2.IMREAD_COLOR)

        self.image_path = image_path
        self.image = self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8")
        self.image.header.stamp = self.get_clock().now().to_msg()
        self.image.header.frame_id = "frame_id"

    def image_callback(self):
        if self.cv_image is not None:
            try:
                self.image = self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8")
                self.image.header.stamp = self.get_clock().now().to_msg()
                self.image.header.frame_id = "frame_id"
                self.image_pub.publish(self.image)
                self.get_logger().info(f"Publishing: {self.image_path}")
            except Exception as e:
                self.get_logger().error(f"Error publishing the image: [{e}]")
        else:
            self.get_logger().fatal(f"There is an error at {self.image_path}")


def main(args=None):
    rclpy.init(args=args)

    # image_publisher = ImagePublisher("/home/xtal/ros2_ws/bedroom.jpg")
    image_publisher = ImagePublisher("/home/xtal/ros2_ws/pictures/bacteria.png")
    rclpy.spin(image_publisher)


if __name__ == "__main__":
    main()
