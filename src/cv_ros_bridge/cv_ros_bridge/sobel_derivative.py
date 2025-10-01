#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from numpy.typing import NDArray
from rclpy.node import Node
from sensor_msgs.msg import Image


class SobelDerivative(Node):
    def __init__(self):
        super().__init__("sobel_derivative")

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, "image/gray", self.image_callback, 10
        )
        self.sobel_pub = self.create_publisher(Image, "image/sobel", 10)

        self.cv_image: NDArray[np.uint8] = np.zeros((100, 100, 3), np.uint8)

    def image_callback(self, msg: Image) -> None:
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

            sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=9)
            sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=9)

            sobel = cv2.magnitude(sobelx, sobely)
            sobel = np.uint8(cv2.normalize(sobel, None, 0, 255, cv2.NORM_MINMAX))

            sobel_ros = self.bridge.cv2_to_imgmsg(sobel, encoding="mono8")
            sobel_ros.header.stamp = self.get_clock().now().to_msg()
            sobel_ros.header.frame_id = "frame"
            self.sobel_pub.publish(sobel_ros)

        except Exception as e:
            self.get_logger().error(f"Error at Sobel filter: {e}")


def main(args=None):
    rclpy.init(args=args)
    sobel_node = SobelDerivative()
    rclpy.spin(sobel_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
