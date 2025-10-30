#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from sensor_msgs.msg import Image


class LineDetection(Node):
    def __init__(self) -> None:
        super().__init__("line_detection")

        self.image_sub: Subscription = self.create_subscription(
            Image, "image/edge_detection", self.image_callback, 10
        )
        self.hough_publisher: Publisher = self.create_publisher(
            Image, "image/line_detection", 10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg: Image) -> None:
        self.image = msg
        cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        lines = cv2.HoughLines(cv_image[:, :, 0], 1, np.pi / 180, 160)

        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))

                cv2.line(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 1, cv2.LINE_AA)

        try:
            hough_transform: Image = self.bridge.cv2_to_imgmsg(
                cv_image, encoding="bgr8"
            )
            hough_transform.header.frame_id = "frame"

            self.hough_publisher.publish(hough_transform)
        except Exception as e:
            self.get_logger().error(f"Error publishing image: [{e}]")


def main(args=None):
    rclpy.init(args=args)
    node = LineDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
