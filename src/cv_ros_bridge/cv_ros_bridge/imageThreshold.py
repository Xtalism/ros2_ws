#!/usr/bin/env python3
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from sensor_msgs.msg import Image


class ImageThreshold(Node):
    def __init__(self) -> None:
        super().__init__("image_threshold")

        self.image_sub: Subscription = self.create_subscription(
            Image, "image/ch2", self.image_callback, 10
        )
        self.threshold_publisher: Publisher = self.create_publisher(
            Image, "image/thresholded", 10
        )
        self.threshold_inv_publisher: Publisher = self.create_publisher(
            Image, "image/thresholded/inv", 10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg: Image) -> None:
        self.image = msg
        cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        _, thresholded = cv2.threshold(cv_image, 153, 255, cv2.THRESH_BINARY)
        _, thresholded_inv = cv2.threshold(cv_image, 153, 255, cv2.THRESH_BINARY_INV)

        try:
            # self.get_logger().info(
            #     f"\nReceving from: [{self.image_sub.topic}], threshold image at: [{self.threshold_publisher.topic}]"
            # )

            grayscale_threshold: Image = self.bridge.cv2_to_imgmsg(
                thresholded, encoding="mono8"
            )
            grayscale_threshold.header.frame_id = "frame"

            grayscale_threshold_inv: Image = self.bridge.cv2_to_imgmsg(
                thresholded_inv, encoding="mono8"
            )
            grayscale_threshold_inv.header.frame_id = "frame"

            self.threshold_publisher.publish(grayscale_threshold)
            self.threshold_inv_publisher.publish(grayscale_threshold_inv)
        except Exception as e:
            self.get_logger().error(f"Error publishing image: [{e}]")


def main(args=None):
    rclpy.init(args=args)
    node = ImageThreshold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
