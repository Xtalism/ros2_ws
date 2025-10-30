#!/usr/bin/env python3
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from sensor_msgs.msg import Image


class EdgeDetection(Node):
    def __init__(self) -> None:
        super().__init__("edge_detection")

        self.image_sub: Subscription = self.create_subscription(
            Image, "image/gray", self.image_callback, 10
        )
        self.canny_publisher: Publisher = self.create_publisher(
            Image, "image/edge_detection", 10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg: Image) -> None:
        self.image = msg
        cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        canny = cv2.Canny(cv_image, 100, 200)

        try:
            # self.get_logger().info(
            #     f"\nReceving from: [{self.image_sub.topic}], making canny image at: [{self.canny_publisher.topic}]"
            # )

            canny: Image = self.bridge.cv2_to_imgmsg(canny, encoding="mono8")
            canny.header.frame_id = "frame"

            self.canny_publisher.publish(canny)
        except Exception as e:
            self.get_logger().error(f"Error publishing image: [{e}]")


def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
