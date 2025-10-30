#!/usr/bin/env python3
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from sensor_msgs.msg import Image


class ImageErotion(Node):
    def __init__(self) -> None:
        super().__init__("image_erotion")

        self.image_sub: Subscription = self.create_subscription(
            Image, "image/thresholded", self.image_callback, 10
        )
        self.erotion_publisher: Publisher = self.create_publisher(
            Image, "image/erotion", 10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg: Image) -> None:
        self.image = msg
        cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        erotion_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        erotion = cv2.erode(cv_image, erotion_kernel, iterations=4)

        try:
            # self.get_logger().info(
            #     f"\nReceving from: [{self.image_sub.topic}], making erotion image at: [{self.erotion_publisher.topic}]"
            # )

            erotion: Image = self.bridge.cv2_to_imgmsg(erotion, encoding="mono8")
            erotion.header.frame_id = "frame"

            self.erotion_publisher.publish(erotion)
        except Exception as e:
            self.get_logger().error(f"Error publishing image: [{e}]")


def main(args=None):
    rclpy.init(args=args)
    node = ImageErotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
