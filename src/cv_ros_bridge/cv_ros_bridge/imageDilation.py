#!/usr/bin/env python3
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from sensor_msgs.msg import Image


class ImageDilation(Node):
    def __init__(self) -> None:
        super().__init__("image_erotion")

        self.image_sub: Subscription = self.create_subscription(
            Image, "image/thresholded", self.image_callback, 10
        )
        self.dilation_publisher: Publisher = self.create_publisher(
            Image, "image/dilation", 10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg: Image) -> None:
        self.image = msg
        cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        dilation_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        dilation = cv2.dilate(cv_image, dilation_kernel, iterations=4)

        try:
            # self.get_logger().info(
            #     f"\nReceving from: [{self.image_sub.topic}], making dilation image at: [{self.dilation_publisher.topic}]"
            # )

            dilation: Image = self.bridge.cv2_to_imgmsg(dilation, encoding="mono8")
            dilation.header.frame_id = "frame"

            self.dilation_publisher.publish(dilation)
        except Exception as e:
            self.get_logger().error(f"Error publishing image: [{e}]")


def main(args=None):
    rclpy.init(args=args)
    node = ImageDilation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
