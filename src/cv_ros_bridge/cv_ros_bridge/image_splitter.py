#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from numpy.typing import NDArray
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from sensor_msgs.msg import Image


class SplitChannels(Node):
    def __init__(self) -> None:
        super().__init__("split_channels")

        self.image: Image = Image()
        self.cv_image: NDArray[np.uint8] = np.zeros((100, 100, 3), np.uint8)
        self.ch0: NDArray[np.uint8] = np.zeros((100, 100, 1), np.uint8)
        self.ch1: NDArray[np.uint8] = np.zeros((100, 100, 1), np.uint8)
        self.ch2: NDArray[np.uint8] = np.zeros((100, 100, 1), np.uint8)
        self.grayscale: NDArray[np.uint8] = np.zeros((100, 100, 1), np.uint8)

        self.bridge: CvBridge = CvBridge()

        self.image_sub: Subscription = self.create_subscription(
            Image, "image/raw", self.image_callback, 10
        )
        self.grayscale_pub: Publisher = self.create_publisher(Image, "image/gray", 10)
        self.ch0_pub: Publisher = self.create_publisher(Image, "image/ch0", 10)
        self.ch1_pub: Publisher = self.create_publisher(Image, "image/ch1", 10)
        self.ch2_pub: Publisher = self.create_publisher(Image, "image/ch2", 10)

    def image_callback(self, msg: Image) -> None:
        self.image = msg
        self.cv_image = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
        self.grayscale = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        self.ch0, self.ch1, self.ch2 = cv2.split(self.cv_image)

        try:
            # self.get_logger().info(
            #     f"\nReceving from: [{self.image_sub.topic}], splitting channels and publishing at: [{self.grayscale_pub.topic}]"
            # )

            grayscale_ros: Image = self.bridge.cv2_to_imgmsg(self.grayscale, "mono8")
            grayscale_ros.header.frame_id = "frame"

            ch0_ros: Image = self.bridge.cv2_to_imgmsg(self.ch0, "mono8")
            ch0_ros.header.frame_id = "frame"

            ch1_ros: Image = self.bridge.cv2_to_imgmsg(self.ch1, "mono8")
            ch1_ros.header.frame_id = "frame"

            ch2_ros: Image = self.bridge.cv2_to_imgmsg(self.ch2, "mono8")
            ch2_ros.header.frame_id = "frame"

            self.grayscale_pub.publish(grayscale_ros)
            self.ch0_pub.publish(ch0_ros)
            self.ch1_pub.publish(ch1_ros)
            self.ch2_pub.publish(ch2_ros)
        except Exception as e:
            self.get_logger().error(f"Error publishing image: [{e}]")


def main(args=None):
    rclpy.init(args=args)
    splitChannels = SplitChannels()
    rclpy.spin(splitChannels)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
