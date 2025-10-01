#!/usr/bin/env python3
import numpy as np
import rclpy
from cv_bridge import CvBridge
from numpy.typing import NDArray
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImagePublisher(Node):
    def __init__(self):
        super().__init__("opencv_image_bridge")
        self.publisher_ = self.create_publisher(Image, "image_cv_bridge", 10)
        self.bridge = CvBridge()
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.channels = tuple(np.random.randint(0, 256) for _ in range(3))
        self.img = self._generate_image(size=(300, 300, 3), color=self.channels)

    def _generate_image(
        self, size: tuple[int, int, int], color: tuple[int, int, int]
    ) -> NDArray[np.uint8]:
        rows, cols, channels = size
        img: NDArray[np.uint8] = np.full((rows, cols, channels), color, dtype=np.uint8)
        return img

    def timer_callback(self):
        msg = self.bridge.cv2_to_imgmsg(self.img, encoding="rgb8")
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing Image")


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
