#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CameraCalibration(Node):
    def __init__(self):
        super().__init__('camera_calibration')
        self.subscription = self.create_subscription(
            Image,
            '/kinect2/hd/image_color',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.subscription
        self.frame = None

    def listener_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def main(args=None):
    rclpy.init(args=args)
    camera_calibration = CameraCalibration()
    cv2.namedWindow("Camera Capture", cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
    frame_count = 1

    while rclpy.ok():
        rclpy.spin_once(camera_calibration, timeout_sec=0.1)
        if camera_calibration.frame is not None:
            cv2.imshow("Camera Capture", camera_calibration.frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                filename = f"/home/xtal/ros2_ws/pictures/calibration/kinect2/{frame_count}.png"
                cv2.imwrite(filename, camera_calibration.frame)
                print(f"Frame saved as '{filename}'")
                frame_count += 1
            
    cv2.destroyAllWindows()
    camera_calibration.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()