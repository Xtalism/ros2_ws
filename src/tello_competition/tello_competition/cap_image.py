import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class TelloImage(Node):

    def __init__(self):
        super().__init__('tello_image')
        self.subscription = self.create_subscription(
            Image,
            'camera',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.subscription
        self.frame = None

    def listener_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

def main(args=None):
    rclpy.init(args=args)
    tello_image = TelloImage()
    cv2.namedWindow("Camera Capture", cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
    frame_count = 1

    while rclpy.ok():
        rclpy.spin_once(tello_image, timeout_sec=0.1)
        if tello_image.frame is not None:
            cv2.imshow("Camera Capture", tello_image.frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                filename = f"/home/xtal/ros2_ws/pictures/calibration/dji_tello/{frame_count}.png"
                cv2.imwrite(filename, tello_image.frame)
                print(f"Frame saved as '{filename}'")
                frame_count += 1
            
    cv2.destroyAllWindows()
    tello_image.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
