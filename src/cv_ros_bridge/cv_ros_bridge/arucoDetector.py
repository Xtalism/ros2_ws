#!/usr/bin/env python3
import cv2
import numpy as np
import yaml
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster

from cv_ros_bridge.arucoPub import (
    publish_visualization, publish_pose, publish_tf, publish_camera_info
)

class ArucoImage(Node):
    def __init__(self):
        super().__init__('ArucoImage')
        self.bridge = CvBridge()
        self.image_raw = self.create_publisher(Image, 'camera/image_raw', 10)
        self.image_aruco = self.create_publisher(Image, 'camera/image_aruco', 10)
        self.marker_pose_pub = self.create_publisher(PoseStamped, 'marker/pose', 10)
        self.camera_pose_pub = self.create_publisher(PoseStamped, 'camera/pose', 10)
        self.marker_viz = self.create_publisher(Marker, 'marker/visualization', 10)
        self.camera_viz = self.create_publisher(Marker, 'camera/visualization', 10)
        self.cam_info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.aruco_size = cv2.aruco.DICT_4X4_100
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_size)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(
            dictionary=self.aruco_dict, detectorParams=self.detector_params
        )
        self.cap = cv2.VideoCapture(index=6)

        with open("camera_calibration/realsense_d435.yaml", "r") as f:
            params = yaml.safe_load(f)
        self.K = np.array(params["K"], dtype=np.float64)
        self.D = np.array(params["D"], dtype=np.float64).ravel()
        self.marker_length = 0.1

        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

    def timer_callback(self):
        clock = self.get_clock()
        ok, frame = self.cap.read()
                    
        if not ok:
            return

        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.K, self.D
            )
            t = tvecs[0][0]
            rvec = rvecs[0][0]
            R, _ = cv2.Rodrigues(rvec)

            publish_pose(R, t, self.marker_pose_pub, clock, frame_id="camera_link")
            publish_tf(R, t, self.tf_broadcaster, clock, parent_frame="camera_link", child_frame="aruco_marker")
            publish_visualization(R, t, self.marker_viz, clock, frame_id="camera_link", is_marker=True)

            R_inv = R.T
            t_inv = -R_inv @ t
            publish_pose(R_inv, t_inv, self.camera_pose_pub, clock, frame_id="aruco_marker")
            publish_tf(R_inv, t_inv, self.tf_broadcaster, clock, parent_frame="aruco_marker", child_frame="camera_link")
            publish_visualization(R_inv, t_inv, self.camera_viz, clock, frame_id="aruco_marker", is_marker=False)

        msg_raw = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg_raw.header.frame_id = "camera_link"
        msg_raw.header.stamp = self.get_clock().now().to_msg()
        self.image_raw.publish(msg_raw)

        detected_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids) if ids is not None else frame
        
        msg_aruco = self.bridge.cv2_to_imgmsg(detected_markers, encoding="bgr8")
        msg_aruco.header.frame_id = "camera_link"
        msg_aruco.header.stamp = self.get_clock().now().to_msg()
        self.image_aruco.publish(msg_aruco)

        publish_camera_info(self.K, self.D, w, h, self.cam_info_pub, self.get_clock(), frame_id="camera_link")

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoImage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()