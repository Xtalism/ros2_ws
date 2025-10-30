#!/usr/bin/env python3
import cv2
import numpy as np
import yaml
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Transform, Pose
from cv_ros_bridge.arucoPub import publish_pose_and_transform

class ArucoImage(Node):
    def __init__(self):
        super().__init__('ArucoImage')
        self.image_raw = self.create_publisher(Image,'camera/image_raw', 10)
        self.image_aruco = self.create_publisher(Image, 'camera/image_aruco', 10)
        
        self.mark2cam_pose = self.create_publisher(Pose, 'camera/pose/mark/cam', 10)
        self.mark2cam_tf = self.create_publisher(Transform, 'camera/tf/mark/cam', 10)
        
        self.cam2mark_pose = self.create_publisher(Pose, 'camera/pose/cam/mark', 10)
        self.cam2mark_tf = self.create_publisher(Transform, 'camera/tf/cam/mark', 10)
        self.bridge = CvBridge()
        
        self.aruco_size = cv2.aruco.DICT_4X4_100
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_size)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(
            dictionary=self.aruco_dict, detectorParams=self.detector_params
        )
        self.cap = cv2.VideoCapture(index=4)

        with open("camera_calibration/realsense_d435.yaml", "r") as f:
            params = yaml.safe_load(f)
        self.k = np.array(params["K"], dtype=np.float64)
        self.D = np.array(params["D"], dtype=np.float64).ravel()

        self.marker_length = 0.1
        
        self.timer = self.create_timer(1e-6, self.timer_callback)

    def timer_callback(self):
            _, frame = self.cap.read()
            msg_raw = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_raw.publish(msg_raw)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = self.detector.detectMarkers(gray)
            
            if ids is not None:
                detected_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
                msg_aruco = self.bridge.cv2_to_imgmsg(detected_markers, encoding="bgr8")
                self.image_aruco.publish(msg_aruco)
                
                rvecs, tvecs, objpts = cv2.aruco.estimatePoseSingleMarkers(
                    corners=corners, markerLength=self.marker_length, cameraMatrix=self.k, distCoeffs=self.D
                )

                t = tvecs[0][0]
                rvec = rvecs[0][0]
                R, _ = cv2.Rodrigues(rvec)

                # Marker to Camera
                publish_pose_and_transform(R, t, self.mark2cam_pose, self.mark2cam_tf)

                # Camera to Marker (inverse)
                R_inv = R.T
                t_inv = -R_inv @ t
                publish_pose_and_transform(R_inv, t_inv, self.cam2mark_pose, self.cam2mark_tf)

def main(args=None):
    rclpy.init(args=args)
    aruco_image = ArucoImage()
    try:
        rclpy.spin(aruco_image)
    finally:
        aruco_image.cap.release()
        aruco_image.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()