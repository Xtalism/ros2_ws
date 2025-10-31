import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformBroadcaster

#ARUCO CAMARA ROS2 PUBLISHER
class aruco(Node):
    def __init__(self)-> None:
        super().__init__('aruco')

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/video', 10)

        self.pose_pub   = self.create_publisher(PoseStamped, 'geometry_msg/PoseStamped', 10)
        self.marker_pub = self.create_publisher(Marker, 'vizualization_msg/Marker', 10)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/camera_info', 10)

        self.cam_in_marker_pub = self.create_publisher(PoseStamped, '/video/camera_in_marker/pose', 10)
        self.marker_video_pub = self.create_publisher(Marker, 'marker_video', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara")
            raise SystemExit

        CALIB_PATH = "camera_calibration/realsense_d435.yaml"
        with open(CALIB_PATH, "r") as f:
            calib = yaml.safe_load(f)
        self.K = np.array(calib["K"], dtype=np.float64)
        self.D = np.array(calib["D"], dtype=np.float64)

        self.markerLength = 0.1 # m

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict=cv2.aruco.DICT_4X4_100)
        try:
            self.detector_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(dictionary=self.aruco_dict, detectorParams=self.detector_params)
            self.use_class = True
        except AttributeError:
            self.detector_params = cv2.aruco.DetectorParameters_create()
            self.detector = None
            self.use_class = False

        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

    def timer_callback(self)-> None:
        ok, frame = self.cap.read()
        if not ok:
            return

        h, w = frame.shape[:2]

        gray = cv2.cvtColor(src=frame, code=cv2.COLOR_BGR2GRAY)
        if self.use_class:
            corners, ids, rejected = self.detector.detectMarkers(image=gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.markerLength, self.K, self.D)
            t = tvecs[0][0]
            rvec = rvecs[0][0]
            R, _ = cv2.Rodrigues(rvec)
            A = np.eye(4); A[:3,:3]=R; A[:3,3]=t
            B = np.eye(4); B[:3,:3]=R.T; B[:3,3]= -R.T @ t

            # CÁMARA EN MARCO DEL MARCADOR (marker_frame -> video_frame)
            R_cm = R.T
            t_cm = (-R_cm @ t).reshape(3)
            qcx, qcy, qcz, qcw = self._quat_from_rotmat(R_cm)

            cam_pose = PoseStamped()
            cam_pose.header.stamp = self.get_clock().now().to_msg()
            cam_pose.header.frame_id = 'marker_frame'
            cam_pose.pose.position.x = float(t_cm[0])
            cam_pose.pose.position.y = float(t_cm[1])
            cam_pose.pose.position.z = float(t_cm[2])
            cam_pose.pose.orientation.x = float(qcx)
            cam_pose.pose.orientation.y = float(qcy)
            cam_pose.pose.orientation.z = float(qcz)
            cam_pose.pose.orientation.w = float(qcw)
            self.cam_in_marker_pub.publish(cam_pose)

            # TF: marker_frame -> video_frame
            tf_cam = TransformStamped()
            tf_cam.header = cam_pose.header
            tf_cam.child_frame_id = 'video_frame'
            tf_cam.transform.translation.x = float(t_cm[0])
            tf_cam.transform.translation.y = float(t_cm[1])
            tf_cam.transform.translation.z = float(t_cm[2])
            tf_cam.transform.rotation.x = float(qcx)
            tf_cam.transform.rotation.y = float(qcy)
            tf_cam.transform.rotation.z = float(qcz)
            tf_cam.transform.rotation.w = float(qcw)
            self.tf_broadcaster.sendTransform(tf_cam)

            # MARCADOR EN MARCO DE LA CÁMARA (video_frame -> aruco_marker)
            qx, qy, qz, qw = self._quat_from_rotmat(R)
            marker_pose = PoseStamped()
            marker_pose.header.stamp = cam_pose.header.stamp
            marker_pose.header.frame_id = 'video_frame'
            marker_pose.pose.position.x = float(t[0])
            marker_pose.pose.position.y = float(t[1])
            marker_pose.pose.position.z = float(t[2])
            marker_pose.pose.orientation.x = float(qx)
            marker_pose.pose.orientation.y = float(qy)
            marker_pose.pose.orientation.z = float(qz)
            marker_pose.pose.orientation.w = float(qw)
            self.pose_pub.publish(marker_pose)

            m = Marker()
            m.header = marker_pose.header
            m.ns = "marker_in_camera"
            m.id = int(ids[0][0])
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose = marker_pose.pose
            m.scale.x = self.markerLength
            m.scale.y = self.markerLength
            m.scale.z = 0.01
            m.color.r = 0.1
            m.color.g = 0.8
            m.color.b = 0.2
            m.color.a = 1.0
            self.marker_pub.publish(m)

            # TF: video_frame -> aruco_marker
            tf_marker = TransformStamped()
            tf_marker.header = marker_pose.header
            tf_marker.child_frame_id = 'aruco_marker'
            tf_marker.transform.translation.x = float(t[0])
            tf_marker.transform.translation.y = float(t[1])
            tf_marker.transform.translation.z = float(t[2])
            tf_marker.transform.rotation.x = float(qx)
            tf_marker.transform.rotation.y = float(qy)
            tf_marker.transform.rotation.z = float(qz)
            tf_marker.transform.rotation.w = float(qw)
            self.tf_broadcaster.sendTransform(tf_marker)

            # CUBO AZUL PARA LA CÁMARA EN marker_video (visual)
            cam_marker = Marker()
            cam_marker.header = cam_pose.header
            cam_marker.ns = "camera_box"
            cam_marker.id = 0
            cam_marker.type = Marker.CUBE
            cam_marker.action = Marker.ADD
            cam_marker.pose = cam_pose.pose
            cam_marker.scale.x = 0.05
            cam_marker.scale.y = 0.03
            cam_marker.scale.z = 0.03
            cam_marker.color.r = 0.1
            cam_marker.color.g = 0.3
            cam_marker.color.b = 1.0
            cam_marker.color.a = 1.0
            self.marker_video_pub.publish(cam_marker)

        img_msg: Image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.frame_id = 'video_frame'
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(img_msg)

        cam = CameraInfo()
        cam.header = img_msg.header
        cam.width  = int(w); cam.height = int(h)
        cam.distortion_model = 'plumb_bob'
        cam.d = self.D.flatten().tolist()
        K = self.K.flatten()
        cam.k = [float(x) for x in K]
        cam.r = [1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0]
        fx, fy, cx, cy = self.K[0,0], self.K[1,1], self.K[0,2], self.K[1,2]
        cam.p = [float(fx),0.0,float(cx),0.0, 0.0,float(fy),float(cy),0.0, 0.0,0.0,1.0,0.0]
        self.cam_info_pub.publish(cam)

    def _quat_from_rotmat(self, R: np.ndarray):
        t = np.trace(R)
        if t > 0:
            s = np.sqrt(t + 1.0) * 2.0
            qw = 0.25 * s
            qx = (R[2,1] - R[1,2]) / s
            qy = (R[0,2] - R[2,0]) / s
            qz = (R[1,0] - R[0,1]) / s
        elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            s = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
            qw = (R[2,1] - R[1,2]) / s
            qx = 0.25 * s
            qy = (R[0,1] + R[1,0]) / s
            qz = (R[0,2] + R[2,0]) / s
        elif R[1,1] > R[2,2]:
            s = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
            qw = (R[0,2] - R[2,0]) / s
            qx = (R[0,1] + R[1,0]) / s
            qy = 0.25 * s
            qz = (R[1,2] + R[2,1]) / s
        else:
            s = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
            qw = (R[1,0] - R[0,1]) / s
            qx = (R[0,2] + R[2,0]) / s
            qy = (R[1,2] + R[2,1]) / s
            qz = 0.25 * s
        return qx, qy, qz, qw

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = aruco()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()