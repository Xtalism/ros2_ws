#!/usr/bin/env python3
from scipy.spatial.transform import Rotation as SciRot
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import CameraInfo

def set_rt(obj, R, t, field_t, field_r):
    quat = SciRot.from_matrix(R).as_quat()
    trans = getattr(obj, field_t)
    rot = getattr(obj, field_r)
    for attr, value in zip(['x', 'y', 'z'], t):
        setattr(trans, attr, float(value))
    for attr, value in zip(['x', 'y', 'z', 'w'], quat):
        setattr(rot, attr, float(value))

def publish_visualization(R, t, publisher, clock, frame_id):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = clock.now().to_msg()
    marker.ns = "aruco"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = Pose()
    set_rt(marker.pose, R, t, 'position', 'orientation')
    marker.scale.x = 0.05
    marker.scale.y = 0.03
    marker.scale.z = 0.03
    marker.color.r = 0.1
    marker.color.g = 0.3
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.lifetime.sec = 0
    publisher.publish(marker)

def publish_pose(R, t, publisher, clock, frame_id):
    pose = PoseStamped()
    set_rt(pose.pose, R, t, 'position', 'orientation')
    pose.header.stamp = clock.now().to_msg()
    pose.header.frame_id = frame_id
    publisher.publish(pose)

def publish_tf(R, t, broadcaster, clock, parent_frame, child_frame):
    tf = TransformStamped()
    tf.header.stamp = clock.now().to_msg()
    tf.header.frame_id = parent_frame
    tf.child_frame_id = child_frame
    set_rt(tf.transform, R, t, 'translation', 'rotation')
    broadcaster.sendTransform(tf)

def publish_camera_info(K, D, width, height, publisher, clock, frame_id):
    cam = CameraInfo()
    cam.header.stamp = clock.now().to_msg()
    cam.header.frame_id = frame_id
    cam.width = int(width)
    cam.height = int(height)
    cam.distortion_model = 'plumb_bob'
    cam.d = D.flatten().tolist()
    cam.k = [float(x) for x in K.flatten()]
    cam.r = [1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0]
    fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
    cam.p = [float(fx),0.0,float(cx),0.0, 0.0,float(fy),float(cy),0.0, 0.0,0.0,1.0,0.0]
    publisher.publish(cam)