#!/usr/bin/env python3
from scipy.spatial.transform import Rotation as SciRot
from geometry_msgs.msg import Transform, Pose

def publish_pose_and_transform(R, t, pose_pub, tf_pub):
    quat = SciRot.from_matrix(R).as_quat()
    pose_msg = Pose()
    pose_msg.position.x = float(t[0])
    pose_msg.position.y = float(t[1])
    pose_msg.position.z = float(t[2])
    pose_msg.orientation.x = float(quat[0])
    pose_msg.orientation.y = float(quat[1])
    pose_msg.orientation.z = float(quat[2])
    pose_msg.orientation.w = float(quat[3])
    pose_pub.publish(pose_msg)

    transform_msg = Transform()
    transform_msg.translation.x = float(t[0])
    transform_msg.translation.y = float(t[1])
    transform_msg.translation.z = float(t[2])
    transform_msg.rotation.x = float(quat[0])
    transform_msg.rotation.y = float(quat[1])
    transform_msg.rotation.z = float(quat[2])
    transform_msg.rotation.w = float(quat[3])
    tf_pub.publish(transform_msg)