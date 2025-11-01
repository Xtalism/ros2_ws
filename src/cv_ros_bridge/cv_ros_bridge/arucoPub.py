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

def marker_scale(marker_type, scale_x, scale_y, scale_z, color_r, color_g, 
                 color_b, color_a, marker_action=None, mesh_resource=None):
    marker = Marker()
    marker.ns = "aruco"
    marker.id = 0
    marker.type = marker_type
    marker.scale.x = scale_x
    marker.scale.y = scale_y
    marker.scale.z = scale_z
    marker.color.r = color_r
    marker.color.g = color_g
    marker.color.b = color_b
    marker.color.a = color_a
    marker.lifetime.sec = 0
    if marker_type == Marker.MESH_RESOURCE and mesh_resource is not None:
        marker.mesh_resource = mesh_resource
        marker.mesh_use_embedded_materials = True
    else:
        marker.action = marker_action
    return marker

def publish_visualization(R, t, publisher, clock, frame_id, is_marker=True):
    if is_marker:
        marker = marker_scale(Marker.CUBE, 0.05, 0.03, 0.03, 0.1, 0.3, 1.0, 1.0, marker_action=Marker.ADD)
        marker.header.frame_id = frame_id
        marker.header.stamp = clock.now().to_msg()
    else:
        marker = marker_scale(Marker.MESH_RESOURCE, 0.0005, 0.0005, 0.0005, 0.5, 0.5, 0.5, 1.0, 
                              mesh_resource="file:///home/xtal/ros2_ws/mesh/D435_Solid.stl")
        marker.header.frame_id = frame_id
        marker.header.stamp = clock.now().to_msg()
    marker.pose = Pose()
    set_rt(marker.pose, R, t, 'position', 'orientation')

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