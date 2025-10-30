#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf_transformations import quaternion_slerp


def translation_to_numpy(t):
    return np.array([t.x, t.y, t.z])


def quaternion_to_numpy(q):
    return np.array([q.x, q.y, q.z, q.w])


class MarkerFilterNode(Node):
    def __init__(self):
        super().__init__('marker_filter')
        self.declare_parameter('alpha', 0.9)
        self.declare_parameter('parent_frame_id', 'kinect2_link')
        self.declare_parameter('marker_id', 'marker0')
        self.declare_parameter('marker_filtered_id', 'marker_id0_filtered')
        self.declare_parameter('rate_value', 125)

        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value
        self.parent_frame_id = self.get_parameter('parent_frame_id').get_parameter_value().string_value
        self.marker_id = self.get_parameter('marker_id').get_parameter_value().string_value
        self.marker_filtered_id = self.get_parameter('marker_filtered_id').get_parameter_value().string_value
        self.rate_value = self.get_parameter('rate_value').get_parameter_value().integer_value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.marker_pose = None
        self.marker_pose0 = None

        self.timer = self.create_timer(1.0 / self.rate_value, self.timer_callback)

    def timer_callback(self):
        self.marker_pose0 = self.marker_pose
        try:
            marker_pose_new = self.tf_buffer.lookup_transform(
                self.parent_frame_id, self.marker_id, rclpy.time.Time())
            if marker_pose_new is not None:
                self.marker_pose = marker_pose_new
        except Exception as e:
            self.get_logger().warn(str(e))
            return

        if self.marker_pose is None:
            return

        # Apply running average filter to translation and rotation
        if self.marker_pose0 is not None:
            rotation0 = quaternion_to_numpy(self.marker_pose0.transform.rotation)
            rotation = quaternion_to_numpy(self.marker_pose.transform.rotation)
            rotation_interpolated = quaternion_slerp(rotation0, rotation, 1 - self.alpha)

            translation0 = translation_to_numpy(self.marker_pose0.transform.translation)
            translation = translation_to_numpy(self.marker_pose.transform.translation)
            translation = self.alpha * translation0 + (1 - self.alpha) * translation

            # Update pose of the marker
            self.marker_pose.transform.rotation.x = rotation_interpolated[0]
            self.marker_pose.transform.rotation.y = rotation_interpolated[1]
            self.marker_pose.transform.rotation.z = rotation_interpolated[2]
            self.marker_pose.transform.rotation.w = rotation_interpolated[3]
            self.marker_pose.transform.translation.x = translation[0]
            self.marker_pose.transform.translation.y = translation[1]
            self.marker_pose.transform.translation.z = translation[2]

        # Create new transform and broadcast it
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = self.marker_filtered_id
        t.transform = self.marker_pose.transform
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
