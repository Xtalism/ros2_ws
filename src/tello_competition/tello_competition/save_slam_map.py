#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseStamped
import open3d as o3d
import numpy as np
from std_srvs.srv import Trigger
import os
from datetime import datetime

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        
        # Parameters
        self.declare_parameter('save_directory', os.path.expanduser('/home/xtal/ros2_ws/slam_maps/tello_maps'))
        self.declare_parameter('auto_save_interval', 30.0)  # seconds
        self.declare_parameter('min_points', 100)
        
        self.save_dir = self.get_parameter('save_directory').value
        auto_save_interval = self.get_parameter('auto_save_interval').value
        self.min_points = self.get_parameter('min_points').value
        
        # Create save directory if it doesn't exist
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Data storage
        self.map_points = []
        self.camera_poses = []
        self.point_cloud = o3d.geometry.PointCloud()
        
        # Subscriptions
        self.map_sub = self.create_subscription(
            PointCloud2,
            '/orb_slam3/map_points',
            self.map_callback,
            10)
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/orb_slam3/camera_pose',
            self.pose_callback,
            10)
        
        # Service to manually trigger save
        self.save_service = self.create_service(
            Trigger,
            'save_map',
            self.save_map_service)
        
        # Auto-save timer
        self.save_timer = self.create_timer(auto_save_interval, self.auto_save_callback)
        
        self.get_logger().info(f'Map saver initialized. Saving to: {self.save_dir}')

    def map_callback(self, msg):
        """Process incoming point cloud from ORB-SLAM3"""
        try:
            # Convert ROS PointCloud2 to numpy array
            points = []
            for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([point[0], point[1], point[2]])
            
            if len(points) > 0:
                self.map_points = np.array(points)
                self.point_cloud.points = o3d.utility.Vector3dVector(self.map_points)
                
                self.get_logger().info(f'Received {len(points)} map points', 
                                      throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def pose_callback(self, msg):
        """Store camera trajectory"""
        pose = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        self.camera_poses.append(pose)

    def save_map(self, filename_prefix="tello_map"):
        """Save the map in multiple formats"""
        if len(self.map_points) < self.min_points:
            self.get_logger().warn(f'Not enough points to save ({len(self.map_points)} < {self.min_points})')
            return False
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_filename = f"{filename_prefix}_{timestamp}"
        
        try:
            # Save as PCD (Point Cloud Data)
            pcd_file = os.path.join(self.save_dir, f"{base_filename}.pcd")
            o3d.io.write_point_cloud(pcd_file, self.point_cloud)
            self.get_logger().info(f'Saved point cloud to {pcd_file}')
            
            # Save as PLY (more compatible format)
            ply_file = os.path.join(self.save_dir, f"{base_filename}.ply")
            o3d.io.write_point_cloud(ply_file, self.point_cloud)
            
            # Save camera trajectory
            if len(self.camera_poses) > 0:
                traj_file = os.path.join(self.save_dir, f"{base_filename}_trajectory.txt")
                np.savetxt(traj_file, np.array(self.camera_poses), fmt='%.6f')
                self.get_logger().info(f'Saved trajectory to {traj_file}')
            
            # Save as numpy array (for further processing)
            npy_file = os.path.join(self.save_dir, f"{base_filename}.npy")
            np.save(npy_file, self.map_points)
            
            # Generate mesh if enough points
            if len(self.map_points) > 1000:
                self.create_mesh(base_filename)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error saving map: {e}')
            return False

    def create_mesh(self, base_filename):
        """Create a mesh from the point cloud using Poisson reconstruction"""
        try:
            # Estimate normals
            self.point_cloud.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            
            # Poisson surface reconstruction
            mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
                self.point_cloud, depth=9)
            
            # Remove low-density vertices
            vertices_to_remove = densities < np.quantile(densities, 0.01)
            mesh.remove_vertices_by_mask(vertices_to_remove)
            
            # Save mesh
            mesh_file = os.path.join(self.save_dir, f"{base_filename}_mesh.ply")
            o3d.io.write_triangle_mesh(mesh_file, mesh)
            self.get_logger().info(f'Saved mesh to {mesh_file}')
            
        except Exception as e:
            self.get_logger().error(f'Error creating mesh: {e}')

    def auto_save_callback(self):
        """Periodically save the map"""
        self.save_map("auto_save")

    def save_map_service(self, request, response):
        """Service callback for manual save"""
        success = self.save_map("manual_save")
        response.success = success
        response.message = f'Map saved with {len(self.map_points)} points' if success else 'Failed to save map'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down, saving final map...')
        node.save_map("final_map")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()