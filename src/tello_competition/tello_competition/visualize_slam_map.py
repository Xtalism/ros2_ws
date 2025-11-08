#!/usr/bin/env python3
"""
Visualize saved SLAM maps
"""
import open3d as o3d
import numpy as np
import sys
import os

def visualize_map(pcd_file, trajectory_file=None):
    """Visualize a saved point cloud map with optional trajectory"""
    
    # Load point cloud
    pcd = o3d.io.read_point_cloud(pcd_file)
    print(f"Loaded {len(pcd.points)} points from {pcd_file}")
    
    # Color the point cloud
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    
    geometries = [pcd]
    
    # Load and visualize camera trajectory
    if trajectory_file and os.path.exists(trajectory_file):
        trajectory = np.loadtxt(trajectory_file)
        
        # Create line set for trajectory
        points = trajectory[:, :3]
        lines = [[i, i+1] for i in range(len(points)-1)]
        
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in lines])
        
        geometries.append(line_set)
        print(f"Loaded trajectory with {len(points)} poses")
    
    # Add coordinate frame
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    geometries.append(coordinate_frame)
    
    # Visualize
    o3d.visualization.draw_geometries(
        geometries,
        window_name="Tello SLAM Map",
        width=1280,
        height=720,
        left=50,
        top=50
    )

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 visualize_map.py <path_to_pcd_file> [trajectory_file]")
        print("Example: python3 visualize_map.py ~/tello_maps/tello_map_20241106_123456.pcd")
        return
    
    pcd_file = sys.argv[1]
    trajectory_file = sys.argv[2] if len(sys.argv) > 2 else None
    
    if not os.path.exists(pcd_file):
        print(f"Error: File {pcd_file} not found")
        return
    
    visualize_map(pcd_file, trajectory_file)

if __name__ == '__main__':
    main() 