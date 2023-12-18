import open3d as o3d
import numpy as np
from glob import glob
import cv2
import random


lidar_file_path = "C:/Users/line/Desktop/data_odometry_velodyne/dataset/sequences/00/velodyne/"
img_file_path = "C:/Users/line/Downloads/data_odometry_gray/dataset/sequences/00/image_0/"

# Load LiDAR data
lidar_files = sorted(glob(f"{lidar_file_path}*.bin"))
lidar_data = [np.fromfile(file, dtype=np.float32).reshape(-1, 4) for file in lidar_files]

# Load image data (optional, depending on your use case)
img_files = sorted(glob(f"{img_file_path}*.png"))
images = [cv2.imread(file) for file in img_files]

# Create Open3D visualizer
vis = o3d.visualization.Visualizer()
vis.create_window(window_name='KITTI Odometry Viewer')

# Create Open3D point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(lidar_data[0][:, :3])
colors = (lidar_data[0][:, 2] - np.min(lidar_data[0][:, 2])) / (np.max(lidar_data[0][:, 2]) - np.min(lidar_data[0][:, 2]))
colors = np.stack([1 - colors, colors, np.zeros_like(colors)], axis=-1)  # Example: Gradient from blue to red
pcd.colors = o3d.utility.Vector3dVector(colors)

# Visualize point cloud
vis.add_geometry(pcd)

# Main visualization loop
for i in range(len(lidar_files)):
    # Update point cloud
    pcd.points = o3d.utility.Vector3dVector(lidar_data[i][:, :3])
    colors = (lidar_data[i][:, 2] - np.min(lidar_data[i][:, 2])) / (np.max(lidar_data[i][:, 2]) - np.min(lidar_data[i][:, 2]))
    colors = np.stack([1 - colors, colors, np.zeros_like(colors)], axis=-1)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Update image (optional, depending on your use case)
    img = o3d.geometry.Image(images[i])
    tex = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
    tex.textures = [o3d.geometry.Image(images[i])]
    vis.update_geometry(tex)

    # Update visualization
    vis.poll_events()
    vis.update_renderer()

# Wait until the user closes the window
vis.run()
vis.destroy_window()


def random_downsample(point_cloud, ratio):
    num_points = point_cloud.shape[0]
    num_sampled_points = int(num_points * ratio)
    
    sampled_indices = random.sample(range(num_points), num_sampled_points)
    
    downsampled_points = point_cloud[sampled_indices, :]
    
    return downsampled_points

