import numpy as np
import open3d as o3d

def compute_odometry(prev_points, current_points):
    translation = current_points.mean(axis=0) - prev_points.mean(axis=0)
    
    return translation

lidar_file_path = "C:/Users/line/Desktop/data_odometry_velodyne/dataset/sequences/00/velodyne/"
point_cloud = o3d.io.read_point_cloud(lidar_file_path)
points = np.asarray(point_cloud.points)

num_frames = len(points)
for frame in range(1, num_frames):
    prev_frame_points = points[frame - 1]
    current_frame_points = points[frame]
    
    translation = compute_odometry(prev_frame_points, current_frame_points)
    print(f"Frame {frame} : Translation vector : {translation}")