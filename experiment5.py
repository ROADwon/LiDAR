import numpy as np
import open3d as o3d

calib_file_path = "C:/Users/line/Desktop/data_odometry_calib/dataset/sequences/00/calib.txt"
calib = np.loadtxt(calib_file_path)

# Load image and LiDAR data
img_file_path = 'C:/Users/line/Downloads/data_odometry_gray/dataset/sequences/00/image_0/000000.png'
lidar_file_path = "C:/Users/line/Desktop/data_odometry_velodyne/dataset/sequences/00/velodyne/000000.pcd"
point_cloud = o3d.io_r

pcd = o3d.io.read_point_cloud(lidar_file_path)
print(pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])