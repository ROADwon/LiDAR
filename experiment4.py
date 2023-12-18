import cv2
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
# Load camera calibration parameters
calib_file_path = "C:/Users/line/Desktop/data_odometry_calib/dataset/sequences/00/calib.txt"
calib = np.loadtxt(calib_file_path)

# Load image and LiDAR data
img_file_path = 'C:/Users/line/Downloads/data_odometry_gray/dataset/sequences/00/image_0/000000.png'
lidar_file_path = "C:/Users/line/Desktop/data_odometry_velodyne/dataset/sequences/00/velodyne/000000.bin"

img = cv2.imread(img_file_path)
lidar_data = np.fromfile(lidar_file_path, dtype=np.float32).reshape(-1, 4)

# Extract camera and LiDAR calibration matrices
P0 = calib[0].reshape(3,4)
P1 = calib[1].reshape(3,4)
P2 = calib[2].reshape(3,4)
P3 = calib[3].reshape(3,4)
Tr = calib[4].reshape(3,4)
Tr = np.vstack([Tr, [0, 0, 0, 1]])
P_rect = calib[:3, :4]
R0 = np.eye(4)
R0[0:3,0:3] = P_rect.reshape(3, -1)
P = P2.reshape(-1,4)

XYZ1 = np.vstack((lidar_data[:,0:3].T, np.ones((1, lidar_data.shape[0]))))
xy1 = P @ R0 @ Tr @ XYZ1
s = xy1[2, :]
x = xy1[0, :] / s
y = xy1[1, :] / s

plt.plot(x,y,'.')
plt.gca().invert_yaxis
img_h, img_w = img.shape[0], img.shape[1]
plt.xlim([0, img_w])
plt.ylim([0, img_h])
plt.show()