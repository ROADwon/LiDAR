import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import cv2 as cv

calib_file_path = "C:/Users/line/Desktop/data_odometry_calib/dataset/sequences/00/calib.txt"
pcd_file_path = "./dataset/process_data/lidar/0_nosie_removal.txt"
next_frame_path = "./dataset/process_data/lidar/1_nosie_removal.txt"
calib = np.loadtxt(calib_file_path)



P0 = calib[0].reshape(3, 4)
P1 = calib[1].reshape(3, 4)
P2 = calib[2].reshape(3, 4)
P3 = calib[3].reshape(3, 4)
Tr_velo_to_cam = calib[4].reshape(3, 4)
Tr_velo_to_cam = np.vstack([Tr_velo_to_cam, [0, 0, 0, 1]])

P_rect = calib[:3, :4]

pcd = o3d.io.read_point_cloud(pcd_file_path)
pcd_next = o3d.io.read_point_cloud(next_frame_path)

lidar_data = np.asarray(pcd.points)
next_lidar_data = np.asarray(pcd_next.points)

lidar_data_homogeneous = np.hstack((lidar_data, np.ones((lidar_data.shape[0], 1))))
lidar_cam = lidar_data_homogeneous.dot(np.linalg.inv(Tr_velo_to_cam).T)
lidar_img = P_rect.dot(lidar_cam.T).T
lidar_img[:, :2] /= lidar_img[:, 2][:, np.newaxis]

next_data_homogeneous = np.hstack((next_lidar_data, np.ones([next_lidar_data.shape[0], 1])))
next_cam = next_data_homogeneous.dot(np.linalg.inv(Tr_velo_to_cam).T)
next_frame = P_rect.dot(next_cam.T).T
next_frame[:, :2] /= next_frame[:, 2][:, np.newaxis]

print("Transformed Lidar points in camera coordinates:")
print(lidar_cam)

print("\nProjected Lidar points onto Images:")
print(lidar_img)

lidar_pcd = o3d.geometry.PointCloud()
lidar_pcd.points = o3d.utility.Vector3dVector(lidar_cam[:, :3])

next_pcd = o3d.geometry.PointCloud()
next_pcd.points = o3d.utility.Vector3dVector(next_cam[:, :3])

# ICP
distance_threshold = 1.0
result = o3d.pipelines.registration.registration_icp(
    lidar_pcd, next_pcd, distance_threshold, np.eye(4),
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
)

transformation = result.transformation
print("Estimated Odometry Transformation Matrix:")
print(transformation)
lidar_pcd_transformed = lidar_pcd.transform(transformation)


def icp_odometry(source_pcd, target_pcd, threshold=0.1, max_iter=50):
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
        max_iteration=max_iter,
        relative_fitness=threshold,
        relative_rmse=threshold
    )
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, distance_threshold, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=criteria
    )
    return reg_p2p.transformation

# 나머지 코드는 이전과 동일


def accumulate_odometry(initial_transformation, odometry_transformation):
    return np.dot(odometry_transformation, initial_transformation)

initial_transformation = np.eye(4)

odometry_transformation = icp_odometry(lidar_pcd,next_pcd)

# 누적 운동 업데이트
cumulative_transformation = accumulate_odometry(initial_transformation, odometry_transformation)

print("Estimated Odometry Transformation Matrix:")
print(odometry_transformation)
print("\nCumulative Transformation Matrix:")
print(cumulative_transformation)
# 변환 전 포인트 클라우드 시각화
# o3d.visualization.draw_geometries([lidar_pcd], "Original Point Cloud")

# # 변환 후 포인트 클라우드 시각화
# o3d.visualization.draw_geometries([lidar_pcd_transformed], "Transformed Point Cloud")

# making trajectory code
tra_cumulative_transformation = np.eye(4)
traj = np.zeros((1000,1000,3), dtype=np.uint8)
curr_point = np.array([500, 500])
print("check point")
for i in range(1, 1000):
    tra_next_frame_path = f"C:/Users/line/Desktop/data_odometry_velodyne/dataset/sequences/00/velodyne_pcd/{i:06d}.pcd"
    tra_pcd_next = o3d.io.read_point_cloud(tra_next_frame_path)

    tra_next_lidar_data = np.asarray(tra_pcd_next.points)  # '.points'를 사용하여 포인트 배열을 가져옵니다.
    tra_next_data_homogeneous = np.hstack((tra_next_lidar_data, np.ones([tra_next_lidar_data.shape[0], 1])))
    tra_next_cam = tra_next_data_homogeneous.dot(np.linalg.inv(Tr_velo_to_cam).T)
    tra_next_frame = P_rect.dot(tra_next_cam.T).T
    tra_next_frame[:, :2] /= tra_next_frame[:, 2][:, np.newaxis]

    tra_odometry_transform = icp_odometry(lidar_pcd, tra_pcd_next)
    tra_cumulative_transformation = accumulate_odometry(tra_cumulative_transformation, tra_odometry_transform)
    curr_point = tra_odometry_transform[:2,3] * 50 + curr_point
    curr_point = curr_point.astype(int)

    cv.circle(traj, tuple(curr_point), 1, (0, 255, 0),2)
    cv.imshow('Trajectory', traj)
    cv.waitKey(5)
    
cv.destroyAllWindows()