import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import os
from glob import glob

file_path = "C:/Users/line/Desktop/LiDAR/dataset/process_data/lidar/"


def icp(prev_pcd, corr_pcd, threshold=0.1, max_iter=50):
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration= max_iter,
                                                                 relative_fitness= threshold,
                                                                 relative_rmse= threshold)
    reg_p2p = o3d.pipelines.registration.registration_icp(prev_pcd, corr_pcd, np.eye(4),
                                                          o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                          criteria= criteria)

    return reg_p2p.transformation


def extract_translation_from_matrix(transformation_matrix):
    translation_vector = transformation_matrix[:3,3]
    return translation_vector


def update_transformation_matrix(curr_mat, icp_transformation):
    icp_translation = extract_translation_from_matrix(icp_transformation)
    
    update_translation = curr_mat[:3,3] + icp_translation
    update_mat = np.eye(4)
    update_mat[:3,3] = update_translation
    
    return update_mat

# 초기 변환 행렬 설정 (단위 행렬)
transformation_matrix = np.eye(4)

vis = o3d.visualization.Visualizer()
vis.create_window()

file_list = sorted([file_name for file_name in os.listdir(file_path) if file_name.endswith('.txt')], key=lambda x: int(x.split('_')[0]))
view_control = vis.get_view_control()

for i, file in file_list:
    file_full_path = os.path.join(file_path, file)
    if os.path.isfile(file_full_path):
        points = np.loadtxt(file_full_path, delimiter=',')
        prev_pcd = o3d.geometry.PointCloud[i]
        corr_pcd = o3d.geometry.PointCloud[i + 1]
        
        
        # 현재 프레임에서의 변환 행렬 업데이트 (이동값 등 적용)
        transformation_matrix = icp(prev_pcd, corr_pcd)  # update_transformation_matrix 함수는 변환 행렬을 업데이트하는 사용자 정의 함수입니다.

        # 현재 프레임에서의 LIDAR 데이터를 전체 맵 좌표계로 변환
        points_transformed = np.dot(transformation_matrix[:3, :3], points[:, :3].T).T + transformation_matrix[:3, 3]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_transformed)

        vis.clear_geometries()
        vis.add_geometry(pcd)

        # 시각화 업데이트
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

        # 초점 설정
        view_control.set_lookat([0, 0, 0])  # 시각화의 중심점 설정
        view_control.set_constant_z_far(10)  # 시각화 중심으로부터의 거리 설정

# 시각화 창 닫기
vis.destroy_window()
