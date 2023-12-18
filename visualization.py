import open3d as o3d
import numpy as np
import os
from glob import glob
file_path = "./dataset/process_data/lidar/"

# 초기화
visualizer = o3d.visualization.Visualizer()
visualizer.create_window()

# 파일 리스트 얻기 및 정렬
file_list = sorted([file_name for file_name in os.listdir(file_path) if file_name.endswith('.txt')], key=lambda x: int(x.split('_')[0]))

# 시각화 컨트롤러 얻기
view_control = visualizer.get_view_control()

# 파일 순서대로 시각화
for file_name in file_list:
    file_full_path = os.path.join(file_path, file_name)
    if os.path.isfile(file_full_path):
        points = np.loadtxt(file_full_path, delimiter=',')
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        print("Processing Success ...")
        print("Add point cloud in visualizer ... ")
        
        # 기존 geometry 제거
        visualizer.clear_geometries()
        
        # 시각화 추가
        visualizer.add_geometry(pcd)

        # 시각화 업데이트
        visualizer.update_geometry(pcd)
        visualizer.poll_events()
        visualizer.update_renderer()

        # 초점 설정
        view_control.set_lookat([0, 0, 0])  # 시각화의 중심점 설정
        view_control.set_constant_z_far(10)  # 시각화 중심으로부터의 거리 설정

# 시각화 창 닫기
visualizer.destroy_window()
