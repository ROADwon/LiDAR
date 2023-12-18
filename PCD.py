import open3d as o3d
import numpy as np
from glob import glob
from SLAM import Node, Edge, SLAM
from scipy.optimize import minimize

# 파일 경로 설정
file_path = 'C:/Users/line/Desktop/KITTI_LiDAR/testing/velodyne/'
files = glob(f"{file_path}*.bin")

# Node와 Edge를 저장할 리스트 초기화
nodes = []
edges = []

# Visualizer 생성
vis = o3d.visualization.Visualizer()
vis.create_window()

# 데이터 로딩 및 초기 노드 생성
for file in files:
    data = np.fromfile(file, dtype=np.float32)
    print(f"File: {file}, Data size: {len(data)}")
    data = data.reshape(-1, 4)
    
    initial_pos = data[0, :3]
    nodes.append(Node(initial_pos))

# Edge 생성
for i in range(len(nodes) - 1):
    edges.append(Edge(nodes[i], nodes[i+1], nodes[i+1].position - nodes[i].position))

# SLAM 초기화
slam = SLAM(nodes, edges)

# 최적화 수행
result = slam.optimize()

# 최적화 결과 출력
print('Optimized Node Positions:')
for i, node in enumerate(nodes):
    print(f'Node {i + 1}: {node.position}')

# Visualizer에 Node를 추가하여 시각화
for node in nodes:
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1, resolution=20)
    sphere.translate(node.position)
    vis.add_geometry(sphere)

# Visualizer 실행
vis.run()
vis.destroy_window()
