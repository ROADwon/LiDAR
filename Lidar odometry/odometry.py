import numpy as np
import open3d as o3d

def icp_odometry(source_pcd, target_pcd, threshold=0.1, max_iter=50):
    criteria = o3d.registration.ICPConvergenceCriteria(
        max_iteration=max_iter,
        relative_fitness=threshold,
        relative_rmse=threshold
    )
    reg_p2p = o3d.registration.registration_icp(
        source_pcd, target_pcd, np.eye(4),
        o3d.registration.TransformationEstimationPointToPoint(),
        criteria=criteria
    )
    return reg_p2p.transformation

def accumulate_odometry(initial_transformation, odometry_transformation):
    return np.dot(odometry_transformation, initial_transformation)

# 예제 코드에서는 lidar_pcd, lidar_pcd_next_frame 등을 사용하고 있다고 가정합니다.
# lidar_pcd는 현재 프레임의 포인트 클라우드이고, lidar_pcd_next_frame는 이전 프레임의 포인트 클라우드입니다.
# 또한, 초기 변환 행렬 initial_transformation은 단위 행렬로 초기화합니다.
initial_transformation = np.eye(4)

# ICP를 사용하여 현재 프레임과 이전 프레임 간의 운동 추정
odometry_transformation = icp_odometry(lidar_pcd, lidar_pcd_next_frame)

# 누적 운동 업데이트
cumulative_transformation = accumulate_odometry(initial_transformation, odometry_transformation)

print("Estimated Odometry Transformation Matrix:")
print(odometry_transformation)
print("\nCumulative Transformation Matrix:")
print(cumulative_transformation)
