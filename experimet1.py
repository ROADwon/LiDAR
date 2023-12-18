import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import scipy
import g2o

file_path = "C:/Users/line/Desktop/KITTI_LiDAR/testing/velodyne/000000.pcd"

pcd = o3d.io.read_point_cloud(file_path)
np_pcd = np.asarray(pcd.points)

optimizer = g2o.SparseOptimizer()
solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3()) 
algorithm = g2o.OptimizationAlgorithmLevenberg(solver)
optimizer.set_algorithm(algorithm)

for i in range(len(np_pcd)):
    node = g2o.VertexSE3()
    node.set_id(i)
    node.set_estimate(np.eye(4))
    optimizer.add_vertex(node)
    
for i in range(len(np_pcd) -1 ):
    edge = g2o.EdgeSE3()
    edge.set_vertex(0, optimizer.vertex(i))
    edge.set_vertex(1, optimizer.vertext(i + 1))
    edge.set_measurement(np.eye(4))
    edge.set_information(np.eye(6))
    
    optimizer.add_edge(edge)
    
optimizer.initializer_optimization()
optimizer.optimize(10)
    

