import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import scipy

from glob import glob
from scipy.optimize import minimize
from mpl_toolkits.mplot3d import Axes3D

#define Node and Edge class
class Node :
    def __init__(self, position):
        self.position = position
        
class Edge :
    def __init__(self, node1, node2, measurement):
        self.node1 = node1
        self.node2 = node2
        self.measurement = measurement
        
class SLAM : 
    def __init__(self, nodes, edges):
        self.nodes = nodes
        self.edges = edges
    # calculating graph cost function
    def cost_function(self, params):
        total_cost = 0
        
        #update nodes position    
        for i, node in enumerate(self.nodes):
            node.position = params[i * 3: (i + 1)* 3]
        
        # calculating nodes cost    
        for edge in self.edges :
            diff = edge.node2.position - edge.node1.position
            error = np.linalg.norm(diff - edge.measurement)
            
            total_cost += error
            
        return total_cost
    
    def optimize(self):
        initial_params = np.concatenate([node.position for node in self.nodes])
        result = minimize(self.cost_function, initial_params, method='L-BFGS-B')
        optimized_params = result.x
        
        for i, node in enumerate(self.nodes):
            node.position = optimized_params[i*3: (i + 1) * 3]
            print(f'Optimized Nopde {i + 1}: {node.position}')



# node1 = Node(np.array([0.0, 0.0, 0.0]))
# node2 = Node(np.array([1.0, 0.0, 0.0]))
# node3 = Node(np.array([1.0, 1.0, 0.0]))

# nodes = [node1,node2,node3]

# edge1 = Edge(node1, node2, np.array([1.0, 0.0, 0.0]))
# edge2 = Edge(node2, node3, np.array([0.0, 1.0, 0.0]))

# edges = [edge1, edge2]


# # first node position 
# print('Initial Node Posi :')
# for i, node in enumerate(nodes):
#     print(f'Node{i + 1}: {node.position}')
    
# # optimization
# initial_params = np.concatnate([node.position for node in nodes])
# result = minimize(cost_function, initial_params, args=(edges, ), method= 'L-BFGS-B')

# # print optimization node position
# optimized_params = result.x
# for i, node in enumerate(nodes):
#     node.position = optimized_params[i*3: (i + 1) * 3]
#     print(f"Optimized Node {i + 1}: {node.position}")
    
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# for node in nodes :
#     ax.scatter(node.position[0], node.position[1], node.position[2],c='r', marker='o')
    
# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')

# plt.show()