import numpy as np
import open3d as o3d

class filter :
    def pass_through_filter_radius(roi, pcd):
        points = np.asarray(pcd.points)
        pass_through_filter = (np.sqrt(np.multiply(points[:,0], points[:,0]) + 
                                    np.multiply(points[:,1], points[:,1]) +
                                    np.multiply(points[:,2]. points[:,2])) < roi["r"][0])
        
        temp = o3d.geometry.PointCloud()
        temp.points = o3d.utility.Vector3dVector(points[pass_through_filter])
        
        return temp