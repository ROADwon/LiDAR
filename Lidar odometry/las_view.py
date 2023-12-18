import numpy as np
import laspy

def bin_to_las(bin_file_path, las_file_path):
    
    data = np.fromfile(bin_file_path, dtype= np.float32)
    
    points = data.reshape((-1,3))
    header = laspy.header.Header()
    header.dataformat_id = 1  # LAS format 1
    header.point_format_id = 3  # LAS point format 3
    
    # Create a LAS file
    
    outfile = laspy.LasData(header=header, file_version="1.2")
    outfile.x = points[:, 0]
    outfile.y = points[:, 1]
    outfile.z = points[:, 2]
    
    
    outfile.write(las_file_path)
    
bin_file_path = "C:/Users/line/Desktop/data_odometry_velodyne/dataset/sequences/00/velodyne/000000.bin"
las_file_path = "C:/Users/line/Desktop/data_odometry_velodyne/dataset/sequences/00/velodyne_las/000000.las"
bin_to_las(bin_file_path, las_file_path)