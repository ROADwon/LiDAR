import argparse
import glob
import os
import numpy as np
import open3d as o3d
import json

visualize_on = False

def change_background_to_black(vis):
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])
    print("Successfully Change the Background Color ")
    return False

def pass_through_filter(roi, pcd):
    points = np.asarray(pcd.points)
    x_range = np.logical_and(points[:,0] >= roi["x"][0], points[:,0] <= roi["x"][1])
    y_range = np.logical_and(points[:,1] >= roi["y"][0], points[:,1] <= roi["y"][1])
    z_range = np.logical_and(points[:,2] >= roi["z"][0], points[:,2] <= roi["z"][1])
    
    pass_through_filter = np.logical_and(x_range, np.logical_and(y_range, z_range))
    temp = o3d.geometry.PointCloud()
    temp.points = o3d.utility.Vector3dVector(points[pass_through_filter])
    return temp

# def pass_through_filter_radius(roi, pcd):
#     points = np.asarray(pcd.points)
#     pass_through_filter = (np.sqrt(np.multiply(points[:,0], points[:,0]) +
#                                    np.multiply(points[:,1], points[:,1]) +
#                                    np.multiply(points[:,2], points[:,2]) <= roi["r"][0]))
#     temp = o3d.geometry.PointCloud()
#     temp.points = o3d.utility.Vector3dVector(points[[pass_through_filter]])
#     return temp

def pass_through_filter_radius(roi, pcd):
    points = np.asarray(pcd.points)
    pass_through_filter = (np.sqrt(np.sum(np.square(points[:, :3]), axis=1)) <= roi["r"][0])
    temp = o3d.geometry.PointCloud()
    temp.points = o3d.utility.Vector3dVector(points[pass_through_filter])
    print("Successfully set ROI radius")
    return temp


def extract_roi(roi, pcd):
    pcd_filtered = pass_through_filter_radius(roi, pcd)
    print("Successfully Filtered")
    return pcd_filtered

def noise_removal(pcd):
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd_filtered = pcd.select_by_index(ind)
    print("Successfully remove noise")
    return pcd_filtered

def save_pointcloud(pcd, ind, file_name, folder):
    if not os.path.exists(folder):
        os.makedirs(folder)
        print("make a foledr...")
    if not os.path.exists(folder + '/lidar/'):
        os.makedirs(folder + '/lidar/')
        print("make a lidar folder ... ")
    if not os.path.exists(folder + '/label/'):
        os.makedirs(folder + '/label')
        print("make a label folder ...")

    lidar_name = folder + '/lidar/' + str(ind) + '_' + file_name + '.txt'
    anno_name = folder + '/label/' + str(ind) + '_' + file_name + '.json'

    xyz = np.asarray(pcd.points)
    xyz = np.hstack((xyz, np.ones((xyz.shape[0],1), dtype=xyz.dtype)))

    np.savetxt(lidar_name, xyz, delimiter=',')

    anno_file = open(anno_name, "w")
    anno_file.write("{\n}")
    
    anno_file.close()

def run(config):
    print(" Starting ... ")
    path = config["pcd_path"]

    pcd_list = glob.glob(path + '*.pcd')
    nfiles = len(pcd_list)

    roi = {"x":[-20.0, 20.0],
           "y":[-20.0, 20.0],
           "z":[-10.0, 10.0]}

    roi_radius = {"r":[18]}

    dataset_dir = "./dataset/process_data"

    for i in range(0, nfiles):
        pcd = o3d.io.read_point_cloud(pcd_list[i])
        roi_pcd = extract_roi(roi_radius, pcd)
        noise_removal_pcd = noise_removal(roi_pcd)

        #save_pointcloud(pcd, i, "1raw", dataset_dir)
        #save_pointcloud(roi_pcd, i, "2roi", dataset_dir)
        save_pointcloud(noise_removal_pcd, i, "noise_removal", dataset_dir)
        print(f"Save point cloud text file in {dataset_dir}...")
if __name__ == "__main__":
    # Load configuration from JSON file
    with open('config.json', 'r') as f:
        config = json.load(f)

    run(config)
