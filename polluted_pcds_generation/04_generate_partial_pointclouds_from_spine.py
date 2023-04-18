import argparse
import subprocess
import os
import open3d as o3d
import numpy as np

# Credits to wenaouyan at https://github.com/wentaoyuan/pcn/tree/master/render
def rotate_pcd(partial_pointcloud):
    # rotate
    R = partial_pointcloud.get_rotation_matrix_from_xyz((-np.pi / 2, 0, 0))
    partial_pointcloud = partial_pointcloud.rotate(R, center=(0, 0, 0))
    return partial_pointcloud

def merge_pcds(pcds_files, to_save):
    all_pcd_points_list = []
    pcds_path = os.listdir(pcds_files)
    for pcd in pcds_path:
        full_path = os.path.join(pcds_files,pcd)
        curr_pcd = o3d.io.read_point_cloud(full_path)
        # rotate
        curr_pcd_rotated = rotate_pcd(curr_pcd)
        # add to the lsit of pcd points
        all_pcd_points_list.append(np.asarray(curr_pcd_rotated.points))

    all_pcd_points = np.concatenate(all_pcd_points_list)
    combined_pcd = o3d.geometry.PointCloud()
    combined_pcd.points = o3d.utility.Vector3dVector(all_pcd_points)
    o3d.io.write_point_cloud(to_save, combined_pcd)

if __name__ == '__main__':

    arg_parser = argparse.ArgumentParser(
        description="Generate partial point clouds that resemble the pointclouds that would be obtained from the utlrasound")

    arg_parser.add_argument(
        "--list_paths_for_raycasting",
        required=True,
        dest="list_paths_for_raycasting",
        help="Txt file that contains paths to the spines that will be raycasted"
    )

    arg_parser.add_argument(
        "--camera_poses",
        required=True,
        dest="camera_poses",
        help="Csv file that contains the camera poses for each of the spine that will be raycasted"
    )

    arg_parser.add_argument(
        "--path_blender_executable",
        required=True,
        dest="blender",
        help="Path to the blender executable"
    )

    args = arg_parser.parse_args()
    calling_blender = args.blender + '/blender'

    # get number of scans per model by checking how many camera poses are in the csv file
    # parse csv file
    with open(args.camera_poses, "r") as f:
        lines = f.readlines()
    num_scans_per_model = len(lines[0].split(";"))-2

    # raycast and obtain exr files
    print("Raycasting...")
    subprocess.run([calling_blender,  # call blender
                    '--background',
                    '--python', '../raycasting_scripts/render_depth.py',  # with the depth rendering script
                    args.list_paths_for_raycasting,
                    args.camera_poses
                    ])

    # process the exr files, generate depth images as well as the pcds
    print("Generating point clouds...")
    subprocess.run(['python',  # call python
                    '../raycasting_scripts/process_exr.py',  # with script that generates point clouds
                    args.list_paths_for_raycasting,
                    str(num_scans_per_model)  # number of scans per model
                    ])

    # rotate the pointclouds to get aligned with the initial mesh
    with open(args.list_paths_for_raycasting) as file:
        path_list = file.read().splitlines()

    for path in path_list:
        # initial spines pcds
        path_to_spine_rendering = os.path.join(os.path.dirname(path),"rendering")
        for dir in os.listdir(path_to_spine_rendering):
            if os.path.isdir(os.path.join(path_to_spine_rendering,dir)):
                path_pcds = os.path.join(path_to_spine_rendering, dir, "pcd")
                merge_pcds(path_pcds,os.path.join(path_to_spine_rendering,dir + ".pcd"))





    




