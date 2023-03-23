import argparse
import subprocess
import os
import open3d as o3d
import numpy as np


# Credits to wenaouyan at https://github.com/wentaoyuan/pcn/tree/master/render
def rotate_pcd(path):
    # load point cloud
    partial_pointcloud = o3d.io.read_point_cloud(path)

    # rotate
    R = partial_pointcloud.get_rotation_matrix_from_xyz((-np.pi / 2, 0, 0))
    partial_pointcloud = partial_pointcloud.rotate(R, center=(0, 0, 0))

    # save
    path_to_save =path.replace(".pcd", "_rotated.pcd")
    o3d.io.write_point_cloud(path_to_save, partial_pointcloud)

def rotate_pcds(pcds_dir):
    pcds_files = os.listdir(pcds_dir)
    for file in pcds_files:
        rotate_pcd(os.path.join(pcds_dir,file))

if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser(
        description="Generate partial point clouds that resemble the pointclouds that would be obtained from the utlrasound")

    arg_parser.add_argument(
        "--root_path_spines",
        required=True,
        dest="root_path_spines",
        help="Root path of the vertebrae folders"
    )

    arg_parser.add_argument(
        "--list_file_names",
        required=True,
        dest="txt_file",
        help="Txt file that contains all vertebrae that have been scaled and centered"
    )

    arg_parser.add_argument(
        "--path_blender_executable",
        required=True,
        dest="blender",
        help="Path to the blender executable"
    )
    arg_parser.add_argument(
        "--num_scans_per_model",
        required=True,
        dest="num_scans_per_model",
        help="Number of scans per model"
    )
    args = arg_parser.parse_args()
    calling_blender = args.blender + '/blender'
    camera_poses = '[[0,0,1]]'

    # raycast and obtain exr files
    print("Raycasting...")
    subprocess.run([calling_blender,  # call blender
                    '--background',
                    '--python', 'raycasting_scripts/render_depth.py',  # with the depth rendering script
                    args.txt_file,
                    camera_poses
                    ])

    # process the exr files, generate depth images as well as the pcds
    print("Generating point clouds...")
    subprocess.run(['python',  # call python
                    'raycasting_scripts/process_exr.py',  # with script that generates point clouds
                    args.txt_file,  # list of vertebrae for which we want to generate point clouds
                    args.num_scans_per_model  # number of scans per model
                    ])

    # rotate the pointclouds to get aligned with the initial mesh
    with open(args.txt_file) as file:
        path_list = file.read().splitlines()

    path_pcds = os.path.join(os.path.dirname(path_list[0]), "rendering", "pcd")

    rotate_pcds(path_pcds)

