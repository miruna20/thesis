import argparse
import subprocess
import os
import glob
import open3d as o3d
import numpy as np

# Credits to wenaouyan at https://github.com/wentaoyuan/pcn/tree/master/render

if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser(description="Generate partial point clouds that resemble the pointclouds that would be obtained from the utlrasound")

    arg_parser.add_argument(
        "--root_path_vertebrae",
        required=True,
        dest="root_path_vertebrae",
        help="Root path of the vertebrae folders"
    )

    arg_parser.add_argument(
        "--list_file_names",
        required=True,
        dest="txt_file",
        help="Txt file that contains all vertebrae that have been scaled and centered"
    )

    arg_parser.add_argument(
        "--output_directory",
        required=True,
        dest="output_directory",
        help="Directory where the results will be saved"
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
        help="Number of partial point clouds that will be generated for each model"
    )

    args = arg_parser.parse_args()
    calling_blender = args.blender + '/blender'

    print("Raycasting...")
    subprocess.run([calling_blender,  #call blender
                    '--background',
                    '--python', 'raycasting_scripts/render_depth.py', # with the depth rendering script
                  args.root_path_vertebrae, # vertebrae directory
                  args.txt_file, # list of vertebrae for which we want to generate point clouds
                  args.output_directory, # output directory
                  args.num_scans_per_model # number of scans per model
                   ])

    print("Generating point clouds...")
    subprocess.run([ 'python3', # call python
                     'raycasting_scripts/process_exr.py', # with script that generates point clouds
                     args.txt_file, # list of vertebrae for which we want to generate point clouds
                     os.path.join(args.output_directory, 'intrinsics.txt'), # intrinsics file
                     args.output_directory, # output directory
                     args.num_scans_per_model # number of scans per model
                    ])

    # rotate resulting point clouds to align with the initial mesh
    print("Rotating resulting point clouds to align with the initial mesh...")
    # iterate over vert IDS
    with open(args.txt_file) as file:
        vert_ids = [line.strip() for line in file]

    partial_pcds_directory = os.path.join(args.output_directory, "pcd")

    for vert_id in vert_ids:

        vert_directory = os.path.join(partial_pcds_directory,vert_id)

        directory_rotated_pcds = os.path.join(vert_directory, "rotated")

        if (not os.path.exists(directory_rotated_pcds)):
            os.mkdir(directory_rotated_pcds)

        # iterate over the pointclouds in the directory, load, rotate, save
        for file in sorted(glob.glob(os.path.join(vert_directory,"*.pcd"))):
           # load
           partial_pointcloud = o3d.io.read_point_cloud(file)

           # rotate
           R = partial_pointcloud.get_rotation_matrix_from_xyz((-np.pi/2, 0, 0))
           partial_pointcloud = partial_pointcloud.rotate(R, center=(0, 0, 0))

           # save
           path_to_save = os.path.join(directory_rotated_pcds, os.path.basename(file).replace(".pcd","_rotated.pcd"))
           o3d.io.write_point_cloud(path_to_save, partial_pointcloud)











