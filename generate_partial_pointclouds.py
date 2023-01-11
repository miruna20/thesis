import argparse
import subprocess
import os

# Credits to wenaouyan at https://github.com/wentaoyuan/pcn/tree/master/render

if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser(description="Scales a mesh to 0.01 of its original size and centers it")

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

    subprocess.run([calling_blender,  #call blender
                    '--background',
                    '--python', 'raycasting_scripts/render_depth.py', # with the depth rendering script
                  args.root_path_vertebrae, # vertebrae directory
                  args.txt_file, # list of vertebrae for which we want to generate point clouds
                  args.output_directory, # output directory
                  args.num_scans_per_model # number of scans per model
                   ])


    subprocess.run([ 'python3', # call python
                     'raycasting_scripts/process_exr.py', # with script that generates point clouds
                     args.txt_file, # list of vertebrae for which we want to generate point clouds
                     os.path.join(args.output_directory, 'intrinsics.txt'), # intrinsics file
                     args.output_directory, # output directory
                     args.num_scans_per_model # number of scans per model
                    ])



    # TODO rotate the point clouds that we obtain (?)







