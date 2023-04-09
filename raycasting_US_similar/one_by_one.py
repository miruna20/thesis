import argparse
import ast
import os
import subprocess
from utils import misc
from time import process_time
if __name__ == '__main__':

    """
    Pipeline steps for one spine and it's corresponding vertebrae (can be previously deformed)
    1. Shift initial spine, merge shifted with original 
        => initial spine, shifted spine, merged spine 
    2. Raycast each of the above spines from 3 camera positions which will be above the spinous
    processes of L1, L2, L3, L4, L5. For each spine merge the 5 obtained pcds
        - for this we need the exact camera poses from which we will raycast
        => raycasted pcd initial spine, raycasted pcd shifted spine, raycasted pcd merged spine 
    3. Account for US artefacts by loading all 3 prev obtained pcds and removing the shadowing 
    obtained by overlapping of the shifted on the initial 
        => shadowed pcd
    """

    arg_parser = argparse.ArgumentParser(
        description="Generate dataset with complete and partial pointclouds from CT for shape completion")

    arg_parser.add_argument(
        "--root_paths_spines",
        required=True,
        dest="root_paths_spines",
        help="Path to root directory of all spines"
    )

    arg_parser.add_argument(
        "--root_paths_vertebrae",
        required=True,
        dest="root_paths_vertebrae",
        help="Path to root directory of all vertebrae"
    )

    arg_parser.add_argument(
        "--list_spines",
        required=True,
        dest="list_spines",
        help="Txt file that contains all spines"
    )

    arg_parser.add_argument(
        "--num_deform",
        required=True,
        dest="num_deform",
        help="Number of deformations for one spine"
    )
    arg_parser.add_argument(
        "--num_shifts",
        required=True,
        dest="num_shifts",
        help="Number of shifts for one spine"
    )
    arg_parser.add_argument(
        "--nr_points_per_point_cloud",
        required=True,
        dest="nr_points_per_point_cloud",
        help="Number of points that will be sampled both from the partial point cloud and from the complete mesh"
    )
    arg_parser.add_argument(
        "--pipeline",
        nargs='+',
        default=['all'],
        help="Specify the steps of the pipeline that will be executed "
    )

    args = arg_parser.parse_args()

    # iterates over the file with spines and writes in a temp one each spine one by one
    save_to_temp = os.path.join(args.root_paths_vertebrae,"list_spines_temp_for_processing.txt")
    # read a list of the spines that we want to process
    with open(args.list_spines) as file:
        spines_ids = file.read().splitlines()

    for spine_id in spines_ids:
        list = open(save_to_temp, "w")
        list.write(spine_id + "\n")
        print("Processing: " + str(spine_id))
        list.close()

        # call the pipeline
        subprocess.run(['python', 'raycast_US_similar_pipeline.py',
                        '--root_paths_spines', args.root_paths_spines,
                        '--root_paths_vertebrae', args.root_paths_vertebrae,
                        '--list_spines', save_to_temp,
                        '--num_deform', args.num_deform,
                        '--num_shifts', args.num_shifts,
                        '--nr_points_per_point_cloud',args.nr_points_per_point_cloud])


