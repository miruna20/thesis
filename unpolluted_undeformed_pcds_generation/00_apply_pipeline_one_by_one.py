import argparse
import ast
import os
import subprocess
import munch as munch
import yaml

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

    arg_parser.add_argument('-c', '--config', help='path to config file', required=True)

    arg = arg_parser.parse_args()
    config_path = arg.config
    args = munch.munchify(yaml.safe_load(open(config_path)))

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

        subprocess.call(['python', '00_unpolluted_pcds_pipeline.py',
                        '--root_paths_spines', args.root_paths_spines,
                        '--root_paths_vertebrae', args.root_paths_vertebrae,
                        '--list_spines', save_to_temp,
                        '--num_deform', str(args.num_deform),
                        '--num_shifts', str(args.num_shifts),
                        '--nr_points_per_point_cloud',str(args.nr_points_per_point_cloud),
                        #'--pipeline', 'scale_down_mesh shift_and_merge get_camera_poses raycast account_US_shadows separate_spine_pc_into_vert'])
                        '--pipeline', ''])



    # after everything is done for all spines, call the create_h5_dataset once
    subprocess.run(['python', '00_unpolluted_pcds_pipeline.py',
                    '--root_paths_spines', args.root_paths_spines,
                    '--root_paths_vertebrae', args.root_paths_vertebrae,
                    '--list_spines', args.list_spines,
                    '--num_deform', str(args.num_deform),
                    '--num_shifts', str(args.num_shifts),
                    '--nr_points_per_point_cloud', str(args.nr_points_per_point_cloud),
                    '--pipeline', 'create_h5_dataset'])