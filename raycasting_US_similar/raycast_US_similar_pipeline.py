import argparse
import ast
import os
import subprocess
from utils import misc

def parse_camera_poses_from_string(camera_poses_string):
    print("Camera poses string: " + str(camera_poses_string))
    camera_poses = ast.literal_eval(camera_poses_string)

    new_camera_poses_as_list = []
    for camera_pose in camera_poses:
        new_camera_poses_as_list.append(ast.literal_eval(str(camera_pose[0])))

    return new_camera_poses_as_list

if __name__ == '__main__':

    """
    Pipeline steps for one spine and it's corresponding vertebrae (can be previously deformed)
    1. Shift initial spine, merge shifted with original 
        => initial spine, shifted spine, merged spine 
    2. Raycast each of the above spines from 3 camera positions which will be above the spinous
    processes of L2, L3, L4. For each spine merge the 3 obtained pcds
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
        "--pipeline",
        nargs='+',
        default=['all'],
        help="Specify the steps of the pipeline that will be executed "
    )

    args = arg_parser.parse_args()

    pipeline = args.pipeline
    root_paths_spines = args.root_paths_spines
    root_paths_vertebrae = args.root_paths_vertebrae

    num_deform = args.num_deform

    list_spines = args.list_spines
    list_paths_for_raycasting = os.path.join(root_paths_vertebrae, "list_spines_for_raycasting.txt")
    list_paths_spines_and_vert_for_scaling = os.path.join(root_paths_vertebrae,"list_spines_and_vert_for_scaling.txt")
    list_paths_spines = os.path.join(root_paths_vertebrae,"list_spines_for_shift_and_merge.txt")
    list_paths_spines_and_vert_for_camera_poses_generation = os.path.join(root_paths_vertebrae,"list_spines_and_vert_for_camera_pose_generation.txt")
    path_to_save_camera_poses_csv = os.path.join(root_paths_vertebrae, "camera_poses.csv")

    path_blender_executable = "/home/miruna20/Documents/Thesis/Code/Preprocessing/blender-2.79-linux-glibc219-x86_64"


    if 'scale_down_mesh' in pipeline or 'all' in pipeline:
        misc.create_list_all_deformed_vert_and_spines_from_spineid(list_spines, list_paths_spines_and_vert_for_scaling,
                                                                   root_paths_vertebrae, root_paths_spines, num_deform)
        subprocess.run(['python', 'scale_down_mesh.py',
                        '--list_mesh_paths', list_paths_spines_and_vert_for_scaling])

    if 'shift_and_merge' in pipeline or 'all' in pipeline:
        misc.create_list_all_deformed_scaled_spines_from_spineid(list_spines,list_paths_spines, root_paths_spines,num_deform)
        subprocess.run(['python', 'shift_and_merge_spine.py',
                        '--list_paths_spine', list_paths_spines])

    if 'get_camera_poses' in pipeline or 'all' in pipeline:
        misc.create_list_all_deformed_scaled_vert_and_spines_from_spineid(list_spines,list_paths_spines_and_vert_for_camera_poses_generation,root_paths_vertebrae,root_paths_spines,num_deform)
        subprocess.run(['python', 'generate_camera_poses.py',
                        '--list_spines_and_corresp_vertebrae',list_paths_spines_and_vert_for_camera_poses_generation,
                        '--path_to_save_camera_poses_csv', path_to_save_camera_poses_csv
                       ])

    if 'raycast' in pipeline or 'all' in pipeline:
        # we create a txt file with all of the paths for further raycasting
        misc.create_list_init_shifted_merged_deformed_for_raycasting(spines_root=root_paths_spines,
                                                           list_spines=list_spines,
                                                            save_to=list_paths_for_raycasting,
                                                            num_deform=num_deform)
        subprocess.run(['python', 'generate_partial_pointclouds_from_spine.py',
                        '--list_paths_for_raycasting', list_paths_for_raycasting,
                        '--camera_poses', path_to_save_camera_poses_csv,
                        '--path_blender_executable', path_blender_executable])

    if 'account_US_shadows' in pipeline or 'all' in pipeline:
        subprocess.run(['python', 'accounts_US_shadows.py',
                        '--root_paths_spines',root_paths_spines,
                        '--list_spines',list_spines,
                        '--num_deform', num_deform])
