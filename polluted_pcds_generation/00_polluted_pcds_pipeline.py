import argparse
import os
import subprocess
from utils import misc

if __name__ == '__main__':

    """
    Prerequisites for this pipeline: 
    1. The folder structure is the following 
    - root_path_spines directory:
        <root_path_spines>/<spine_id>/ folders are already created 
    
    - root_path_vertebrae:
        <root_path_vertebrae>/<spine_id>/<spine_id>*_msh.obj --> mesh files of individual vertebrae are used for deformation
        --> to separate spine segmentations into vertebrae segmentations and transform segmentation to mesh check 
            - "https://github.com/miruna20/thesis/blob/main/separate_spine_into_vertebrae.py"
            - "https://github.com/miruna20/thesis/blob/main/convert_segmentation_into_mesh.py"
              
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
    print(args)
    pipeline = args.pipeline

    # cover the case in which we call this pipeline from another script
    #TODO does this always work?
    pipeline.extend(pipeline[0].split())
    root_paths_spines = args.root_paths_spines
    root_paths_vertebrae = args.root_paths_vertebrae

    num_deform = args.num_deform
    num_shifts = args.num_shifts
    nr_points_per_point_cloud = args.nr_points_per_point_cloud

    list_spines = args.list_spines
    list_paths_for_raycasting = os.path.join(root_paths_vertebrae, "list_spines_for_raycasting.txt")
    list_paths_spines_and_vert_for_scaling = os.path.join(root_paths_vertebrae, "list_spines_and_vert_for_scaling.txt")
    list_paths_spines = os.path.join(root_paths_vertebrae, "list_spines_for_shift_and_merge.txt")
    list_paths_spines_and_vert_for_camera_poses_generation = os.path.join(root_paths_vertebrae,
                                                                          "list_spines_and_vert_for_camera_pose_generation.txt")

    list_paths_vertebrae = os.path.join(root_paths_vertebrae, "list_vertebrae.txt")
    path_to_save_camera_poses_csv = os.path.join(root_paths_vertebrae, "camera_poses.csv")

    result_h5_file = os.path.join(root_paths_vertebrae, "dataset.h5")
    path_blender_executable = "/home/miruna20/Documents/Thesis/Code/Preprocessing/blender-2.79-linux-glibc219-x86_64"

    if 'scale_down_mesh' in pipeline or 'all' in pipeline:
        misc.create_list_all_deformed_vert_and_spines_from_spineid(list_spines, list_paths_spines_and_vert_for_scaling,
                                                                   root_paths_vertebrae, root_paths_spines, num_deform)
        subprocess.run(['python', '01_scale_down_mesh.py',
                        '--list_mesh_paths', list_paths_spines_and_vert_for_scaling])

    if 'shift_and_merge' in pipeline or 'all' in pipeline:
        misc.create_list_all_deformed_scaled_spines_from_spineid(list_spines, list_paths_spines, root_paths_spines,
                                                                 num_deform)
        subprocess.run(['python', '02_shift_and_merge_spine.py',
                        '--list_paths_spine', list_paths_spines,
                        '--num_shifts',num_shifts])

    if 'get_camera_poses' in pipeline or 'all' in pipeline:
        misc.create_list_all_deformed_scaled_vert_and_spines_from_spineid(list_spines,
                                                                          list_paths_spines_and_vert_for_camera_poses_generation,
                                                                          root_paths_vertebrae, root_paths_spines,
                                                                          num_deform)
        subprocess.run(['python', '03_generate_camera_poses.py',
                        '--list_spines_and_corresp_vertebrae', list_paths_spines_and_vert_for_camera_poses_generation,
                        '--path_to_save_camera_poses_csv', path_to_save_camera_poses_csv
                        ])

    if 'raycast' in pipeline or 'all' in pipeline:
        # we create a txt file with all of the paths for further raycasting
        misc.create_list_init_shifted_merged_deformed_for_raycasting(spines_root=root_paths_spines,
                                                                     list_spines=list_spines,
                                                                     save_to=list_paths_for_raycasting,
                                                                     num_deform=num_deform,
                                                                     num_shifts=num_shifts)
        subprocess.run(['python', '04_generate_partial_pointclouds_from_spine.py',
                        '--list_paths_for_raycasting', list_paths_for_raycasting,
                        '--camera_poses', path_to_save_camera_poses_csv,
                        '--path_blender_executable', path_blender_executable])

        misc.delete_paths(list_paths_for_raycasting)
    if 'account_US_shadows' in pipeline or 'all' in pipeline:
        subprocess.run(['python', '05_accounts_US_shadows.py',
                        '--root_paths_spines', root_paths_spines,
                        '--list_spines', list_spines,
                        '--num_deform', num_deform,
                        ])

    if 'separate_spine_pc_into_vert' in pipeline or 'all' in pipeline:
        subprocess.run(['python', '06_separate_spine_pc_into_vertebrae.py',
                        '--list_file_names', list_spines,
                        '--root_path_vertebrae', root_paths_vertebrae,
                        '--root_path_spines', root_paths_spines,
                        '--nr_deform_per_spine', num_deform,
                        '--nr_shift_per_spine',num_shifts
                        ])

    if 'create_h5_dataset' in pipeline or 'all' in pipeline:
        # create the list of vertebrae that will be used to create the h5 dataset

        subprocess.run(['python', '07_get_list_vertebrae_in_folders.py',
                        '--root_path_vertebrae', root_paths_vertebrae,
                        '--vert_list_to_save', list_paths_vertebrae,
                        '--list_file_names_spines', list_spines,
                        '--nr_deform_per_spine', num_deform,
                        '--num_shifts',num_shifts
                        ])

        subprocess.run(['python', '07_create_dataset_for_shape_completion.py',
                        '--vertebrae_list', list_paths_vertebrae,
                        '--root_path_vertebrae', root_paths_vertebrae,
                        '--result_h5_file', result_h5_file,
                        '--nr_deform_per_sample', num_deform,
                        '--nr_points_per_point_cloud', nr_points_per_point_cloud,
                        '--num_shifts', num_shifts
                        #'--visualize_vertebrae'
                        ])


