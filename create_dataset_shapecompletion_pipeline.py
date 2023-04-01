import argparse
import os
import subprocess

if __name__ == '__main__':
    """
    
    Pipeline steps:
    1. Make txt list with all spine ids of spines that contain lumbar vertebrae --> get_spines_lumbar_vertebrae.py
    ### For each of the spines in the list:
    2. Separate spine segmentation into individual vertebrae segmentation (only for lumbar vertebrae) --> separate_spine_into_vertebrae.py
    ### For each vertebra:
    3. Transform the vert segm(.nii.gz) into a mesh(.obj) --> convert_segmentation_into_mesh.py
    4. Scale the vert mesh and center it --> scale_and_center_mesh.py
    5. Create a list of vertebrae for the dataset --> get_list_of_vertebrae_in_folder.py
    6. Create partial point clouds through raycasting that resemble partial point clouds extracted from US --> generate_partial_pointclouds_from_vertebrae.py
    ### Put dataset together
    7. Combine partial and complete point clouds into one .h5 dataset which will be used by VRCNet for shape completion --> create_dataset_for_shape_completion.py
    
    """

    arg_parser = argparse.ArgumentParser(description="Generate dataset with complete and partial pointclouds from CT for shape completion")

    arg_parser.add_argument(
        "--root_path_spines",
        required=True,
        dest="root_path_spines",
        help="Root path to the vertebrae folders."
    )

    arg_parser.add_argument(
        "--list_spine_names",
        required=True,
        dest="txt_file_spines",
        help="File where the names of the lumbar vertebrae will be written"
    )

    arg_parser.add_argument(
        "--root_path_vertebrae",
        required=True,
        dest="root_path_vertebrae",
        help="Root path to the vertebrae folders."
    )

    arg_parser.add_argument(
        "--workspace_file_segm_to_mesh",
        required=True,
        dest="workspace_file_segm_to_mesh",
        help="ImFusion workspace files that has converts a segm in nii.gz to a mesh in obj"
    )

    arg_parser.add_argument(
        "--output_directory_raycasting",
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

    arg_parser.add_argument(
        "--list_vertebrae_names",
        required=True,
        dest="txt_file_vertebrae",
        help="Txt file where the file names will be saved"
    )

    arg_parser.add_argument(
        "--result_h5_file",
        required=True,
        dest="result_h5_file",
        help="Path to the h5 file where the dataset will be saved"
    )

    arg_parser.add_argument(
        "--nr_points_per_point_cloud",
        required=True,
        dest="nr_points_per_point_cloud",
        help="Number of points per point cloud. This number is used for the sampling technique."
    )

    arg_parser.add_argument(
        "--pipeline",
        nargs='+',
        default=['all'],
        help="Specify the steps of the pipeline that will be executed "
    )

    args = arg_parser.parse_args()

    root_path_spines = args.root_path_spines
    root_path_vertebrae = args.root_path_vertebrae

    spine_ids = args.txt_file_spines
    vert_ids = args.txt_file_vertebrae

    workspace_file_segm_to_mesh = args.workspace_file_segm_to_mesh

    # raycasting
    output_directory_raycasting = args.output_directory
    directory_partial_point_clouds = os.path.join(args.output_directory, 'pcd')
    path_blender_executable = args.blender
    num_scans_per_model = args.num_scans_per_model

    # h5 dataset
    result_h5_file = args.result_h5_file
    nr_points_per_point_cloud = args.nr_points_per_point_cloud

    # pipeline steps
    pipeline = args.pipeline

    if 'get_lumbar_spines' in pipeline or 'all' in pipeline:
        subprocess.run(['python', 'get_spines_lumbar_vertebrae.py',
                        '--root_path_spines', root_path_spines,
                        '--list_file_names', spine_ids])

    if 'separate_spine_into_vert' in pipeline or 'all' in pipeline:
        subprocess.run(['python', 'separate_spine_into_vertebrae.py',
                        '--list_file_names', spine_ids,
                        '--root_path_vertebrae', root_path_vertebrae,
                        '--root_path_spines', root_path_spines])

    if 'segm_to_mesh' in pipeline or 'all' in pipeline:
        subprocess.run(['python', 'convert_segmentation_into_mesh.py',
                        '--root_path_vertebrae', root_path_vertebrae,
                        '--list_file_names', spine_ids,
                        '--workspace_file_segm_to_mesh', workspace_file_segm_to_mesh])

    if 'scale_center_vert' in pipeline or 'all' in pipeline:
        subprocess.run(['python', 'scale_and_center_mesh.py',
                        '--root_path_vertebrae', root_path_vertebrae,
                        '--list_file_names',  spine_ids])

    if 'get_vertebrae_ids' in pipeline or 'all' in pipeline:
        subprocess.run(['python', 'get_list_of_vertebrae_in_folder.py',
                        '--root_path_vertebrae', root_path_vertebrae,
                        '--list_file_names', vert_ids
                        ])

    if 'get_partial_pcds' in pipeline or 'all' in pipeline:
        subprocess.run(['python', 'generate_partial_pointclouds_from_vertebrae.py',
                        '--root_path_vertebrae', root_path_vertebrae,
                        '--list_file_names', vert_ids,
                        '--output_directory', output_directory_raycasting,
                        '--path_blender_executable', path_blender_executable,
                        '--num_scans_per_model', num_scans_per_model
                        ])

    if 'create_dataseth5' in pipeline or 'all' in pipeline:
        subprocess.run(['python', 'create_dataset_for_shape_completion.py',
                        '--vertebrae_list', vert_ids,
                        '--root_path_vertebrae', root_path_vertebrae,
                        '--directory_partial_pcds', directory_partial_point_clouds,
                        '--result_h5_file', result_h5_file,
                        '--nr_partial_pcds_per_sample', num_scans_per_model,
                        '--nr_points_per_point_cloud', nr_points_per_point_cloud])