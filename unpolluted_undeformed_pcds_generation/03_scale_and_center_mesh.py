import os
import sys
import argparse
import glob
import open3d as o3d
import numpy as np

def find_spine_folder_and_vertebrae_files(spine_identifier, vertebrae_folder,spine_folder):
    spine_folder_path = None
    vertebrae_files = []

    for folder_name in os.listdir(spine_folder):
        if spine_identifier in folder_name:
            spine_folder_path = os.path.join(spine_folder, folder_name)
            break

    if spine_folder_path is not None:
        for file in os.listdir(spine_folder_path):
            if file.endswith('.obj'):
                spine_path = os.path.join(spine_folder_path, file)


    # Get immediate subdirectories
    subdirs = [os.path.join(vertebrae_folder, d) for d in os.listdir(vertebrae_folder) if os.path.isdir(os.path.join(vertebrae_folder, d))]
    for subdir in subdirs:
        # Check if the current directory contains the identifier
        if spine_identifier in subdir:
            for file in os.listdir(subdir):
                if file.endswith(".obj"):
                    vertebrae_files.append(os.path.join(subdir, file))
    return spine_path, sorted(vertebrae_files)

def center_and_scale_spines(spine_identifier, root_path_vertebrae, root_path_spines,scale=0.01):
    # Find spine folder and vertebrae files based on identifier
    spine_folder, vertebrae_files = find_spine_folder_and_vertebrae_files(spine_identifier, root_path_vertebrae, root_path_spines)
    unique_identifier = "*/**/**" + str(spine_id) + "*labelmap*.obj"
    labelmaps_files = sorted(glob.glob(os.path.join(root_path_vertebrae, unique_identifier), recursive=True))

    if spine_folder is None:
        print("Spine folder not found for identifier:", spine_identifier)
        return
    if not vertebrae_files:
        print("No vertebrae files found for identifier:", spine_identifier)
        return

    # Load spine mesh
    spine_mesh = o3d.io.read_triangle_mesh(spine_folder)  # Assuming first file is spine mesh

    # Load vertebrae meshes
    vertebrae_meshes = [o3d.io.read_triangle_mesh(file) for file in vertebrae_files]
    labelmap_meshes = [o3d.io.read_triangle_mesh(labelmap) for labelmap in labelmaps_files]


    # Compute centroid of spine
    spine_centroid = spine_mesh.get_center()
    spine_mesh.translate(-spine_centroid)
    spine_mesh.scale(scale, center=np.asarray([0, 0, 0]))

    # Calculate translation and apply to vertebrae
    for idx,vertebra_mesh in enumerate(vertebrae_meshes):
        vertebra_mesh.translate(-spine_centroid)
        vertebra_mesh.scale(scale, center=np.asarray([0, 0, 0]))
        o3d.io.write_triangle_mesh(vertebrae_files[idx].replace(".obj", "_centered_scaled.obj"), vertebra_mesh)

    for idx, labelmap_mesh in enumerate(labelmap_meshes):
        labelmap_mesh.translate(-spine_centroid)
        labelmap_mesh.scale(scale, center=np.asarray([0, 0, 0]))
        o3d.io.write_triangle_mesh(labelmaps_files[idx].replace(".obj", "_centered_scaled.obj"), labelmap_mesh)


    # Save centered spine and vertebrae meshes
    o3d.io.write_triangle_mesh(spine_folder.replace(".obj", "_centered_scaled.obj"), spine_mesh)


if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser(description="Scales a mesh to 0.01 of its original size and centers it")

    arg_parser.add_argument(
        "--root_path_vertebrae",
        required=True,
        dest="root_path_vertebrae",
        help="Root path of the vertebrae folders"
    )
    arg_parser.add_argument(
        "--root_path_spines",
        required=True,
        dest="root_path_spines",
        help="Root path of the spines folders"
    )

    arg_parser.add_argument(
        "--txt_file",
        required=True,
        dest="txt_file",
        help="Txt file that contains all spines that contain all lumbar vertebrae"
    )

    args = arg_parser.parse_args()

    # iterate over spine IDS
    with open(args.txt_file) as file:
        spine_ids = [line.strip() for line in file]


    for spine_id in spine_ids:

        # get the paths for the segmentations of all vertebrae belonging to this spine
        center_and_scale_spines(spine_id, args.root_path_vertebrae,args.root_path_spines)