import os
import sys
import argparse
import glob
import open3d as o3d


def merge_meshes(mesh_files, root_path_spines, spine_id):
    unique_identifier = "*/**" + str(spine_id)  + "*.nii.gz"
    path_spine = sorted(glob.glob(os.path.join(root_path_spines, unique_identifier), recursive=True))[0]

    merged_mesh = o3d.geometry.TriangleMesh()

    for file in mesh_files:
        mesh = o3d.io.read_triangle_mesh(file)
        merged_mesh += mesh
    folder = os.path.dirname(path_spine)
    o3d.io.write_triangle_mesh(os.path.join(folder, spine_id + "_lumbar_msh.obj"), merged_mesh)


def convert_segmentation_vertebrae_into_mesh(root_path_vertebrae,spine_id,workspace_file_segm_to_mesh):
    # get the paths for the segmentations of all vertebrae belonging to this spine
    unique_identifier = "*/**" + str(spine_id)  + "*.nii.gz"
    vert_segm_paths = sorted(glob.glob(os.path.join(root_path_vertebrae, unique_identifier), recursive=True))
    vert_mesh_paths = []
    for vert_segm_path in vert_segm_paths:
        arguments_imfusion = ""
        for p in placeholders:

            if p == 'PathToFile':
                value = vert_segm_path

            if p == 'Name':
                vert_segm_name = os.path.basename(vert_segm_path)
                value = vert_segm_name[:vert_segm_name.find('.nii.gz')]

            if p == 'PathToSave':
                value = vert_segm_path.replace('.nii.gz', '_msh.obj')
                vert_mesh_paths.append(value)
            arguments_imfusion += p + "=" + value + " "

        print('ARGUMENTS: ', arguments_imfusion)
        os.system("ImFusionConsole" + " " + workspace_file_segm_to_mesh + " " + arguments_imfusion)
        print('################################################### ')

    return vert_mesh_paths

def convert_labelmaps_into_meshes(root_path_vertebrae, spine_id,workspace_file_segm_to_mesh):
    unique_identifier = "*/**/**" + str(spine_id) + "*labelmap*.nii.gz"
    vert_segm_paths = sorted(glob.glob(os.path.join(root_path_vertebrae, unique_identifier), recursive=True))
    vert_mesh_paths = []
    for vert_segm_path in vert_segm_paths:
        arguments_imfusion = ""
        for p in placeholders:

            if p == 'PathToFile':
                value = vert_segm_path

            if p == 'Name':
                vert_segm_name = os.path.basename(vert_segm_path)
                value = vert_segm_name[:vert_segm_name.find('.nii.gz')]

            if p == 'PathToSave':
                value = vert_segm_path.replace('.nii.gz', '_msh.obj')
                vert_mesh_paths.append(value)
            arguments_imfusion += p + "=" + value + " "

        print('ARGUMENTS: ', arguments_imfusion)
        os.system("ImFusionConsole" + " " + workspace_file_segm_to_mesh + " " + arguments_imfusion)
        print('################################################### ')

if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser(description="Convert a segmentation of type .nii.gz into a mesh of type obj with ImFusion workspace")

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
        "--list_file_names",
        required=True,
        dest="txt_file",
        help="Txt file that contains all spines that contain all lumbar vertebrae"
    )

    arg_parser.add_argument(
        "--workspace_file_segm_to_mesh",
        required=True,
        dest="workspace_file_segm_to_mesh",
        help="ImFusion workspace files that has converts a segm in nii.gz to a mesh in obj"
    )

    args = arg_parser.parse_args()

    # iterate over spine IDS
    with open(args.txt_file) as file:
        spine_ids = [line.strip() for line in file]

    placeholders = ['Name', 'PathToFile', 'PathToSave']
    for spine_id in spine_ids:
        vert_mesh_paths= convert_segmentation_vertebrae_into_mesh(args.root_path_vertebrae,spine_id,args.workspace_file_segm_to_mesh)
        convert_labelmaps_into_meshes(args.root_path_vertebrae, spine_id,args.workspace_file_segm_to_mesh )
        merge_meshes(vert_mesh_paths,args.root_path_spines,spine_id)





