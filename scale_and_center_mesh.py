import os
import sys
import argparse
import glob
import open3d as o3d

def scale_and_center(vert_path):
    # read mesh
    vert = o3d.io.read_triangle_mesh(vert_path)

    # get center
    centerVertebra = vert.get_center()

    # scale to 0.01 of the initial size
    vert.scale(0.01, center=centerVertebra)
    # move to center
    vertsVertebra = vert.vertices - centerVertebra
    vert.vertices = o3d.utility.Vector3dVector(vertsVertebra)

    # save mesh
    o3d.io.write_triangle_mesh(vert_path.replace("_msh.obj", "_msh_scaled_centered.obj"), vert)



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
        help="Txt file that contains all spines that contain all lumbar vertebrae"
    )

    args = arg_parser.parse_args()

    # iterate over spine IDS
    with open(args.txt_file) as file:
        spine_ids = [line.strip() for line in file]


    for spine_id in spine_ids:

        # get the paths for the segmentations of all vertebrae belonging to this spine
        unique_identifier = "*/**" + str(spine_id) + "*_msh.obj"
        vert_mesh_paths = sorted(glob.glob(os.path.join(args.root_path_vertebrae, unique_identifier), recursive=True))

        for vert_mesh_path in vert_mesh_paths:
            print("Scaling and centering " + str(vert_mesh_path))
            scale_and_center(vert_mesh_path)

