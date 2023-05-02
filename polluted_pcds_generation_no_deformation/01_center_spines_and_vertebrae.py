import os
import sys
import argparse
import glob
import open3d as o3d
from pathlib import Path
from utils import namings


def center(spine_path, vert_paths):
    # read mesh
    spine = o3d.io.read_triangle_mesh(spine_path)

    # get center
    centerSpine = spine.get_center()

    # move to center
    vertsSpine = spine.vertices - centerSpine
    spine.vertices = o3d.utility.Vector3dVector(vertsSpine)

    # save mesh
    o3d.io.write_triangle_mesh(spine_path.replace(".obj", "_centered.obj"), spine)

    # move also all of the vertebrae meshes to the center
    for vert_path in vert_paths:
        curr_vert = o3d.io.read_triangle_mesh(vert_path)
        vertices_curr_vert = curr_vert.vertices - centerSpine
        curr_vert.vertices = o3d.utility.Vector3dVector(vertices_curr_vert)
        o3d.io.write_triangle_mesh(vert_path.replace(".obj", "_centered.obj"), curr_vert)


def center_all(txt_file, root_path_spines, root_path_vertebrae):
    # iterate over spine IDS
    with open(txt_file) as file:
        spine_ids = [line.strip() for line in file]

    for spine_id in spine_ids:
        print("Centering the spine and vertebrae of: " + str(spine_id))

        # get the paths to the spine with current spine_id and deformation number
        spine_mesh_path = namings.get_path_lumbar_spine_initial(root_path_spines, spine_id)
        if (not os.path.isfile(spine_mesh_path)):
            print("No mesh found for spine %s in path %s" % (spine_id,spine_mesh_path), file=sys.stderr)

        vertebrae_mesh_paths = namings.get_paths_vertebrae(root_path_vertebrae, spine_id)

        center(spine_mesh_path, vertebrae_mesh_paths)


if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser(description="Center mesh")

    arg_parser.add_argument(
        "--root_path_spines",
        required=True,
        dest="root_path_spines",
        help="Root path of the spine folders"
    )

    arg_parser.add_argument(
        "--root_path_vertebrae",
        required=True,
        dest="root_path_vertebrae",
        help="Root path of the spine folders"
    )

    arg_parser.add_argument(
        "--list_file_names",
        required=True,
        dest="txt_file",
        help="Txt file that contains all spines that contain all lumbar vertebrae"
    )

    args = arg_parser.parse_args()

    center_all(txt_file=args.txt_file, root_path_vertebrae=args.root_path_vertebrae,
               root_path_spines=args.root_path_spines)
