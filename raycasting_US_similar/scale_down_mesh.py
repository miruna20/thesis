import os
import sys
import argparse
import glob
import open3d as o3d
import numpy as np

def scale(mesh_path,scale=0.01):
    """
    Reads a mesh and scales it down to a certain scale
    """
    # read mesh
    mesh = o3d.io.read_triangle_mesh(mesh_path)

    # scale down
    mesh.scale(scale,center=np.asarray([0,0,0]))

    # save mesh
    o3d.io.write_triangle_mesh(mesh_path.replace(".obj", "_scaled.obj"), mesh)

def scale_all(list):
    """
    Scales all meshes in a list of paths
    """
    # iterate over spine IDS
    with open(list) as file:
        mesh_paths = [line.strip() for line in file]

    for path in mesh_paths:
        print("Scaling " + str(path))
        scale(path)

if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser(description="Scales a mesh to 0.01 of its original size")

    arg_parser.add_argument(
        "--list_mesh_paths",
        required=True,
        dest="list_mesh_paths",
        help="Txt file with paths to the meshes that will be scaled"
    )

    args = arg_parser.parse_args()
    scale_all(args.list_mesh_paths)



