import numpy as np
import h5py
import open3d as o3d
import math
import argparse
import os
import glob

def convert_pcd_to_mesh(pcd_path, visualize=False):
    # read pcd with o3d
    pcd = o3d.io.read_point_cloud(pcd_path)

    # estimate the normals of the pcd
    pcd.estimate_normals()
    # pcd.orient_normals_consistent_tangent_plane(100)

    # try algo for surface reconstruction (e.g ball pivoting)
    # ball pivoting
    radii = [0.005, 0.01, 0.02, 0.04, 0.5]
    mesh_from_ball_pivoting = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))

    if(visualize):
        o3d.visualization.draw_geometries([mesh_from_ball_pivoting])

    o3d.io.write_triangle_mesh(pcd_path.replace(".pcd",".obj"),mesh_from_ball_pivoting)


if __name__ == "__main__":

    arg_parser = argparse.ArgumentParser(description="Convert a point cloud to a mesh")

    arg_parser.add_argument(
        "--root_path_pcds",
        required=True,
        dest="root_path_pcds",
        help="Root path of the pointclouds folders"
    )

    arg_parser.add_argument(
        "--visualize",
        action="store_true",
        help="Visualize the resulting mesh"
    )

    args = arg_parser.parse_args()

    # gather all of the pointclouds in the folder, transform each one into a mesh and save the result
    pcd_paths = sorted(glob.glob(os.path.join(args.root_path_pcds, "*.pcd"), recursive=True))

    #convert_pcd_to_mesh(os.path.join(args.root_path_pcds,"b'sub-verse502_verLev21_deform2'_4096_reconstruction.pcd"))
    for pcd_path in pcd_paths:
        convert_pcd_to_mesh(pcd_path, args.visualize)