import argparse
import os
import glob
import pymeshlab

def convert_pcd_to_mesh_meshlab(pcd_path):
    # reads ply
    # create a new MeshSet
    ms = pymeshlab.MeshSet()
    # load a new mesh in the MeshSet, and sets it as current mesh
    # the path of the mesh can be absolute or relative
    ms.load_new_mesh(pcd_path)

    # compute normals
    ms.compute_normal_for_point_clouds()

    # use poisson for surface generation
    ms.generate_surface_reconstruction_screened_poisson()

    ms.meshing_remove_unreferenced_vertices()
    ms.save_current_mesh(pcd_path.replace(".ply",".obj"))

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
    pcd_paths = sorted(glob.glob(os.path.join(args.root_path_pcds, "*.ply"), recursive=True))

    for pcd_path in pcd_paths:
        print("Converting: " + pcd_path)
        convert_pcd_to_mesh_meshlab(pcd_path, args.visualize)
