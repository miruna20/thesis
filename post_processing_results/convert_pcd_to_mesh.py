import argparse
import os
import glob
import pymeshlab
import open3d as o3d

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

def convert_all_pcds_from_one_folder(path_folder):
    # gather all of the pointclouds in the folder, transform each one into a mesh and save the result
    pcd_paths = sorted(glob.glob(os.path.join(path_folder, "*.ply"), recursive=True))

    for pcd_path in pcd_paths:
        print("Converting: " + pcd_path)
        convert_pcd_to_mesh_meshlab(pcd_path)

if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser(description="Convert a point cloud to a mesh")

    arg_parser.add_argument(
        "--root_path_pcds",
        required=True,
        dest="root_path_pcds",
        help="Root path of the pointclouds folders"
    )

    args = arg_parser.parse_args()

    convert_all_pcds_from_one_folder(args.root_path_pcds)

    """
    path = "/home/miruna20/Desktop/pointcloud500.ply"
    pcd = o3d.io.read_point_cloud(path)
    filtered_pcd, _ = pcd.remove_radius_outlier(1, 0.03)
    o3d.io.write_point_cloud(path,filtered_pcd)
    #convert_pcd_to_mesh_meshlab()
    """
