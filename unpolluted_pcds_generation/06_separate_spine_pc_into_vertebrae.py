import math
import argparse
import open3d as o3d
import numpy as np
import glob
import os
import sys
from utils import namings
import trimesh

# Algorithm:
# 1. load the centered scaled pcd of the spine and the centered vertebrae
# 3. for each individual vertebra from this spine before merging together
    # get the bounding box
    # increase the box size slightly in x direction
    # save as pointcloud all of the points within this bounding box

def load_centered_spine_pcd(root_path_spine, spine_id, deform, shift):

    path_root_shifts = os.path.join(root_path_spine,spine_id,"shifts", namings.get_name_spine_lumbar_mesh_deformed_scaled_centered(spine_id,deform).split(".")[0])
    path_shifts = sorted(os.listdir(path_root_shifts))
    path_spine = os.path.join(path_root_shifts,path_shifts[shift],"account_for_shadow.pcd")
    print("Loading" + str(path_spine))
    spine_pcd = o3d.io.read_point_cloud(path_spine)
    return spine_pcd, path_shifts

def load_centered_scaled_vertebrae(root_path_vertebrae, spine_id, deform):
    paths_vertebrae = namings.get_paths_one_deformation_scaled_vertebrae(root_path_vertebrae,spine_id,deform)

    if (len(paths_vertebrae) != 5):
        print("More or less than 5 vertebrae found for " + str(spine_id), file=sys.stderr)

    vertebrae_meshes = []
    for path_vertebra in paths_vertebrae:
        vertebrae_meshes.append(o3d.io.read_triangle_mesh(path_vertebra))

    return vertebrae_meshes, paths_vertebrae

def load_pcd(root_path_spine, spine_id, deform):
    look_for = "**/*" + str(spine_id) + "*force*" + str(deform) + '*.pcd'
    path_pcd = glob.glob(os.path.join(root_path_spine, look_for), recursive=True)

    if (len(path_pcd) != 1):
        print("More than 1 or no pcd found for spine " + str(spine_id) + "and deformation" + str(deform),
              file=sys.stderr)

    pcd = o3d.io.read_point_cloud(path_pcd[0])

    return pcd

def separate_US_pointcloud_into_vertebrae(root_path_spine, root_path_vertebrae, spine_id, deform, shift, visualize=False):
    print("Separating US point cloud from: " + str(spine_id) + " and deformation " + str(deform))

    #  load the centered, scaled spine pcd
    spine_pcd, path_shifts = load_centered_spine_pcd(root_path_spine, spine_id, deform, shift)

    #  load vertebrae that are also scaled and centered
    vertebrae_meshes, vertebrae_paths = load_centered_scaled_vertebrae(root_path_vertebrae, spine_id, deform)

    pcd_meshes = []
    # create a list with pcds of the meshes
    for vert_ind in range(5):
        pcd_mesh = vertebrae_meshes[vert_ind].sample_points_uniformly(number_of_points=10000)
        pcd_meshes.append(pcd_mesh)

    # unite the pcd from the spine with the pcds from the meshes
    pcd_combined = o3d.geometry.PointCloud()
    points_combined = np.concatenate((np.asarray(spine_pcd.points), np.asarray(pcd_meshes[0].points), np.asarray(pcd_meshes[1].points), np.asarray(pcd_meshes[2].points), np.asarray(pcd_meshes[3].points), np.asarray(pcd_meshes[4].points)))
    pcd_combined.points = o3d.utility.Vector3dVector(points_combined)
    pcd_tree = o3d.geometry.KDTreeFlann(pcd_combined)

    nr_points_in_spine = np.asarray(spine_pcd.points).shape[0]
    shapes = [np.asarray(pcd_mesh.points).shape[0] for pcd_mesh in pcd_meshes]

    # iterate only once over the spine pcd and check which points belong to which vertebra
    clean_points_all_vert= [[],[],[],[],[]]
    for idx_point_in_spine_pcd in range(0, nr_points_in_spine):
        curr_point = pcd_combined.points[idx_point_in_spine_pcd]

        [_, idx_neighbors, _] = pcd_tree.search_radius_vector_3d(curr_point, 0.01)
        index_closest_point = next(filter(lambda index: index >= np.asarray(spine_pcd.points).shape[0], idx_neighbors),
                                   None)

        # verify to which vertebra this point belongs
        if (index_closest_point != None):
            # it belongs to the first vertebra
            if nr_points_in_spine < index_closest_point < nr_points_in_spine + shapes[0]:
                clean_points_all_vert[0].append(curr_point)
            elif nr_points_in_spine + shapes[0] < index_closest_point < nr_points_in_spine + shapes[0] + shapes[1]:
                clean_points_all_vert[1].append(curr_point)
            elif nr_points_in_spine + shapes[0] + shapes[1] < index_closest_point < nr_points_in_spine + shapes[0] + shapes[1] + shapes[2]:
                clean_points_all_vert[2].append(curr_point)
            elif nr_points_in_spine + shapes[0] + shapes[1] + shapes[2] < index_closest_point < nr_points_in_spine + shapes[0] + shapes[1] + shapes[2] + shapes[3]:
                clean_points_all_vert[3].append(curr_point)
            elif nr_points_in_spine + shapes[0] + shapes[1] + shapes[2] + shapes[3] < index_closest_point < nr_points_in_spine + shapes[0] + shapes[1] + shapes[2] + shapes[3] + shapes[4]:
                clean_points_all_vert[4].append(curr_point)

    for vert_ind in range(5):
        curr_clean_points = clean_points_all_vert[vert_ind]

        # create point cloud from clean set of points
        curr_vert_pcd = o3d.geometry.PointCloud()
        curr_vert_pcd.points = o3d.utility.Vector3dVector(np.asarray(curr_clean_points))

        # get output dir
        print("Writing the clean pcd for " + vertebrae_paths[vert_ind])
        root_folder_vert = os.path.dirname(vertebrae_paths[vert_ind])
        vert_name = os.path.basename(vertebrae_paths[vert_ind])
        to_save_dir = os.path.join(root_folder_vert,"shifts",path_shifts[shift])
        os.makedirs(to_save_dir,exist_ok=True)
        output_pcd_path = os.path.join (to_save_dir,vert_name.replace('.obj','_clean.pcd'))
        print("output path:" + str(output_pcd_path))

        # save pcd
        o3d.io.write_point_cloud(output_pcd_path, curr_vert_pcd)

        if(visualize):
            o3d.visualization.draw([curr_vert_pcd])


if __name__ == "__main__":


    arg_parser = argparse.ArgumentParser(
        description="Separate point cloud of the spine obtained through raycasting of US labelmaps into individual "
                    "vertebrae pointclouds")

    arg_parser.add_argument(
        "--list_file_names",
        required=True,
        dest="txt_file",
        help="Txt file that contains all spines ids"
    )

    arg_parser.add_argument(
        "--root_path_vertebrae",
        required=True,
        dest="root_path_vertebrae",
        help="Root path to the vertebrae folders."
    )

    arg_parser.add_argument(
        "--root_path_spines",
        required=True,
        dest="root_path_spines",
        help="Root path to the spine folders."
    )

    arg_parser.add_argument(
        "--nr_deform_per_spine",
        required=True,
        dest="nr_deform_per_spine",
        help="Number of deformed spines per initial spine."
    )
    arg_parser.add_argument(
        "--nr_shift_per_spine",
        required=True,
        dest="nr_shift_per_spine",
        help="Number of deformed spines per initial spine."
    )
    arg_parser.add_argument(
        "--visualize",
        action="store_true",
        default=False,
        dest="visualize",
        help="Activate flag for visualization"
    )


    print("Separate point cloud of the spine obtained through raycasting of US labelmaps into individual "
                    "vertebrae pointclouds")

    args = arg_parser.parse_args()

    # iterate over all spines
    with open(args.txt_file) as file:
        spine_ids = [line.strip() for line in file]

    for spine_id in spine_ids:
        try:
            for deform in range(int(args.nr_deform_per_spine)):
                for shift in range(int(args.nr_shift_per_spine)):
                    separate_US_pointcloud_into_vertebrae(args.root_path_spines, args.root_path_vertebrae, spine_id, deform,shift,visualize=args.visualize)
        except:
            print("Something went wrong with processing" + str( spine_id))
