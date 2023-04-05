import numpy as np
import open3d as o3d
import os
from datetime import datetime
import argparse


def delete_shadows_from_pcd(pcd0_path, pcd1_path, pcd_combined_path, save_path, visualize=False):

    # for debugging
    print("PCD init: " + str(pcd0_path))
    print("PCD shifted: " + str(pcd1_path))
    print("PCD merged: " + str(pcd_combined_path))

    # load pcd_init, pcd_shifted and pcd_combo
    pcd_init = o3d.io.read_point_cloud(pcd0_path)
    pcd_shifted = o3d.io.read_point_cloud(pcd1_path)
    pcd_merged = o3d.io.read_point_cloud(pcd_combined_path)

    # merge all of these 3 in one point cloud to be able to get the knn tree which allows for closest neighbor search
    all_in_one_pcd = o3d.geometry.PointCloud()
    points_from_all_pcds_stacked = np.concatenate(
        (np.asarray(pcd_merged.points), np.asarray(pcd_init.points), np.asarray(pcd_shifted.points)))
    all_in_one_pcd.points = o3d.utility.Vector3dVector(points_from_all_pcds_stacked)
    all_in_one_pcd.paint_uniform_color([0.5, 0.5, 0.5])

    # remember the order in which the points from the 3 point clouds are situated in the bigger point cloud
    # i.e the points in merged are the ones from [0 to nr_points_in_merged-1]
    # the points from initial pcd are the ones from [nr_points_in_merged to nr_points_in_merged + nr_points_in_pcd_init-1]
    # the points from shifted pcd are the ones from [nr_points_in_merged + nr_points_in_pcd_init to nr_points_in_merged + nr_points_in_pcd_init + nr_points_in_pcd_shifted-1 ]
    nr_points_in_merged = np.asarray(pcd_merged.points).shape[0]
    nr_points_in_pcd_init = np.asarray(pcd_init.points).shape[0]
    nr_points_in_pcd_shifted = np.asarray(pcd_shifted.points).shape[0]

    pcd_tree = o3d.geometry.KDTreeFlann(all_in_one_pcd)

    # will this be a list of points from pcd_merged or will this be a list of points from pcd_init? --> it s the same result
    unshadowed_points = []
    # at indices from 0 to nr_points_in_merged-1 we can find points from the initial pcd_merged pcd
    for idx_points_in_pcd_merged in range(0, nr_points_in_merged):
        # find all points that are within a certain radius
        [_, idx_neighbors, _] = pcd_tree.search_radius_vector_3d(all_in_one_pcd.points[idx_points_in_pcd_merged],
                                                                 0.01)

        # find index and therefore closest neighbor that is larger than nr_points_in_merged and smallest than nr_points_in_merged+nr_points_in_pcd_init-1
        # this means that this index will belong to a point in pcd_init which is the initial poincloud
        # so if there at least one point in the neighborhood that belongs to the initial, non shifted pointcloud we keep it and add it to the
        # unshadowed points
        idx_closest_points_from_init_pcd = [idx for idx in idx_neighbors if idx >= nr_points_in_merged and idx < nr_points_in_merged + nr_points_in_pcd_init]
        """
          index_closest_point = next(
            filter(lambda index: index >= nr_points_in_merged and index < nr_points_in_merged + nr_points_in_pcd_init,
                   idx_neighbors), None)

        if (index_closest_point != None):
            unshadowed_points.append(all_in_one_pcd.points[index_closest_point])
        """
        # add all points that
        for idx in idx_closest_points_from_init_pcd:
            unshadowed_points.append(all_in_one_pcd.points[idx])


    # create a new point cloud with the points that have indexes in idx_of_unshadowed_points and visualize it if we cannot visualize the neighborhoods
    unshadowed_pcd = o3d.geometry.PointCloud()
    unshadowed_pcd.points = o3d.utility.Vector3dVector(np.asarray(unshadowed_points))

    if (visualize):
        # visualize the resulting point cloud
        o3d.visualization.draw_geometries([unshadowed_pcd])

    o3d.io.write_point_cloud(save_path,
                             unshadowed_pcd)

if __name__ == '__main__':
    """
    Algo:
    - merge all of these 3 in one point cloud to be able to get the knn tree which allows for closest neighbor search 
    - remember the order in which the points from the 3 point clouds are situated in the bigger point cloud 
    - for each point_from_combo in pcd_combo:
        - find closest point 
        - if closest point in pcd0 then keep, else discard 
        - save the resulting point cloud  

    """

    arg_parser = argparse.ArgumentParser(
        description="Generate dataset with complete and partial pointclouds from CT for shape completion")

    arg_parser.add_argument(
        "--root_paths_spines",
        required=True,
        dest="root_paths_spines",
        help="Path to root directory of all spines"
    )

    arg_parser.add_argument(
        "--list_spines",
        required=True,
        dest="list_spines",
        help="Txt file that contains all spines"
    )

    arg_parser.add_argument(
        "--num_deform",
        required=True,
        dest="num_deform",
        help="Number of deformations for one spine"
    )

    args = arg_parser.parse_args()
    print("Accounting for the shadows in US")

    # iterate over list of spines
    with open(args.list_spines) as file:
        spine_ids = file.read().splitlines()

    for spine_id in spine_ids:
        print("For spine: " + str(spine_id))

        rendering_path = os.path.join(args.root_paths_spines,spine_id,"rendering")
        root_dir = os.listdir(rendering_path)
        path_to_init_pcds = []
        for path in root_dir:
            joined = os.path.join(rendering_path,path)
            if os.path.isfile(joined):
                path_to_init_pcds.append(path)

        path_to_init_pcds = sorted(path_to_init_pcds)
        for deform in range(int(args.num_deform)):
            name = path_to_init_pcds[deform].split(".")[0]
            shifts_root = os.path.join(args.root_paths_spines, spine_id, "shifts",name)
            for shift_dir in sorted(os.listdir(shifts_root)):
                curr_rendering = os.path.join(shifts_root,shift_dir,"rendering")
                delete_shadows_from_pcd(pcd0_path=os.path.join(rendering_path,path_to_init_pcds[deform]),
                                        pcd1_path=os.path.join(curr_rendering,name + "_shifted.pcd"),
                                        pcd_combined_path=os.path.join(curr_rendering,name + "_merged.pcd"),
                                        save_path=os.path.join(os.path.join(shifts_root,shift_dir,"account_for_shadow.pcd")))


