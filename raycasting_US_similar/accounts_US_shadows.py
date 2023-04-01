import numpy as np
import open3d as o3d
import os
from datetime import datetime
import argparse


def delete_shadows_from_pcd(pcd0_path, pcd1_path, pcd_combined_path, save_path, visualize=False):
    # load pcd0, pcd1 and pcd_combo
    pcd0 = o3d.io.read_point_cloud(pcd0_path)
    pcd1 = o3d.io.read_point_cloud(pcd1_path)
    pcd_combined = o3d.io.read_point_cloud(pcd_combined_path)

    # merge all of these 3 in one point cloud to be able to get the knn tree which allows for closest neighbor search
    all_in_one_pcd = o3d.geometry.PointCloud()
    points_from_all_pcds_stacked = np.concatenate(
        (np.asarray(pcd_combined.points), np.asarray(pcd0.points), np.asarray(pcd1.points)))
    all_in_one_pcd.points = o3d.utility.Vector3dVector(points_from_all_pcds_stacked)
    all_in_one_pcd.paint_uniform_color([0.5, 0.5, 0.5])

    # remember the order in which the points from the 3 point clouds are situated in the bigger point cloud
    nr_points_in_combo = np.asarray(pcd_combined.points).shape[0]
    nr_points_in_pcd0 = np.asarray(pcd0.points).shape[0]
    nr_points_in_pcd1 = np.asarray(pcd1.points).shape[0]

    pcd_tree = o3d.geometry.KDTreeFlann(all_in_one_pcd)

    # will this be a list of points from pcd_combined or will this be a list of points from pcd0? --> it s the same result
    unshadowed_points = []
    # at indices from 0 to nr_points_in_combo we can find points from the initial pcd_combined pcd
    for idx_points_in_pcd_combined in range(0, nr_points_in_combo):
        # find all points that are within a certain radius
        [_, idx_neighbors, _] = pcd_tree.search_radius_vector_3d(all_in_one_pcd.points[idx_points_in_pcd_combined],
                                                                 0.001)

        # find index and therefore closest that is larger than nr_points_in_combo and smallest than nr_points_in_combo+nr_points_in_pcd1
        # this means that this index will belong to a point in pcd0
        index_closest_point = next(
            filter(lambda index: index >= nr_points_in_combo and index < nr_points_in_combo + nr_points_in_pcd0,
                   idx_neighbors), None)

        if (index_closest_point != None):
            unshadowed_points.append(all_in_one_pcd.points[idx_points_in_pcd_combined])

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
    args = arg_parser.parse_args()
    print("Accounting for the shadows in US")

    # iterate over list of spines
    with open(args.list_spines) as file:
        spine_ids = file.read().splitlines()

    for spine_id in spine_ids:
        print("For spine: " + str(spine_id))
        path_to_pcds = os.path.join(args.root_paths_spines,spine_id,"rendering","pcd")
        pcds = sorted(os.listdir(path_to_pcds))
        delete_shadows_from_pcd(os.path.join(path_to_pcds, pcds[0]),
                                os.path.join(path_to_pcds, pcds[2]),
                                os.path.join(path_to_pcds, pcds[1]),
                                os.path.join(path_to_pcds, "account_for_shadows.pcd"))
