import numpy as np
import open3d as o3d
import os
from datetime import datetime
import argparse


# accounting for symmetry
def shadow_one_x_side(pcd_init, pcd_merged, pos=True):
    # merge init pcd and the merged pcd
    pcd_init_and_merged = o3d.geometry.PointCloud()
    points_from_init_and_merged = np.concatenate(
        (np.asarray(pcd_merged.points), np.asarray(pcd_init.points)))
    pcd_init_and_merged.points = o3d.utility.Vector3dVector(points_from_init_and_merged)
    pcd_init_and_merged.paint_uniform_color([0.5, 0.5, 0.5])

    # remember the order in which the points from the 2 point clouds are situated in the bigger point cloud
    # i.e the points in merged are the ones from [0 to nr_points_in_merged-1]
    # the points from initial pcd are the ones from [nr_points_in_merged to nr_points_in_merged + nr_points_in_pcd_init-1]
    nr_points_in_merged = np.asarray(pcd_merged.points).shape[0]
    nr_points_in_pcd_init = np.asarray(pcd_init.points).shape[0]

    pcd_tree = o3d.geometry.KDTreeFlann(pcd_init_and_merged)

    # will this be a list of points from pcd_merged or will this be a list of points from pcd_init? --> it s the same result
    unshadowed_points = []
    # at indices from 0 to nr_points_in_merged-1 we can find points from the initial pcd_merged pcd
    for idx_points_in_pcd_merged in range(0, nr_points_in_merged):
        curr_point = pcd_init_and_merged.points[idx_points_in_pcd_merged]

        # for merged pos x skip all points that have negative x values
        # for merged neg x skip all points that have positive x values 
        if (pos and curr_point[0] < 0) or (not pos and curr_point[0] > 0):
            continue
        # find all points that are within a certain radius
        [_, idx_neighbors, _] = pcd_tree.search_radius_vector_3d(curr_point, 0.01)

        # find index and therefore closest neighbor that is larger than nr_points_in_merged and smallest than nr_points_in_merged+nr_points_in_pcd_init-1
        # this means that this index will belong to a point in pcd_init which is the initial poincloud
        # so if there at least one point in the neighborhood that belongs to the initial, non shifted pointcloud we keep it and add it to the
        # unshadowed points
        idx_closest_points_from_init_pcd = [idx for idx in idx_neighbors if
                                            idx >= nr_points_in_merged and idx < nr_points_in_merged + nr_points_in_pcd_init]

        # add all points that are in pcd_init and  in the neighborhood of curr_point --> which belongs to the merged
        for idx in idx_closest_points_from_init_pcd:
            unshadowed_points.append(pcd_init_and_merged.points[idx])

    return unshadowed_points

# apply this on the fly
def add_noise(pcd, sigma):
    """
    Adds gaussian noise to the point cloud
    """
    # sample a vector that is from a gaussian distribution (num_points x 3)
    individual_points_shifts_y_axis = np.random.normal(loc=0.0, scale=sigma, size=np.asarray(pcd.points).shape[0])

    # then add it to the pcd.points
    new_points = np.asarray(pcd.points)
    new_points[:,1] += individual_points_shifts_y_axis
    pcd.points = o3d.utility.Vector3dVector(new_points)

    return pcd
def delete_shadows_from_pcd(pcd_init, pcd_merged_posx, pcd_merged_negx, save_path, add_noise_flag, visualize=False):
    """
    Accounts for the shadow that occurs because of the shifted pcd on the initial pcd
    """
    # for debugging
    print("PCD init: " + str(pcd_init))
    print("PCD merged posx: " + str(pcd_merged_posx))
    print("PCD merged negx: " + str(pcd_merged_negx))

    # load pcd_init, pcd_shifted and pcd_combo
    pcd_init = o3d.io.read_point_cloud(pcd_init)
    pcd_merged_posx = o3d.io.read_point_cloud(pcd_merged_posx)
    pcd_merged_negx = o3d.io.read_point_cloud(pcd_merged_negx)

    # these are lists of 3d numpy array 
    shadowed_pcd = shadow_one_x_side(pcd_init, pcd_merged_posx, pos=True)
    shadowed_pcd.extend(shadow_one_x_side(pcd_init, pcd_merged_negx, pos=False))

    shadowed_points = np.stack(shadowed_pcd)

    shadowed_pcd = o3d.geometry.PointCloud()
    shadowed_pcd.points = o3d.utility.Vector3dVector(np.asarray(shadowed_points))

    if (visualize):
        # visualize the resulting point cloud
        o3d.visualization.draw_geometries([shadowed_pcd])

    o3d.io.write_point_cloud(save_path, shadowed_pcd)

    if(add_noise_flag):
        sigma = 0.01
        noisy_pcd = add_noise(shadowed_pcd,sigma)
        o3d.io.write_point_cloud(save_path.replace(".pcd", "_noisy_sigma" + str(sigma) + ".pcd"), noisy_pcd)



if __name__ == '__main__':
    """
    Algo:
    - merge init pcd and the merged pcd in one point cloud to be able to get the knn tree which allows for closest neighbor search 
    - remember the order in which the points from the 2 point clouds are situated in the bigger point cloud 
    - for each point_from_combo in pcd_combo:
        - find closest points within the neighborhood with radius r 
        - keep all closest points belonging to point cloud_init

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
    arg_parser.add_argument(
        "--add_noise",
        action="store_true",
        default=False,
        dest="add_noise",
        help="Activate flag for adding gaussian noise to the pcd"
    )


    args = arg_parser.parse_args()
    print("Accounting for the shadows in US")

    # iterate over list of spines
    with open(args.list_spines) as file:
        spine_ids = file.read().splitlines()

    for spine_id in spine_ids:
        print("For spine: " + str(spine_id))

        rendering_path = os.path.join(args.root_paths_spines, spine_id, "rendering")
        root_dir = os.listdir(rendering_path)
        path_to_init_pcds = []
        for path in root_dir:
            joined = os.path.join(rendering_path, path)
            if os.path.isfile(joined):
                path_to_init_pcds.append(path)

        path_to_init_pcds = sorted(path_to_init_pcds)
        for deform in range(int(args.num_deform)):
            name = path_to_init_pcds[deform].split(".")[0]
            shifts_root = os.path.join(args.root_paths_spines, spine_id, "shifts", name)
            for shift_dir in sorted(os.listdir(shifts_root)):
                curr_rendering = os.path.join(shifts_root, shift_dir, "rendering")

                # accounting for symmetry
                delete_shadows_from_pcd(pcd_init=os.path.join(rendering_path, path_to_init_pcds[deform]),
                                        pcd_merged_posx=os.path.join(curr_rendering, name + "_posx_merged.pcd"),
                                        pcd_merged_negx=os.path.join(curr_rendering, name + "_negx_merged.pcd"),
                                        save_path=os.path.join(os.path.join(shifts_root, shift_dir, "account_for_shadow.pcd")),
                                        add_noise_flag=args.add_noise)
