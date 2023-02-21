import numpy as np
import h5py
import open3d as o3d
import os
import argparse

if __name__ == "__main__":

    arg_parser = argparse.ArgumentParser(description="Extract point clouds from h5 dataset")

    arg_parser.add_argument(
        "--path_dataset",
        required=False,
        dest="path_dataset",
        help="Path to the dataset"
    )

    arg_parser.add_argument(
        "--path_dataset_with_names",
        required=False,
        dest="path_dataset_with_names",
        help="Path to the dataset that contains a dataset_ids vector so that we have the names"
    )

    arg_parser.add_argument(
        "--dataset_entry",
        required=False,
        dest="dataset_entry",
        help="The entry of the dataset that we want to transform to point clouds"
    )

    arg_parser.add_argument(
        "--root_path_pcds",
        required=False,
        dest="root_path_pcds",
        help="Path where the resulting point clouds will be saved "
    )
    args = arg_parser.parse_args()

    nr_pcds = 11

    dataset = h5py.File(args.path_dataset, 'r')
    dataset_with_names = h5py.File(args.path_dataset_with_names, 'r')

    pcds = np.array(dataset[args.dataset_entry][()])
    names = np.array(dataset_with_names['datasets_ids'])

    os.makedirs(args.root_path_pcds, exist_ok=True)
    for i in range(nr_pcds):
        # get corresponding points from current point cloud as np array
        pcd = pcds[i]

        # create empty point cloud
        o3d_pcd = o3d.geometry.PointCloud()

        # add points to the point cloud
        o3d_pcd.points = o3d.utility.Vector3dVector(pcd)
        filtered_pcd, _ = o3d_pcd.remove_radius_outlier(4, 0.03)

        # save point cloud
        if('results' in args.path_dataset):
            o3d.io.write_point_cloud(os.path.join(args.root_path_pcds,str(names[i]) + "_" + str(pcds.shape[1]) + "_reconstruction.ply"), filtered_pcd)
        else:
            o3d.io.write_point_cloud(os.path.join(args.root_path_pcds,str(names[i])+ "_" + str(pcds.shape[1]) + "_GT.ply"), filtered_pcd)






