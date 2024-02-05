import numpy as np
import h5py
import open3d as o3d
import os
import argparse

def get_filtered_pcd_from_numpy(np_array):
    # create empty point cloud
    pcd = o3d.geometry.PointCloud()
    # add points to the point cloud
    pcd.points = o3d.utility.Vector3dVector(np_array)
    filtered_np_array, _ = pcd.remove_radius_outlier(4, 0.03)
    return filtered_np_array

if __name__ == "__main__":

    arg_parser = argparse.ArgumentParser(description="Extract point clouds from h5 dataset")

    arg_parser.add_argument(
        "--results_dataset",
        required=False,
        dest="results_dataset",
        help="Path to the results dataset obtained from inference"
    )

    arg_parser.add_argument(
        "--inputs_dataset",
        required=False,
        dest="inputs_dataset",
        help="Path to the inference dataset"
    )

    arg_parser.add_argument(
        "--root_path_pcds",
        required=False,
        dest="root_path_pcds",
        help="Path where the resulting point clouds will be saved "
    )
    arg_parser.add_argument(
        "--visualize",
        action="store_true",
        help="Visualize point clouds "
    )
    args = arg_parser.parse_args()

    nr_pcds = 100

    # read the h5 files of the datasets
    results_dataset = h5py.File(args.results_dataset, 'r')
    inputs_dataset = h5py.File(args.inputs_dataset, 'r')

    # get the numpy arrays
    inputs = np.array(inputs_dataset['incomplete_pcds'])
    names = np.array(inputs_dataset['datasets_ids'])
    results = np.array(results_dataset["results"][()])
    gts = np.array(results_dataset["gt"][()])

    if(len(names) == 0):
        names = []
        for idx in range(nr_pcds):
            names.append(str(idx))

    os.makedirs(args.root_path_pcds, exist_ok=True)

    if(results.shape[0]<5):
        nr_pcds = results.shape[0]

    for i in range(0, nr_pcds):

        filtered_input = get_filtered_pcd_from_numpy(inputs[i])
        filtered_gt = get_filtered_pcd_from_numpy(gts[i])
        filtered_result = get_filtered_pcd_from_numpy(results[i])

        if(args.visualize):
            filtered_input.paint_uniform_color([1, 0, 0])
            filtered_gt.paint_uniform_color([0, 1, 0])
            filtered_result.paint_uniform_color([0, 0, 1])

            o3d.visualization.draw_geometries([filtered_input,filtered_gt,filtered_result])

        # save point cloud
        o3d.io.write_point_cloud(os.path.join(args.root_path_pcds,str(names[i]) + "_" + str(inputs[i].shape[1]) + "_input.ply"), filtered_input)
        o3d.io.write_point_cloud(os.path.join(args.root_path_pcds,str(names[i])+ "_" + str(gts[i].shape[1]) + "_GT.ply"), filtered_gt)
        o3d.io.write_point_cloud(os.path.join(args.root_path_pcds,str(names[i]) + "_" + str(results[i].shape[1]) + "_reconstruction.ply"), filtered_result)






