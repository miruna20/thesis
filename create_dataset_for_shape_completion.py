import collections

import numpy as np
import h5py
import os
import open3d as o3d
import sys
import math
from collections import Counter
import argparse
import glob


def processOneVertebra(pathCompleteVertebra, pathToRootPartialPCD, nrPartialPCDPerSample=16, nrPointsProPartialPC=2048,
                       nrPointsProCompletePC=4098,
                       visualize=False):
    """

    :param pathCompleteVertebra: path to the .obj file representing a complete vertebra
    :param pathToRootPartialPCD: path to the root folder containing the .pcd files which represent partial point clouds
    :param nrPartialPCDPerSample: number of partial pointclouds per vertebra
    :param nrPointsProPointCloud: number of points to be sampled from a pointcloud
    :param visualize: flag for visualization
    :return: complete vertebra point cloud, list of partial point clouds

    """

    print("Processing: " + pathCompleteVertebra)

    # load complete vertebra
    completeVertebra = o3d.io.read_triangle_mesh(pathCompleteVertebra)

    # sample complete vertebra with the poisson disk sampling technique
    pointCloudComplete = o3d.geometry.TriangleMesh.sample_points_poisson_disk(completeVertebra, nrPointsProCompletePC)

    # iterate over all partial point clouds
    crops = []
    if (not os.path.isdir(pathToRootPartialPCD)):
        print("Root folder to partial point clouds does not exist: " + str(pathToRootPartialPCD), file=sys.stderr)

    partial_pcds_names = sorted(os.listdir(pathToRootPartialPCD))

    # make sure that there are nrPartialPCDPerSample in all directories
    if (len(partial_pcds_names) != nrPartialPCDPerSample):
        print("Insufficient partial pointclouds for this vertebra", file=sys.stderr)
        return

    # visualize complete vertebra
    if (visualize):
        o3d.visualization.draw([pointCloudComplete])

    # iterate over partial point clouds
    for partial_pcd_name in partial_pcds_names:
        # load each one
        partial_pcd = o3d.io.read_point_cloud(os.path.join(pathToRootPartialPCD, partial_pcd_name))

        # sample with Farthest Point Sample 2048 points
        try:
            sampled_partial_pcd = partial_pcd.farthest_point_down_sample(nrPointsProPartialPC)
        except:
            print("PCD with less than " + str(nrPointsProPartialPC) + "points" + partial_pcd_name)
            return 0, []

        # add them to the list of partial pointclouds
        # duplicate number of points
        #crop = np.repeat(np.asarray(sampled_partial_pcd.points), repeats=2, axis=0)
        crop = np.asarray(sampled_partial_pcd.points)
        crops.append(crop)

        # visualize the subsampling (on the left the partial point cloud, on the right the FPS sampled point cloud)
        if (visualize):
            # translate the subsampled point cloud to avoid overlap during visualization
            print("Visualizing initial partial pointcloud and the sampled one")
            sampled_partial_pcd_translated = o3d.geometry.PointCloud.translate(sampled_partial_pcd,
                                                                               np.asarray([1.5, 0, 0]))
            o3d.visualization.draw([partial_pcd, sampled_partial_pcd_translated])

    return np.asarray(pointCloudComplete.points), crops


def extractLabel(nameVertebra):
    label = nameVertebra.split("verLev")[1]
    return int(label)


def computeNrPerClass(labels, nrSamplesPerClass=16):
    # the labels need to start with 0 (make sure that if you have labels for lumbar vertebrae you substract minimal label

    nr_samples_per_class = Counter(labels)
    ordered_dict = collections.OrderedDict(sorted(nr_samples_per_class.items()))

    nr_samples_per_class_as_list = []
    for key in ordered_dict.keys():
        nr_samples_per_class_as_list.append(math.floor(ordered_dict[key] / nrSamplesPerClass))

    return np.asarray(nr_samples_per_class_as_list)


def saveToH5(fileName, stackedCropped, stackedComplete, labels, nrSamplesPerClass=16):
    # save the dataset in a .h5 file for VRCNet
    vertebrae_file = h5py.File(fileName, "w")
    dset_incompletepcds = vertebrae_file.create_dataset("incomplete_pcds", data=stackedCropped)
    dset_completepcds = vertebrae_file.create_dataset("complete_pcds", data=stackedComplete)
    dset_labels = vertebrae_file.create_dataset("labels", data=labels)
    number_per_class = computeNrPerClass(labels, nrSamplesPerClass)
    dset_number_per_class = vertebrae_file.create_dataset("number_per_class", data=number_per_class)
    print("Number_per_class" + str(number_per_class))


def processAllVertebrae(list_path, rootDirectoryVertebrae, rootDirectoryPartialPointClouds, saveTo,
                        nrPartialPCDPerSample=16, visualize=False, nrPointsProPartialPC=2048,
                        nrPointsProCompletePC=4098):
    # prepare lists for storing all vertebrae
    labels = []
    complete_pcds_all_vertebrae = []
    partial_pcds_all_vertebrae = []

    # create a list with all vertebrae names
    with open(os.path.join(list_path)) as file:
        model_list = [line.strip() for line in file]

    # get min label
    all_labels = [extractLabel(model_id) for model_id in model_list]
    min_label = np.min(np.asarray(all_labels))

    idx = 0

    # iterate over the vertebrae names
    for model_id in model_list:
        print(str(idx) + "/" + str(len(model_list)))

        # obtain the path to the complete vertebra and to the root of the partial point clouds
        unique_identifier_mesh_vertebra = "*verLev*" + "*_msh_scaled_centered.obj"
        model_path = glob.glob(os.path.join(os.path.join(rootDirectoryVertebrae, model_id), unique_identifier_mesh_vertebra))[0]
        partial_models_root = os.path.join(rootDirectoryPartialPointClouds, model_id, "rotated")

        #  process each vertebra individually
        try:
            complete_pcd, partial_pcds = processOneVertebra(pathCompleteVertebra=model_path,
                                                            pathToRootPartialPCD=partial_models_root,
                                                            nrPartialPCDPerSample=nrPartialPCDPerSample,
                                                            visualize=visualize,
                                                            nrPointsProPartialPC=nrPointsProPartialPC,
                                                            nrPointsProCompletePC=nrPointsProCompletePC)
        except:
            print("Something went wrong with the processing")
            continue

        # in case at least one of the partial point clouds had less than 2048 points then return an empty list as partial_pcds
        if len(partial_pcds) == 0:
            continue

        # add it to h5py
        # make sure that the smallest label will be 0
        label_normalized = extractLabel(model_id) - min_label
        complete_pcds_all_vertebrae.append(complete_pcd)
        partial_pcds_all_vertebrae.extend(partial_pcds)

        # size of labels = size of all_partial_pcds
        labels.extend([label_normalized for j in range(0, nrPartialPCDPerSample)])
        idx += 1

    # stack the results
    stacked_partial_pcds = np.stack(partial_pcds_all_vertebrae, axis=0)
    stacked_complete_pcds = np.stack(complete_pcds_all_vertebrae, axis=0)
    labels_array = np.asarray(labels)

    # for debugging print the shape
    print("Shape of stacked_partial_pcds: " + str(stacked_partial_pcds.shape))
    print("Shape of stacked_complete_pcds" + str(stacked_complete_pcds.shape))
    print("Shape of labels" + str(labels_array.shape))

    saveToH5(saveTo, stackedCropped=stacked_partial_pcds, stackedComplete=stacked_complete_pcds,
             labels=labels_array, nrSamplesPerClass=nrPartialPCDPerSample)


if __name__ == "__main__":
    ## example of setup:
    # vertebraeList = "/home/miruna20/Documents/Thesis/Code/Preprocessing/create_partialPCD/verse2020_vertebrae_train_lumbar.txt"
    # rootFolderCompleteMeshes = "/home/miruna20/Documents/Thesis/Dataset/VerSe2020"
    # rootFolderPartialPCD = "/home/miruna20/Documents/Thesis/Code/Preprocessing/create_partialPCD/pcn/render/results/pcd"
    # saveTo = "/home/miruna20/Documents/Thesis/Dataset/VerSe2020/vertebrae_train_lumbar.h5"
    # nrIncompletePCDsPerSample = 16

    arg_parser = argparse.ArgumentParser(description="Create a dataset for completion training")
    arg_parser.add_argument(
        "--vertebrae_list",
        required=True,
        dest="vertebrae_list",
        help="txt file with names of vertebrae that will be included in the dataset"
    )
    arg_parser.add_argument(
        "--root_path_vertebrae",
        required=True,
        dest="folder_complete_meshes",
        help="Root path of the vertebrae folders"

    )
    arg_parser.add_argument(
        "--directory_partial_pcds",
        required=True,
        dest="directory_partial_pcds",
        help="Root folder for partial point clouds"
    )
    arg_parser.add_argument(
        "--result_h5_file",
        required=True,
        dest="result_h5_file",
        help="Path to the h5 file where the dataset will be saved"
    )
    arg_parser.add_argument(
        "--nr_partial_pcds_per_sample",
        required=True,
        dest="nr_partial_pcds_per_sample",
        help="Number of partial point clouds per sample. This should correspond to the number that has been generated by the rendering script"
    )

    arg_parser.add_argument(
        "--nr_points_per_point_cloud",
        required=True,
        dest="nr_points_per_point_cloud",
        help="Number of points per point cloud. This number is used for the sampling technique."
    )

    arg_parser.add_argument(
        "--visualize_vertebrae",
        action="store_true",
        help="Visualize vertebrae before they are added to the dataset"
    )

    args = arg_parser.parse_args()

    # process all vertebrae
    processAllVertebrae(list_path=args.vertebrae_list,
                        rootDirectoryVertebrae=args.folder_complete_meshes,
                        rootDirectoryPartialPointClouds=args.directory_partial_pcds,
                        saveTo=args.result_h5_file,
                        nrPartialPCDPerSample=int(args.nr_partial_pcds_per_sample),
                        visualize=args.visualize_vertebrae,
                        nrPointsProPartialPC=int(args.nr_points_per_point_cloud),
                        nrPointsProCompletePC=int(args.nr_points_per_point_cloud),
                        )
