import collections
import numpy as np
import h5py
import os
import open3d as o3d
import math
from collections import Counter
import argparse
from utils import namings
import fps
import logging 


def processOneVertebra(pathCompleteVertebra, pathToPartialPCD, nrPointsProPartialPC=2048,
                       nrPointsProCompletePC=4096,
                       visualize=False):
    """

    :param pathCompleteVertebra: path to the .obj file representing a complete vertebra
    :param pathToRootPartialPCD: path to the root folder containing the .pcd files which represent partial point clouds
    :param nrPartialPCDPerSample: number of partial pointclouds per vertebra
    :param nrPointsProPointCloud: number of points to be sampled from a pointcloud
    :param visualize: flag for visualization
    :return: complete vertebra point cloud, list of partial point clouds

    """

    logging.debug("Processing: " + pathCompleteVertebra)

    # load complete vertebra and its partial point cloud
    completeVertebra = o3d.io.read_triangle_mesh(pathCompleteVertebra)
    partial_pcd = o3d.io.read_point_cloud(pathToPartialPCD)
    logging.debug("Path to partial pcd " + str(pathToPartialPCD))

    # delete all points that are below the center of mass of completeVert
    """
    # hopefully this is not needed because we do the raycasting of the whole spine
    center_pcd = partial_pcd.get_center()
    points = np.asarray(partial_pcd.points).tolist()
    points_above_center_of_mass = [point for point in points if (point[1] > center_pcd[1]-4)]
    partial_pcd.points = o3d.utility.Vector3dVector(np.asarray(points_above_center_of_mass))
    """

    # scale down
    # they are already scaled down from before raycasting
    """
    completeVertebra.scale(scale_factor, completeVertebra.get_center())
    partial_pcd.scale(scale_factor, completeVertebra.get_center())
    """

    # move the pcd and the vert to the center
    center_vertebra = completeVertebra.get_center()
    completeVertebra.vertices = o3d.utility.Vector3dVector(completeVertebra.vertices - center_vertebra)
    partial_pcd.points = o3d.utility.Vector3dVector(partial_pcd.points - center_vertebra)

    # sample complete vertebra with the poisson disk sampling technique
    pointCloudComplete = o3d.geometry.TriangleMesh.sample_points_poisson_disk(completeVertebra, nrPointsProCompletePC)

    # sample partial point cloud Farthest Point Sample

    # check if partial_pcd has >= nrPointsProPartialPC points
    # if yes then sample this number directly
    # if it has at least half of the nrPointsProPartialPC duplicate the number of points then resample
    nr_points_in_partial_pcd = np.asarray(partial_pcd.points).shape[0]
    logging.debug("Initial number of points in pcd:" + str(nr_points_in_partial_pcd))
    if (nr_points_in_partial_pcd >= nrPointsProPartialPC):
        sampled_partial_pcd = fps.fps_points(np.asarray(partial_pcd.points), num_samples=nr_points_in_partial_pcd)
    else:
        logging.debug("PCD with less than " + str(nrPointsProPartialPC) + "points" + str(os.path.basename(pathToPartialPCD)))
        return 0, []

    if (visualize):
        coord_sys = o3d.geometry.TriangleMesh.create_coordinate_frame()
        # create pcd from the sampling
        pcd_subsampled = o3d.geometry.PointCloud()
        pcd_subsampled.points = o3d.utility.Vector3dVector(sampled_partial_pcd)

        logging.debug("Visualizing input and ground truth after scaling and centering")
        pointCloudComplete.paint_uniform_color([0, 1, 0])
        partial_pcd.paint_uniform_color([0, 0, 1])
        o3d.visualization.draw([partial_pcd, coord_sys, pointCloudComplete])

    partial_pcds = []
    partial_pcds.append((sampled_partial_pcd))
    return np.asarray(pointCloudComplete.points), partial_pcds


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


def saveToH5(fileName, stackedCropped, stackedComplete, labels, datasets_ids, nrSamplesPerClass=16):
    # save the dataset in a .h5 file for VRCNet
    vertebrae_file = h5py.File(fileName, "w")
    dset_incompletepcds = vertebrae_file.create_dataset("incomplete_pcds", data=stackedCropped)
    dset_completepcds = vertebrae_file.create_dataset("complete_pcds", data=stackedComplete)
    dset_labels = vertebrae_file.create_dataset("labels", data=labels)
    dset_ids = vertebrae_file.create_dataset("datasets_ids", data=datasets_ids)
    number_per_class = computeNrPerClass(labels, nrSamplesPerClass)
    dset_number_per_class = vertebrae_file.create_dataset("number_per_class", data=number_per_class)
    logging.debug("Number_per_class" + str(number_per_class))


def processAllVertebrae(list_path, rootDirectoryVertebrae, saveTo,
                        nr_deform_per_sample, nr_shifts_per_sample, visualize=False, nrPointsProPartialPC=2048,
                        nrPointsProCompletePC=4096):
    # prepare lists for storing all vertebrae
    labels = []
    complete_pcds_all_vertebrae = []
    partial_pcds_all_vertebrae = []
    dataset_ids = []

    # create a list with all vertebrae names
    with open(os.path.join(list_path)) as file:
        model_list = [line.strip() for line in file]

    # get min label
    all_labels = [extractLabel(model_id) for model_id in model_list]
    min_label = np.min(np.asarray(all_labels))

    idx = 0

    # iterate over the vertebrae names
    for model_id in model_list:
        vert_folder_name = model_id
        shift_root = os.path.join(rootDirectoryVertebrae, vert_folder_name, "shifts")
        shift_folders = sorted(os.listdir(shift_root))
        if (len(shift_folders) != int(nr_shifts_per_sample)):
            raise Exception(
                "Nmber of found shift folders does not match the number of given shifts per sample for " + str(
                    model_id))

        for shift in range(int(nr_shifts_per_sample)):
            for deform in range(nr_deform_per_sample):

                logging.debug(str(idx) + "/" + str(len(model_list) * nr_deform_per_sample * int(nr_shifts_per_sample)))
                logging.debug("Processing " + str(model_id) + " deformation: " + str(deform) + " and shift: " + str(shift))

                # get the name of the pcd and the name of the mesh
                polluted_pcd_path = os.path.join(rootDirectoryVertebrae, vert_folder_name, "shifts",
                                                 shift_folders[shift],
                                                 namings.get_name_polluted_vert_pcd(vert_folder_name, deform))
                vert_mesh_path = os.path.join(rootDirectoryVertebrae, vert_folder_name,
                                              namings.get_name_vert_deform_scaled(vert_folder_name, deform))

                #  process each vertebra individually
                complete_pcd, partial_pcds = processOneVertebra(pathCompleteVertebra=vert_mesh_path,
                                                                pathToPartialPCD=polluted_pcd_path,
                                                                visualize=visualize,
                                                                nrPointsProPartialPC=nrPointsProPartialPC,
                                                                nrPointsProCompletePC=nrPointsProCompletePC)

                # if the partial point cloud has less than nrPointsProPartialPC then partial_pcds will be an empty list
                if len(partial_pcds) == 0:
                    continue
                logging.debug(partial_pcds[0].shape)
                # add it to h5py
                # make sure that the smallest label will be 0
                label_normalized = extractLabel(model_id) - min_label
                complete_pcds_all_vertebrae.append(complete_pcd)
                partial_pcds_all_vertebrae.extend(partial_pcds)
                dataset_ids.append((model_id + "_deform" + str(deform)).encode("ascii"))

                # size of labels = size of all_partial_pcds
                labels.extend([label_normalized for j in range(0, 1)])
                idx += 1

    # stack the results
    stacked_partial_pcds = np.stack(partial_pcds_all_vertebrae, axis=0)
    stacked_complete_pcds = np.stack(complete_pcds_all_vertebrae, axis=0)
    stacked_dataset_ids = np.stack(dataset_ids, axis=0)
    labels_array = np.asarray(labels)

    # for debugging logging.debug the shape
    logging.debug("Shape of stacked_partial_pcds: " + str(stacked_partial_pcds.shape))
    logging.debug("Shape of stacked_complete_pcds" + str(stacked_complete_pcds.shape))
    logging.debug("Shape of labels" + str(labels_array.shape))

    saveToH5(saveTo, stackedCropped=stacked_partial_pcds, stackedComplete=stacked_complete_pcds,
             labels=labels_array, datasets_ids=stacked_dataset_ids, nrSamplesPerClass=1)


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser(description="Create a dataset for completion training")
    arg_parser.add_argument(
        "--vertebrae_list",
        required=True,
        dest="vertebrae_list",
        help="Txt file with a list of spines"
    )
    arg_parser.add_argument(
        "--root_path_vertebrae",
        required=True,
        dest="folder_complete_meshes",
        help="Root path of the vertebrae folders"

    )
    arg_parser.add_argument(
        "--result_h5_file",
        required=True,
        dest="result_h5_file",
        help="Path to the h5 file where the dataset will be saved"
    )

    arg_parser.add_argument(
        "--nr_deform_per_sample",
        required=True,
        dest="nr_deform_per_sample",
        help="Number of deformations for one spine."
    )
    arg_parser.add_argument(
        "--nr_points_per_point_cloud",
        required=True,
        dest="nr_points_per_point_cloud",
        help="Number of points that will be sampled both from the partial point cloud and from the complete mesh"
    )

    arg_parser.add_argument(
        "--num_shifts",
        required=True,
        dest="num_shifts",
        help="Number of shifts for one spine"
    )
    arg_parser.add_argument(
        "--visualize_vertebrae",
        action="store_true",
        default=False,
        help="Visualize vertebrae before they are added to the dataset"
    )

    args = arg_parser.parse_args()
    logging.debug("Creating the shape completion dataset from partial point clouds obtained from US")

    logging.basicConfig(filename="create_dataset_logs.txt", filemode='w',level=logging.DEBUG, force=True)

    # process all vertebrae
    processAllVertebrae(list_path=args.vertebrae_list,
                        rootDirectoryVertebrae=args.folder_complete_meshes,
                        saveTo=args.result_h5_file,
                        nr_deform_per_sample=int(args.nr_deform_per_sample),
                        visualize=args.visualize_vertebrae,
                        nrPointsProPartialPC=int(args.nr_points_per_point_cloud),
                        nrPointsProCompletePC=int(args.nr_points_per_point_cloud),
                        nr_shifts_per_sample=args.num_shifts
                        )
