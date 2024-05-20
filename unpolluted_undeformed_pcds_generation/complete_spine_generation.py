import collections
import numpy as np
import h5py
import os
import open3d as o3d
import math
from collections import Counter
import argparse
from utils import namings
from utils import fps
import logging
import re
import time
import glob
import sys
import nibabel as nib


def get_vert_body(mesh):
    # Get the oriented bounding box (OBB) of the mesh
    obb = mesh.get_oriented_bounding_box()
    obb_corners = np.asarray(obb.get_box_points())

    # Find the four corners with the maximum y-values
    plane_corners = obb_corners[np.argsort(obb_corners[:, 1])[-4:]]

    # Calculate the normal vector of the plane
    v1, v2 = plane_corners[1] - plane_corners[0], plane_corners[2] - plane_corners[0]
    normal_vector = np.cross(v1, v2) / np.linalg.norm(np.cross(v1, v2))
    reference_vector = np.array([0, 1, 0])

    # Ensure normal vector is within 180 degrees of the reference vector
    if np.dot(normal_vector, reference_vector) < 0:
        normal_vector *= -1

    # Calculate the centroid of the mesh
    mesh_centroid = np.asarray(mesh.get_center())

    # Calculate the equation of the plane
    D = -np.dot(normal_vector, mesh_centroid)
    plane_equation = np.concatenate((normal_vector, [D]))

    # Project plane corners onto the plane
    new_plane_corners = [corner - np.dot(corner - mesh_centroid, normal_vector) * normal_vector for corner in
                         plane_corners]

    # Find all points in the mesh below the plane
    below_points = [point for point in np.asarray(mesh.vertices) if np.dot(point, normal_vector) + D < 0]

    # Visualize

    #if (vis):
    """
    coord_sys = o3d.geometry.TriangleMesh.create_coordinate_frame()
    o3d.visualization.draw_geometries([mesh, o3d.geometry.PointCloud(o3d.utility.Vector3dVector(plane_corners)),
                                       o3d.geometry.PointCloud(o3d.utility.Vector3dVector(new_plane_corners)),
                                       o3d.geometry.PointCloud(o3d.utility.Vector3dVector(below_points)),
                                       coord_sys,complete])
    """



    return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(below_points)),plane_corners, new_plane_corners


def find_file_in_folder_with_unique_identifier(folder, unique_identifier):
    files = glob.glob(os.path.join(folder, unique_identifier))
    if (len(files) != 1):
        print("More or less than 1 file were found for folder" + str(folder) + " and unique identifier: " + str(
            unique_identifier),
              file=sys.stderr)
        return ""
    return files[0]


def align_real_to_synthetic(real_pcd, synthetic_pcd):
    """
    Align the real input point cloud with a synthetic template of the same level
    """

    # match the centers of the 2 point clouds
    center_synthetic = np.asarray(synthetic_pcd.get_center())
    center_real = np.asarray(real_pcd.get_center())
    translation = center_synthetic - center_real

    return translation


def complete_vert_fits_into_unit_sphere(completeVertebra):
    bb_complete_vert = completeVertebra.get_axis_aligned_bounding_box()
    length_x = bb_complete_vert.get_max_bound()[0] - bb_complete_vert.get_min_bound()[0]
    length_y = bb_complete_vert.get_max_bound()[1] - bb_complete_vert.get_min_bound()[1]
    length_z = bb_complete_vert.get_max_bound()[2] - bb_complete_vert.get_min_bound()[2]

    if (length_x < 1 and length_y < 1 and length_z < 1):
        return True
    return False


def processOneSpine(path_complete_spine, pathToPartialPCD, nrPointsProPartialPC=2048,
                       nrPointsProCompletePC=4096,
                       visualize=False):
    """

    :param path_complete_spine: path to the .obj file representing a complete vertebra
    :param pathToRootPartialPCD: path to the root folder containing the .pcd files which represent partial point clouds
    :param nrPartialPCDPerSample: number of partial pointclouds per vertebra
    :param nrPointsProPointCloud: number of points to be sampled from a pointcloud
    :param visualize: flag for visualization
    :return: complete vertebra point cloud, list of partial point clouds

    """

    logging.debug("Processing: " + path_complete_spine)

    # load complete vertebra and its partial point cloud
    complete_spine = o3d.io.read_triangle_mesh(path_complete_spine)

    #o3d.visualization.draw([labelmap,completeVertebra])

    partial_pcd = o3d.io.read_point_cloud(pathToPartialPCD)
    logging.debug("Path to partial pcd " + str(pathToPartialPCD))

    # labelmap_pcd.points = o3d.utility.Vector3dVector(voxel_coords)

    # first scale everything back up with a scale factor of 100
    complete_spine.scale(100, center=np.asarray([0, 0, 0]))
    partial_pcd.scale(100, center=np.asarray([0, 0, 0]))

    unit_sphere_size = 1
    bb_partial_pcd = partial_pcd.get_axis_aligned_bounding_box()

    # we know that the length between the first and the last vert will be along the z axis
    length_partial_pcd = bb_partial_pcd.get_max_bound()[2] - bb_partial_pcd.get_min_bound()[2]

    # + 40 here is just a padding to ensure that the full shape of the vertebra
    # will fit in the unit sphere (which it won't without padding if the axis from arch to vert body
    # is longer than the one in between transverse process
    scaling_factor = unit_sphere_size / (length_partial_pcd + 40)

    complete_spine.scale(scaling_factor, center=np.asarray([0, 0, 0]))

    # do not add vertebrae with GT larger than unit sphere
    if not complete_vert_fits_into_unit_sphere(complete_spine):
        logging.debug("DOES NOT FIT INTO UNIT SPHERE")
        return [], [], []


    synth_template_path = os.path.join("../synthetic_templates", "spine.pcd")
    synthetic_pcd = o3d.io.read_point_cloud(synth_template_path)

    partial_pcd.scale(scaling_factor, center=np.asarray([0, 0, 0]))
    synthetic_pcd.scale(scaling_factor, center=np.asarray([0, 0, 0]))


    # translation to the center of the template
    transl = align_real_to_synthetic(partial_pcd, synthetic_pcd)
    # apply the same trafo on the vertebra
    partial_pcd.translate(transl)
    complete_spine.translate(transl)

    # test the object aligned selection of the vertebral bodies

    # sample complete vertebra with the poisson disk sampling technique
    pointCloudComplete = o3d.geometry.TriangleMesh.sample_points_poisson_disk(complete_spine, nrPointsProCompletePC)
    # sample partial point cloud Farthest Point Sample

    # check if partial_pcd has >= nrPointsProPartialPC points
    # if yes then sample this number directly
    # if it has at least half of the nrPointsProPartialPC duplicate the number of points then resample
    nr_points_in_partial_pcd = np.asarray(partial_pcd.points).shape[0]
    logging.debug("Initial number of points in pcd:" + str(nr_points_in_partial_pcd))
    if (nr_points_in_partial_pcd >= nrPointsProPartialPC):
        sampled_partial_pcd = fps.fps_points(np.asarray(partial_pcd.points), num_samples=nrPointsProPartialPC)
        logging.debug("Number of points after sampling: " + str(sampled_partial_pcd.shape[0]))
    else:
        logging.debug(
            "PCD with less than " + str(nrPointsProPartialPC) + "points " + str(os.path.basename(pathToPartialPCD)))
        return 0, [], []


    if (visualize):
        coord_sys = o3d.geometry.TriangleMesh.create_coordinate_frame()
        # create pcd from the sampling
        pcd_subsampled = o3d.geometry.PointCloud()
        pcd_subsampled.points = o3d.utility.Vector3dVector(sampled_partial_pcd)

        logging.debug("Visualizing input and ground truth after scaling and centering")
        pointCloudComplete.paint_uniform_color([0, 1, 0])
        o3d.io.write_point_cloud("complete.pcd", pointCloudComplete)

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


def saveToH5(fileName, stackedCropped, stackedComplete, datasets_ids, nrSamplesPerClass=16):
    # save the dataset in a .h5 file for VRCNet
    vertebrae_file = h5py.File(fileName, "w")
    dset_incompletepcds = vertebrae_file.create_dataset("incomplete_pcds", data=stackedCropped)
    dset_completepcds = vertebrae_file.create_dataset("complete_pcds", data=stackedComplete)
    dset_ids = vertebrae_file.create_dataset("datasets_ids", data=datasets_ids)


def processAllSpines(list_path, root_directory_spines, saveTo,
                     nr_deform_per_sample, nr_shifts_per_sample, visualize=False, nrPointsProPartialPC=2048,
                     nrPointsProCompletePC=4096):
    # prepare lists for storing all vertebrae
    labels = []
    complete_pcds_all_spines = []
    partial_pcds_all_spines = []
    labelmaps = []
    dataset_ids = []

    # create a list with all spines names
    with open(os.path.join(list_path)) as file:
        model_list = [line.strip() for line in file]

    idx = 0

    # iterate over the vertebrae names
    for model_id in model_list:
        spine_folder_name = model_id
        shift_root = os.path.join(root_directory_spines, spine_folder_name, "shifts", spine_folder_name + "_lumbar_msh_centered_scaled")
        shift_folders = sorted(os.listdir(shift_root))

        if (len(shift_folders) != int(nr_shifts_per_sample)):
            raise Exception(
                "Number of found shift folders: " + str(
                    len(shift_folders)) + " does not match the number of given shifts per sample for" + str(
                    model_id))

        for shift in range(int(nr_shifts_per_sample)):

            logging.debug(str(idx) + "/" + str(len(model_list) * int(nr_shifts_per_sample)))
            logging.debug("Processing " + str(model_id) + " and shift: " + str(shift))

            # get the name of the pcd and the name of the mesh
            polluted_pcd_path_spine = os.path.join(root_directory_spines, spine_folder_name, "shifts",spine_folder_name + "_lumbar_msh_centered_scaled",
                                                   shift_folders[shift],"account_for_shadow.pcd")
            spine_mesh_path = os.path.join(root_directory_spines, spine_folder_name, spine_folder_name + "_lumbar_msh_centered_scaled.obj")

            #  process each spine individually
            #labelmap_path = find_file_in_folder_with_unique_identifier(
            #    os.path.join(root_directory_spines, spine_folder_name, "labelmap"),
            #    spine_folder_name + "*labelmap*_centered_scaled*.obj")

            complete_pcd, partial_pcds = processOneSpine(path_complete_spine=spine_mesh_path,
                                                                      pathToPartialPCD=polluted_pcd_path_spine,
                                                                      visualize=visualize,
                                                                      nrPointsProPartialPC=nrPointsProPartialPC,
                                                                      nrPointsProCompletePC=nrPointsProCompletePC)

            # if the partial point cloud has less than nrPointsProPartialPC then partial_pcds will be an empty list
            if len(partial_pcds) == 0:
                continue

            logging.debug(partial_pcds[0].shape)
            # add it to h5py
            # make sure that the smallest label will be 0
            complete_pcds_all_spines.append(complete_pcd)
            partial_pcds_all_spines.extend(partial_pcds)
            dataset_ids.append((model_id).encode("ascii"))

            # size of labels = size of all_partial_pcds
            idx += 1

    # stack the results
    stacked_partial_pcds = np.stack(partial_pcds_all_spines, axis=0)
    stacked_complete_pcds = np.stack(complete_pcds_all_spines, axis=0)
    stacked_dataset_ids = np.stack(dataset_ids, axis=0)

    # for debugging logging.debug the shape
    logging.debug("Shape of stacked_partial_pcds: " + str(stacked_partial_pcds.shape))
    logging.debug("Shape of stacked_complete_pcds" + str(stacked_complete_pcds.shape))

    saveToH5(saveTo, stackedCropped=stacked_partial_pcds, stackedComplete=stacked_complete_pcds,
        datasets_ids=stacked_dataset_ids, nrSamplesPerClass=1)


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser(description="Create a dataset for completion training")
    arg_parser.add_argument(
        "--spine_list",
        required=True,
        dest="spine_list",
        help="Txt file with a list of spines"
    )
    arg_parser.add_argument(
        "--root_path_spines",
        required=True,
        dest="root_path_spines",
        help="Root path of the spine folders"

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
        "--visualize",
        action="store_true",
        default=False,
        help="Visualize spines before they are added to the dataset"
    )

    args = arg_parser.parse_args()
    logging.debug("Creating the shape completion dataset from partial point clouds obtained from US")
    timestr = time.strftime("%Y%m%d-%H%M%S")

    logging.basicConfig(filename="create_dataset_logs" + timestr + ".txt", filemode='w', level=logging.DEBUG,
                        force=True)

    # process all vertebrae
    processAllSpines(list_path=args.spine_list,
                     root_directory_spines=args.root_path_spines,
                     saveTo=args.result_h5_file,
                     nr_deform_per_sample=int(args.nr_deform_per_sample),
                     visualize=args.visualize,
                     nrPointsProPartialPC=int(args.nr_points_per_point_cloud),
                     nrPointsProCompletePC=int(args.nr_points_per_point_cloud),
                     nr_shifts_per_sample=int(args.num_shifts)
                     )
