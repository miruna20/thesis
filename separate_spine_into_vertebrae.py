import nibabel as nib
import os
import glob
import numpy as np
from timeit import default_timer as timer
import sys
import argparse
from pathlib import Path
import json

def get_coordinates_from_json(json_data, target_label):
    for entry in json_data:
        if "label" in entry and entry["label"] == target_label:
            return [entry.get("X"), entry.get("Y"), entry.get("Z")]
    return [None, None, None]

def crop_vert(data, centroid, crop_size=(0, 128,128)):
    # Calculate starting indices for the crop
    start_indices = np.floor(centroid - np.array(crop_size) / 2).astype(int)

    # Ensure indices are within bounds
    start_indices = np.maximum(start_indices, 0)

    # Calculate ending indices for the crop
    end_indices = start_indices + np.array(crop_size)

    # Crop the 3D array
    cropped_data = data[:, start_indices[1]:end_indices[1],
                   start_indices[2]:end_indices[2]]

    # TODO check that padding works
    pad_amount = np.array([1,crop_size[1],crop_size[2]]) - np.array(cropped_data.shape)

    # Ensure that padding is non-negative (only pad if needed)
    pad_amount = np.maximum(pad_amount, 0)

    print("Pad amount: " + str(pad_amount))

    # Calculate padding for each side of the array
    pad_before = pad_amount//2
    pad_after = pad_amount - pad_before

    # Pad the array
    cropped_data = np.pad(cropped_data, ((0, 0), (pad_before[1], pad_after[1]), (pad_before[2], pad_after[2])) , mode='constant')

    return cropped_data


def generate_2D_labelmap(vert_segm_np_data, savePath, spine_segm_name_wo_ext, sample,level, centroid,patch_size):
    print("Generating labelmap")
    vertData_2D = np.sum(vert_segm_np_data,axis=0)
    vertData_2D = vertData_2D[np.newaxis,:,:]

    vertData_2D = crop_vert(vertData_2D,centroid,patch_size)

    path_labelmap_2D = os.path.join(savePath, spine_segm_name_wo_ext + "_verLev" + str(level) + "2D_labelmap.nii.gz")
    vert_image_2D = nib.Nifti1Image(vertData_2D, sample.affine)
    nib.save(vert_image_2D, path_labelmap_2D)


def find_file_in_folder_with_unique_identifier(folder,unique_identifier):
    files = glob.glob(os.path.join(folder,unique_identifier))
    if(len(files)!=1):
        print("More or less than 1 file were found for folder" + str(folder) + " and unique identifier: " + str(unique_identifier),
              file=sys.stderr)
        return ""
    return files[0]


def separate_spine_into_vertebrae(root_path_spines, spine_id, root_path_vertebrae,generate2DLabelmap,patch_size):

    # path of folder of the spine with spine_id
    path_spine = os.path.join(root_path_spines,spine_id)

    # get the path of the segmentation
    unique_identifier_spine = spine_id + "*_msk.nii.gz"
    spine_segm_file = find_file_in_folder_with_unique_identifier(path_spine,unique_identifier_spine)
    spine_segm_name = os.path.basename(spine_segm_file)
    spine_segm_name_wo_ext = spine_segm_name[:spine_segm_name.find('.nii.gz')]

    if(spine_segm_file==""):
        print("The segmentation file cannot be found in " + str(path_spine), file=sys.stderr)
        return

    # read the segmentation with nibabel
    sample = nib.load(spine_segm_file)
    numpyData = sample.get_fdata()
    # iterate over the labels in json file
    start = timer()
    lumbar_levels = [20,21,22,23,24]
    for level in lumbar_levels:
        if(level in numpyData):
            print("level" + str(level))

            # if the file already exists, skip
            savePath = os.path.join(root_path_vertebrae, spine_id + "_verLev" + str(level))

            path_segm_vert = os.path.join(savePath, spine_segm_name_wo_ext + "_verLev" + str(level) + ".nii.gz")
            if (not os.path.exists(Path(savePath))):
                os.mkdir(savePath)
            elif (os.path.exists(Path(path_segm_vert))):
                print("This file has already been processed, the results can be found in: " + str(savePath))
                if (generate2DLabelmap):
                    vert_segm_np_data = nib.load(path_segm_vert).get_fdata()
                    # load and parse json file
                    json_file = find_file_in_folder_with_unique_identifier(path_spine,spine_id + "*.json")
                    with open(json_file, "r") as json_file:
                        json_data = json.load(json_file)
                    centroid = get_coordinates_from_json(json_data,level)
                    generate_2D_labelmap(vert_segm_np_data,savePath,spine_segm_name_wo_ext,sample,level,centroid,patch_size)
                continue


            vertData_3D = np.copy(numpyData)
            vertData_3D[vertData_3D < level] = 0
            vertData_3D[vertData_3D > level] = 0

            vertImage = nib.Nifti1Image(vertData_3D, sample.affine, sample.header)

            nib.save(vertImage,path_segm_vert)

            # Generate 2D labelmaps that simulate Xray annotations
            if(generate2DLabelmap):
                json_file = find_file_in_folder_with_unique_identifier(path_spine, spine_id + "*.json")
                with open(json_file, "r") as json_file:
                    json_data = json.load(json_file)
                centroid = get_coordinates_from_json(json_data, level)
                generate_2D_labelmap(vertData_3D, savePath, spine_segm_name_wo_ext, sample,level,centroid,patch_size)

    end = timer()
    print("process took: " + str(end-start) +  " seconds")
    sample.uncache()

if __name__ == '__main__':

    # example setup:
    # rootFolderSpines = "/home/miruna20/Documents/Thesis/Dataset/VerSe2020/full_spines/01_training"
    # rootFolderVertebrae = "/home/miruna20/Documents/Thesis/sofa/vertebrae/train"

    arg_parser = argparse.ArgumentParser(description="Separate spine segmentations into segmentations of individual vertebrae")

    arg_parser.add_argument(
        "--list_file_names",
        required=True,
        dest="txt_file",
        help="Txt file that contains all spines that contain all lumbar vertebrae"
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
        "--generate2DLabelmap",
        action="store_true",
        dest="generate2DLabelmap",
        help="Flag whether to generate 2d labelmap to be further used as aid to the completion network"

    )

    print("Separate spine segmentation into segmentation of individual vertebrae")

    args = arg_parser.parse_args()
    patch_size = (0,256,256)

    # iterate over the txt file and process all spines
    with open(args.txt_file) as file:
        spine_ids = [line.strip() for line in file]

    if(not os.path.exists(args.root_path_vertebrae)):
        os.mkdir(args.root_path_vertebrae)

    for spine_id in spine_ids:
        print("Separating spine: " + str(spine_id))
        try:
            separate_spine_into_vertebrae(root_path_spines=args.root_path_spines, spine_id=spine_id,
                                           root_path_vertebrae=args.root_path_vertebrae, generate2DLabelmap=args.generate2DLabelmap, patch_size=patch_size)
        except Exception as e:
            print("Error occured for:  " + str(spine_id) + str(e), file=sys.stderr)



