import nibabel as nib
import os
import glob
import numpy as np
import nibabel.processing
from timeit import default_timer as timer
import json
import sys
import argparse

def find_file_in_folder_with_unique_identifier(folder,unique_identifier):
    files = glob.glob(os.path.join(folder,unique_identifier))
    if(len(files)!=1):
        print("More or less than 1 file were found for folder" + str(folder) + " and unique identifier: " + str(unique_identifier),
              file=sys.stderr)
        return ""
    return files[0]

def separate_spine_into_vertebrae(root_path_spines, spine_id, root_path_vertebrae):

    # if the path for the vertebrae segmentations does not exist create it
    root_path_vertebrae = os.path.realpath(root_path_vertebrae)
    print("Path for vertebrae segmentation: " + root_path_vertebrae)
    if(not os.path.exists(root_path_vertebrae)):
        os.mkdir(root_path_vertebrae)

    # path of folder of the spine with spine_id
    path_spine = os.path.join(root_path_spines,spine_id)

    # get the path of the segmentation
    unique_identifier_spine = spine_id + "*_msk.nii.gz"
    spine_segm_file = find_file_in_folder_with_unique_identifier(path_spine,unique_identifier_spine)
    spine_segm_name = os.path.basename(spine_segm_file)
    spine_segm_name_wo_ext = spine_segm_name[:spine_segm_name.find('.nii.gz')]

    # get the path of the json file with the labels
    unique_identifier_json = spine_id + "*.json"
    labels = find_file_in_folder_with_unique_identifier(path_spine, unique_identifier_json)


    if(spine_segm_file==""):
        print("The segmentation file cannot be found in " + str(path_spine), file=sys.stderr)
        return

    if(labels==""):
        print("Json file cannot be found in " + str(path_spine), file=sys.stderr)
        return


    f = open(labels)
    data = json.load(f)

    # read the segmentation with nibabel
    sample = nib.load(spine_segm_file)
    numpyData = sample.get_fdata()

    # iterate over the labels in json file
    start = timer()
    for ind in range(1,len(data)):
        level = data[ind]['label']
        print("level" + str(level))

        # if the file already exists, skip
        savePath = os.path.join(root_path_vertebrae, spine_id + "_verLev" + str(level))
        savePath = os.path.realpath(savePath)
        if (not os.path.exists(savePath)):
            os.mkdir(savePath)
        elif (not (len(os.listdir(savePath)) == 0)):
            print("This file has already been processed, the results can be found in: " + str(savePath))
            continue

        vertData = np.zeros([numpyData.shape[0], numpyData.shape[1], numpyData.shape[2]])

        indices = np.transpose((numpyData == level).nonzero())

        for indic in indices:
            vertData[indic[0],indic[1], indic[2]] = 1

        vertImage = nib.Nifti1Image(vertData,sample.affine,sample.header)

        nib.save(vertImage,os.path.join(savePath, spine_segm_name_wo_ext + "_verLev" + str(level) + ".nii.gz"))
    end = timer()
    print("process took: " + str(end-start) +  " seconds")
    f.close()
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

    print("Separate spine segmentation into segmentation of individual vertebrae")

    args = arg_parser.parse_args()

    # iterate over the txt file and process all spines
    with open(args.txt_file) as file:
        spine_ids = [line.strip() for line in file]

    for spine_id in spine_ids:
        print("Separating spine: " + str(spine_id))
        try:
            separate_spine_into_vertebrae(root_path_spines=args.root_path_spines, spine_id=spine_id,
                                           root_path_vertebrae=args.root_path_vertebrae)
        except Exception as e:
            print("Error occured for:  " + str(spine_id) + str(e), file=sys.stderr)



