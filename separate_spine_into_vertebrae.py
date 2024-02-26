import nibabel as nib
import os
import glob
import numpy as np
from timeit import default_timer as timer
import sys
import argparse

def find_file_in_folder_with_unique_identifier(folder,unique_identifier):
    files = glob.glob(os.path.join(folder,unique_identifier))
    if(len(files)!=1):
        print("More or less than 1 file were found for folder" + str(folder) + " and unique identifier: " + str(unique_identifier),
              file=sys.stderr)
        return ""
    return files[0]

def separate_spine_into_vertebrae(root_path_spines, spine_id, root_path_vertebrae,generate2DLabelmap):

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
    #for ind in range(1,len(data)):
    lumbar_levels = [20,21,22,23,24]
    for level in lumbar_levels:
        if(level in numpyData):
            print("level" + str(level))

            # if the file already exists, skip
            savePath = os.path.join(root_path_vertebrae, spine_id + "_verLev" + str(level))
            savePath = os.path.realpath(savePath)
            if (not os.path.exists(savePath)):
                os.mkdir(savePath)
            elif (not (len(os.listdir(savePath)) == 0)):
                print("This file has already been processed, the results can be found in: " + str(savePath))
                continue

            vertData_3D = np.zeros([numpyData.shape[0], numpyData.shape[1], numpyData.shape[2]])

            indices = np.transpose((numpyData == level).nonzero())

            for indic in indices:
                vertData_3D[indic[0], indic[1], indic[2]] = 1

            vertImage = nib.Nifti1Image(vertData_3D, sample.affine, sample.header)

            nib.save(vertImage,os.path.join(savePath, spine_segm_name_wo_ext + "_verLev" + str(level) + ".nii.gz"))

            # Generate 2D labelmaps by taking the middle slice
            if(generate2DLabelmap):
                # create somehow 2D label from 3D labelmap, for now select the middle slice
                # TODO think of better ways to create 2D label that simulates Xray label, maybe accumulation of values?
                vertData_2D = vertData_3D[:,:,vertData_3D.shape[2]//2]
                np.save(savePath, spine_segm_name_wo_ext + "_verLev" + str(level), vertData_2D)


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
        required=False,
        default=False,
        dest="generate2DLabelmap",
        help="Flag whether to generate 2d labelmap to be further used as aid to the completion network"

    )

    print("Separate spine segmentation into segmentation of individual vertebrae")

    args = arg_parser.parse_args()

    # iterate over the txt file and process all spines
    with open(args.txt_file) as file:
        spine_ids = [line.strip() for line in file]

    if(not os.path.exists(args.root_path_vertebrae)):
        os.mkdir(args.root_path_vertebrae)

    for spine_id in spine_ids:
        print("Separating spine: " + str(spine_id))
        try:
            separate_spine_into_vertebrae(root_path_spines=args.root_path_spines, spine_id=spine_id,
                                           root_path_vertebrae=args.root_path_vertebrae, generate2DLabelmap=args.generate2DLabelmap)
        except Exception as e:
            print("Error occured for:  " + str(spine_id) + str(e), file=sys.stderr)



