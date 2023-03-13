import os
import sys
import argparse
import glob
import nibabel as nib
import numpy as np
import napari


def process_data_pair(path_us, path_segm, visualize):
    # print current data name
    name_file = os.path.basename(path_us)
    print("Transforming: " + name_file)

    # load segm
    segm = nib.load(path_segm)
    segm_array = segm.get_fdata()

    # load US
    us = nib.load(path_us)
    us_array = us.get_fdata()

    # segm arrays have a different shape
    segm_array = segm_array[:, :, 0, :]

    if(segm_array.shape != us_array.shape):
        raise Exception("US and its segm do not have the same shape for: " + name_file)

    # iterate over the segm slices, if they do not have a segm delete from both us and segm

    for indx_slice in range(segm_array.shape[2]):
        if segm_array[:,:,indx_slice].max() == 0:
            print("Deleted: " + str(indx_slice))
            np.delete(arr=segm_array,obj=indx_slice,axis=0)
            np.delete(arr=us_array,obj=indx_slice,axis=0)

    # swap axes for segm
    segm_array = np.swapaxes(segm_array, 0, 2)
    segm_array = np.flip(segm_array, axis=1)

    # swap axis for ultrasound
    us_array = np.swapaxes(us_array, 0, 2)
    us_array = np.flip(us_array, axis=1)

    # expand the last dim for the us and the segmentation
    us_array = us_array[:, :, :, np.newaxis]
    segm_array = segm_array[:, :, :, np.newaxis]

    print("Shape of the us: " + str(us_array.shape))
    print("Shape of the segm: " + str(segm_array.shape))

    if(visualize):
        viewer = napari.view_image(us_array, channel_axis=3)
        napari.run()

        viewer = napari.view_image(segm_array, channel_axis=3)
        napari.run()
    # save the us and the segm
    np.save(path_us.replace(".nii.gz",".npy"), us_array)
    np.save(path_segm.replace(".nii.gz",".npy"), segm_array)


if __name__ == '__main__':
    root_path = "/home/miruna20/Documents/Thesis/Dataset/Patients/US"
    visualize = True

    # look for all of the segmentations
    unique_identifier = "*segm.nii.gz"
    paths_segms = sorted(glob.glob(os.path.join(root_path, unique_identifier), recursive=True))

    # for each segmentation process both the segmentation and the original ultrasound
    for path_segm in paths_segms:
        # look for US which is paired with the current segmentation
        path_us = path_segm.replace("_segm.nii.gz", ".nii.gz")
        process_data_pair(path_us,path_segm,visualize)
