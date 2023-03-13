import numpy as np
from matplotlib import pyplot as plt
import matplotlib as mpl
import glob
import os
import napari
from skimage import data

if __name__ == "__main__":
    #mpl.use('TkAgg')
    root_path = "/home/miruna20/Documents/Thesis/Dataset/Patients/US/US_segm"
    unique_identifier =  "*segm_256.npy"
    paths_us_and_segm = sorted(glob.glob(os.path.join(root_path, unique_identifier), recursive=True))

    for path in paths_us_and_segm:
        segm = np.load(path)
        img = np.load(path.replace("_segm_256","_256"))

        print(path)
        print(segm.shape)
        print(img.shape)

        viewer = napari.view_image(segm, channel_axis=3)
        viewer2 = napari.view_image(img,channel_axis=3)

        napari.run()
        #np.save(path, image)




