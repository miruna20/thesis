import numpy as np
import glob
import os
import napari
import SimpleITK as sitk

def resample_npy(np_image_3d,visualize=False):
    new_slice_size = 256
    np_image_3d = np_image_3d.astype(np.float32)
    print("Shape np image 3d before: " + str(np_image_3d.shape))

    image = sitk.GetImageFromArray(np_image_3d)

    print("Image before has:" + str(image.GetSize()))
    resample = sitk.ResampleImageFilter()
    resample.SetInterpolator = sitk.sitkLinear
    resample.SetOutputDirection = image.GetDirection()
    resample.SetOutputOrigin = image.GetOrigin()

    orig_size = np.asarray([np_image_3d.shape[2],np_image_3d.shape[1],np_image_3d.shape[0]])
    orig_spacing = np.asarray([1,1,1])
    new_size = np.asarray([new_slice_size,new_slice_size,np_image_3d.shape[0]]) # slices x y
    new_size = np.ceil(new_size).astype(np.int)
    new_spacing = (orig_size * orig_spacing) / new_size

    print("orig_size: " + str(orig_size))
    print("new_size: " + str(new_size))
    print("new_spacing: " + str(new_spacing))

    resample.SetOutputSpacing(new_spacing.tolist())
    resample.SetSize(new_size.tolist())

    newImage = resample.Execute(image)
    print("Image after has:" + str(newImage.GetSize()))

    # visualize after resampling
    newImage_np = sitk.GetArrayFromImage(newImage)

    print("Shape resampled np array: " + str(newImage_np.shape))

    # visualize after resampling
    if(visualize):
        viewer = napari.view_image(newImage_np)
        napari.run()

    return newImage_np



if __name__ == "__main__":
    # read all npy files into numpy array
    # trafo numpy array into sitk image
    # resample sitk image so that we get 128 x 128 x nr slices
    # write the npy file back
    #root_path = "/home/miruna20/Documents/Thesis/Dataset/Slicer_plugin_dataset/DataArrays"
    root_path = "/home/miruna20/Documents/Thesis/Dataset/Patients/US/US_segm"
    unique_identifier = "*npy"
    paths_us_and_segm = sorted(glob.glob(os.path.join(root_path, unique_identifier), recursive=True))

    visualize = False
    save = True
    for path in paths_us_and_segm:

        np_image_4d = np.load(path)
        print("Shape: " + str(np_image_4d.shape))

        pad_with = np_image_4d.shape[2]-np_image_4d.shape[1]
        np_image_4d_padded = np.pad(np_image_4d, pad_width= ((0,0),
                                        (int(pad_with/2),int(pad_with/2)),
                                        (0,0),
                                        (0,0)),
                                        mode='constant',
                                        constant_values=0)
        print("Shape padded: " + str(np_image_4d_padded.shape))

        #viewer = napari.view_image(np_image_4d_padded, channel_axis=3)
        #napari.run()

        # do the resampling
        np_image_3d = np_image_4d_padded[:,:,:,0] #slices x y
        newImage_np = resample_npy(np_image_3d,visualize)
        newImage_np_extended = newImage_np[:, :, :, np.newaxis]

        #save sampled images
        if(save):
            np.save(path.replace(".npy","_resampled.npy"),newImage_np_extended)





