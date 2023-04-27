import os
import shutil

# Set the source directory and destination directory
src_dir = '/home/miruna20/Documents/Thesis/Dataset/VerSe2020/full_spines/subjectbased_structure/01_training'
dst_dir = '/home/miruna20/Documents/Thesis/Dataset/VerSe2020/full_spines/subjectbased_structure/01_training_duplicate'

# Iterate over all folders in the source directory
for folder in sorted(os.listdir(src_dir)):
    # Check if the current item is a directory
    if os.path.isdir(os.path.join(src_dir, folder)) and "sub-verse" in folder:

        # Check if the folder contains a subfolder named "shifts"
        if os.path.isdir(os.path.join(src_dir, folder, 'shifts')):
            # Create the destination directory if it does not exist
            dst_folder = os.path.join(dst_dir, folder)
            if not os.path.exists(dst_folder):
                os.makedirs(dst_folder,exist_ok=True)
            # Copy the "shifts" folder to the destination directory

            # aditionally also copy the obj files form forces0 and forces1
            name_file1 = os.path.join(src_dir,folder,folder + "_forcefield0_lumbar_deformed_centered_scaled.obj")
            name_file2 = os.path.join(src_dir,folder,folder + "_forcefield1_lumbar_deformed_centered_scaled.obj")
            shutil.copy(name_file1,dst_folder)
            shutil.copy(name_file2,dst_folder)
