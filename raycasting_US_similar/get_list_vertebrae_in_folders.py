import os
import argparse
from utils import namings

if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser(description="Get a list of all vertebra ids from a folder which contains at least one pointcloud file")

    arg_parser.add_argument(
        "--root_path_vertebrae",
        required=True,
        dest="root_path_vertebrae",
        help="Root path to the vertebrae folders."
    )
    arg_parser.add_argument(
        "--vert_list_to_save",
        required=True,
        dest="vert_list_to_save",
        help="Txt file where the vertebrae ids will be saved"
    )
    arg_parser.add_argument(
        "--list_file_names_spines",
        required=True,
        dest="txt_file_spines",
        help="Txt file that contains all spines that contain all lumbar spines"
    )
    arg_parser.add_argument(
        "--nr_deform_per_spine",
        required=True,
        dest="nr_deform_per_spine",
        help="Number of deformations per spine"
    )

    args = arg_parser.parse_args()
    root_vert = args.root_path_vertebrae

    vertebrae_file = open(args.vert_list_to_save, "w")

    # iterate over the txt file
    with open(args.txt_file_spines) as file:
        spine_ids = [line.strip() for line in file]


    # iterate over all spines
    for spine in spine_ids:
        # iterate over all vertebrae of this spine
        for i in range(20, 25):
            # iterate over all deformations of one vert
            for deform in range(int(args.nr_deform_per_spine)):
                vert_folder_name = namings.get_lumbar_vert_folder_name_from_spine(spine, i)
                # get the name of the pcd and the name of the mesh
                polluted_pcd_path = os.path.join(root_vert, vert_folder_name, namings.get_name_polluted_vert_pcd(vert_folder_name,deform))
                vert_mesh_path = os.path.join(root_vert, vert_folder_name, namings.get_name_vert_deform_scaled(vert_folder_name,deform))

                # if they both exist then add to the dataset
                if(os.path.isfile(polluted_pcd_path) and os.path.isfile(vert_mesh_path)):
                    vertebrae_file.write(os.path.basename(vert_folder_name).split(".")[0])
                    vertebrae_file.write("\n")


