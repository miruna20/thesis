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
        "--num_shifts",
        required=True,
        dest="num_shifts",
        help="Number of shifts for one spine"
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
            # idea make a simple list with all vert from all sppines, do all of the path verification in the create_dataset_for_shape_completion
            vertebrae_file.write(spine + "_verLev" + str(i))
            vertebrae_file.write("\n")


