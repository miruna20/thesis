import glob
import os
import json
import argparse
from pathlib import Path

def get_spines_with_lumbar_vertebrae(root_folder, file):

    """
    Generate a .txt file with the names of all spines that contain all lumbar vertebrae
    :param root_folder: root folder of the spine folders
    :param file: path of txt file to be saved
    :return:
    """

    if(not os.path.exists(os.path.dirname(file))):
        os.makedirs(os.path.dirname(file))

    # gather all of the json files to be able to check for lumbar vertebrae
    filenames = []
    for path in sorted(Path(os.path.join(root_folder)).rglob('*.json')):
        filenames.append(str(path))

    lumbar_spines_file = open(file, "w")

    # iterate over them and check if they have L1 until L5 present
    lumbar_labels = [20,21,22,23,24]
    for json_file in filenames:
        contained_lumbar_vert = []
        f = open(json_file)
        data = json.load(f)
        for i in range(1,len(data)):
            if 20 <= data[i]["label"] <= 24:
                contained_lumbar_vert.append(data[i]["label"])

        # if all lumbar vertebrae are contained
        if contained_lumbar_vert == lumbar_labels:
            path_to_json_file = os.path.dirname(json_file)
            lumbar_spines_file.write(os.path.basename(path_to_json_file))
            lumbar_spines_file.write("\n")


if __name__ == "__main__":

    # example setup
    """
    root_folder = "/home/miruna20/Documents/Thesis/Dataset/VerSe2020/full_spines/subjectbased_structure/01_training"
    file = "../samples/lumbar_spines.txt"
    
    """
    arg_parser = argparse.ArgumentParser(description="Generate txt file with the names of the spines that contain all lumbar vertebrae")

    arg_parser.add_argument(
        "--root_path_spines",
        required=True,
        dest="root_path_spines",
        help="Root path to the vertebrae folders."
    )

    arg_parser.add_argument(
        "--list_file_names",
        required=True,
        dest="txt_file",
        help="File where the names of the lumbar vertebrae will be written"
    )

    args = arg_parser.parse_args()
    print("Creating a txt list with the spine ids of all spines that contain lumbar vertebrae")

    get_spines_with_lumbar_vertebrae(root_folder=args.root_path_spines, file=args.txt_file)



