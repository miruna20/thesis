import glob
import os
import json
import argparse
from pathlib import Path
import munch
import yaml

def get_spines_with_lumbar_vertebrae(root_folder, file):

    """
    Generate a .txt file with the names of all spines that contain all lumbar vertebrae
    :param root_folder: root folder of the spine folders
    :param file: path of txt file to be saved
    :return:
    """

    if(not os.path.exists(os.path.dirname(file))):
        os.makedirs(os.path.dirname(file))

    # gather all the json files to be able to check for lumbar vertebrae
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

    arg_parser = argparse.ArgumentParser(
        description="Generate dataset with complete and partial pointclouds from CT for shape completion")

    arg_parser.add_argument('-c', '--config', help='path to config file', required=True)

    arg = arg_parser.parse_args()
    config_path = arg.config
    args = munch.munchify(yaml.safe_load(open(config_path)))

    get_spines_with_lumbar_vertebrae(root_folder=args.root_paths_spines, file=args.list_spines)



