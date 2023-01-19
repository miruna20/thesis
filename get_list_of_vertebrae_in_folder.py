import os
import argparse
import glob

if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser(description="Get a list of all vertebra folders from a folder which contain a mesh")

    arg_parser.add_argument(
        "--root_path_vertebrae",
        required=True,
        dest="root_path_vertebrae",
        help="Root path to the vertebrae folders."
    )
    arg_parser.add_argument(
        "--list_file_names",
        required=True,
        dest="txt_file",
        help="Txt file where the file names will be saved"
    )

    args = arg_parser.parse_args()

    if(not os.path.exists(os.path.dirname(args.txt_file,))):
        os.makedirs(os.path.dirname(args.txt_file,))

    vertebrae_file = open(args.txt_file, "w")

    for file in sorted(os.listdir(args.root_path_vertebrae)):

        unique_identifier_mesh_vertebra = "*verLev*" + "*_msh.obj"
        files = glob.glob(os.path.join(os.path.join(args.root_path_vertebrae,file), unique_identifier_mesh_vertebra))
        if(len(files)!=0):
            spine_id_and_verLev = os.path.basename(file)
            vertebrae_file.write(spine_id_and_verLev)
            vertebrae_file.write("\n")




