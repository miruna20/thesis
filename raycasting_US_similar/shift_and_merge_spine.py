import os
import argparse

def generate_random_trafo():
    # TODO find a suitable range from we generate this random transformation
    transl = [0.05, 0, 0]
    trafo = "1 0 0 " + str(transl[0]) + " 0 1 0 " + str(transl[1]) + " 0 0 1 " + str(transl[2]) + " 0 0 0 1"
    return trafo, transl

def shift_and_merge(trafo, path_lumbar_spine, path_to_save_shifted, path_to_save_merged):
    placeholders = ['PathLumbarSpine', 'Trafo', 'PathToSaveShifted', 'PathToSaveMerged']
    arguments_imfusion = ""
    workspace_file_shift_and_merge = "../imfusion_workspaces/shift_and_merge_placeholders_withtrafo.iws"
    for p in placeholders:
        if p == 'Trafo':
            value = '"' + str(trafo) + '"'
        if p == 'PathLumbarSpine':
            value = path_lumbar_spine

        if p == 'PathToSaveShifted':
            value = path_to_save_shifted

        if p == 'PathToSaveMerged':
            value = path_to_save_merged

        arguments_imfusion += p + "=" + str(value) + " "

    # call imfusion with arguments
    print('ARGUMENTS: ', arguments_imfusion)
    os.system("ImFusionConsole" + " " + workspace_file_shift_and_merge + " " + arguments_imfusion)
    print('################################################### ')


if __name__ == '__main__':

    arg_parser = argparse.ArgumentParser(
        description="Generate shift and merge")

    arg_parser.add_argument(
        "--list_paths_spine",
        required=True,
        dest="txt_file",
        help="Txt file that contains all paths for which we "
    )
    arg_parser.add_argument(
        "--num_shifts",
        required=True,
        dest="num_shifts",
        help="Number of shifts applied on the spine"
    )

    args = arg_parser.parse_args()

    # read a list of the spines that we want to process
    with open(args.txt_file) as file:
        paths_spines_list = file.read().splitlines()

    # iterate over the spines
    for path in paths_spines_list:

        # create folder where the shifts and merged will be saved
        base_dir = os.path.dirname(path)
        spine_id = os.path.basename(path).split(".")[0]
        path_curr_spine_and_deform = os.path.join(base_dir,"shifts",spine_id)
        os.makedirs(path_curr_spine_and_deform,exist_ok=True)

        for nr_shift in range(int(args.num_shifts)):
            trafo, transl = generate_random_trafo()
            # create a folder that indicates which shifts where used
            shift_folder = os.path.join(path_curr_spine_and_deform,"shiftx" + str(transl[0]) + "_shifty" + str(transl[1]))
            os.makedirs(shift_folder, exist_ok=True)

            shift_and_merge(trafo=trafo,
                            path_lumbar_spine=path,
                            path_to_save_shifted=os.path.join(shift_folder,spine_id + "_shifted.obj"),
                            path_to_save_merged=os.path.join(shift_folder,spine_id + "_merged.obj"))
