import os
import argparse

def generate_random_trafo():
    # TODO here define some ranges
    transl = [0.05, 0, 0]
    trafo = "1 0 0 " + str(transl[0]) + " 0 1 0 " + str(transl[1]) + " 0 0 1 " + str(transl[2]) + " 0 0 0 1"
    return trafo


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
    os.system("ImFusionSuite" + " " + workspace_file_shift_and_merge + " " + arguments_imfusion)
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

    args = arg_parser.parse_args()

    # read a list of the spines that we want to process
    with open(args.txt_file) as file:
        paths_spines_list = file.read().splitlines()

    for path in paths_spines_list:
        trafo = generate_random_trafo()
        spine_id = os.path.basename(path)

        # TODO extend this to work with multiple shifts
        # TODO extend this to work on multiple deformations, now the paths are set to work on the non deformed lumbar mesh
        shift_and_merge(trafo=trafo,
                        path_lumbar_spine=path,
                        path_to_save_shifted=path.replace(".obj", "_shifted.obj"),
                        path_to_save_merged=path.replace(".obj", "_merged.obj"))
