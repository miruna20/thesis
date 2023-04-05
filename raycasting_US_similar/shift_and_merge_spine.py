import os
import argparse
import shutil


def generate_all_combinations_of_trafo(x_shifts, y_shifts):
    """
    Given a list of x shifts and a list of y shifts return all combinations possible
    :return: all possible combinations of shifts
    """
    unique_combinations = []
    for i in range(len(x_shifts)):
        for j in range(len(y_shifts)):
            unique_combinations.append((x_shifts[i], y_shifts[j]))

    return unique_combinations


def generate_random_trafo():
    """
    Given intervals of values for shifts along x and y axis generate a random combo
    :return:
    """
    transl = [0.05, 0, 0]
    trafo = "1 0 0 " + str(transl[0]) + " 0 1 0 " + str(transl[1]) + " 0 0 1 " + str(transl[2]) + " 0 0 0 1"
    return trafo, transl


def transl_to_trafo(x, y):
    """
    Given the x and y component of the translation vector of a 4x4 matrix create the whole 4x4 matrix as a string with 3x3 identity matrix
    :param transl:
    :return:
    """
    trafo = "1 0 0 " + str(x) + " 0 1 0 " + str(y) + " 0 0 1 " + str(0) + " 0 0 0 1"
    return trafo


def shift_and_merge(trafo, path_lumbar_spine, path_to_save_merged):
    placeholders = ['PathLumbarSpine', 'Trafo', 'PathToSaveMerged']
    arguments_imfusion = ""
    workspace_file_shift_and_merge = "../imfusion_workspaces/shift_and_merge_placeholders.iws"
    for p in placeholders:
        if p == 'Trafo':
            value = '"' + str(trafo) + '"'
        if p == 'PathLumbarSpine':
            value = path_lumbar_spine
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

    # determine the values for the shifts in x and y direction
    # too small x shifts: 0.01, 0.03, too large x shifts > 0.3
    # too large y shifts > -0.1, too small shifts occur too much e.g -0.01, -0.03
    x_shifts = [ 0.05, 0.07, 0.1, 0.15, 0.2, 0.25]
    y_shifts = [-0.01, -0.03, -0.05, -0.07, -0.1]

    # larger y lead to less occlusion
    # larger x models spines that are more similar to Maria's spine

    # get all of the combinations of shifts and make sure they match the num_shifts passed
    unique_comb_of_shifts = generate_all_combinations_of_trafo(x_shifts, y_shifts)
    if (len(unique_comb_of_shifts) != int(args.num_shifts)):
        raise Exception(
            "Number of shifts passed as a parameter does not match the unique number of shifts obtained through combination of the 2 intervals")

    # iterate over the spines
    for path in paths_spines_list:

        # create folder where the shifts and merged will be saved
        base_dir = os.path.dirname(path)
        spine_id = os.path.basename(path).split(".")[0]
        path_curr_spine_and_deform = os.path.join(base_dir, "shifts", spine_id)
        # create new even if it already existed to make sure that we do not mix previous shifts with current ones
        if (os.path.isdir(path_curr_spine_and_deform)):
            shutil.rmtree(path_curr_spine_and_deform)
        # cannot exist by this point since we had just deleted it
        os.makedirs(path_curr_spine_and_deform)

        # iterate over the shifts
        for nr_shift in range(int(args.num_shifts)):
            transl = unique_comb_of_shifts[nr_shift]
            # create a folder that indicates which shifts where used
            shift_folder = os.path.join(path_curr_spine_and_deform,
                                        "shiftx" + str(transl[0]) + "_shifty" + str(transl[1]))
            os.makedirs(shift_folder, exist_ok=True)

            # get the trafo as string for +x, and y
            trafo = transl_to_trafo(abs(transl[0]), transl[1])
            shift_and_merge(trafo=trafo,
                            path_lumbar_spine=path,
                            path_to_save_merged=os.path.join(shift_folder, spine_id + "_merged.obj"))

            """
            # later on we apply the shadowing symmetrically so we need to shift and merge in pos and neg x direction
            trafo = transl_to_trafo(abs(transl[0]), transl[1])
            shift_and_merge(trafo=trafo,
                            path_lumbar_spine=path,
                            path_to_save_merged=os.path.join(shift_folder, spine_id + "posx_merged.obj"))
            # get the trafo as string for -x and y
            trafo = transl_to_trafo(-abs(transl[0]), transl[1])
            shift_and_merge(trafo=trafo,
                            path_lumbar_spine=path,
                            path_to_save_merged=os.path.join(shift_folder, spine_id + "negx_merged.obj"))

            """
