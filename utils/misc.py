import os
from utils import namings
import glob


def create_list_init_shifted_merged_for_raycasting(spines_root, list_spines, save_to):
    list = open(save_to, "w")

    with open(list_spines) as file:
        spines_list = file.read().splitlines()

    for spine in spines_list:
        folder_name = spine.split("_")[0]
        init_spine = os.path.join(spines_root, folder_name, namings.get_name_spine_lumbar_mesh_scaled_centered(spine))
        shifted_spine = os.path.join(spines_root, folder_name,
                                     namings.get_name_spine_lumbar_mesh_scaled_centered_shifted(spine))
        merged_spine = os.path.join(spines_root, folder_name,
                                    namings.get_name_spine_lumbar_mesh_scaled_centered_merged(spine))

        list.write(init_spine + "\n")
        list.write(shifted_spine + "\n")
        list.write(merged_spine + "\n")


def create_list_init_shifted_merged_deformed_for_raycasting(spines_root, list_spines, num_deform, num_shifts, save_to):
    list = open(save_to, "w")

    with open(list_spines) as file:
        spines_list = file.read().splitlines()

    for spine in spines_list:
        folder_name = spine.split("_")[0]

        for deform in range(int(num_deform)):
            curr_lumbar_mesh_name = namings.get_name_spine_lumbar_mesh_deformed_scaled_centered(spine, deform)
            shifts_path = os.path.join(spines_root, folder_name, "shifts", curr_lumbar_mesh_name.split(".")[0])
            shifts_directories = os.listdir(shifts_path)
            init_spine = os.path.join(spines_root, folder_name, curr_lumbar_mesh_name)
            if (os.path.isfile(init_spine)):
                list.write(init_spine + "\n")

            for shift in range(int(num_shifts)):
                # add merged_spine_posx
                merged_spine_posx = os.path.join(shifts_path,
                                            shifts_directories[shift],
                                            namings.get_name_spine_lumbar_mesh_deformed_scaled_centered_posxmerged(spine,
                                                                                                               deform))
                # add merged_spine_negx
                merged_spine_negx = os.path.join(shifts_path,
                                            shifts_directories[shift],
                                            namings.get_name_spine_lumbar_mesh_deformed_scaled_centered_negxmerged(spine,
                                                                                                                deform))
                if (os.path.isfile(merged_spine_posx)):
                    list.write(merged_spine_posx + "\n")
                if (os.path.isfile(merged_spine_negx)):
                    list.write(merged_spine_negx + "\n")

def create_list_init_deformed_for_raycasting(spines_root, list_spines, num_deform, save_to):
    list = open(save_to, "w")

    with open(list_spines) as file:
        spines_list = file.read().splitlines()

    for spine in spines_list:
        folder_name = spine.split("_")[0]

        for deform in range(int(num_deform)):
            curr_lumbar_mesh_name = namings.get_name_spine_lumbar_mesh_deformed_scaled_centered(spine, deform)
            init_spine = os.path.join(spines_root, folder_name, curr_lumbar_mesh_name)
            if (os.path.isfile(init_spine)):
                list.write(init_spine + "\n")

def create_list_init_shifted_merged_for_raycasting(spines_root, list_spines, num_shifts, save_to):
    list = open(save_to, "w")

    with open(list_spines) as file:
        spines_list = file.read().splitlines()

    for spine in spines_list:
        folder_name = spine.split("_")[0]

        curr_lumbar_mesh_name = namings.get_name_spine_lumbar_mesh_scaled_centered(spine)
        shifts_path = os.path.join(spines_root, folder_name, "shifts", curr_lumbar_mesh_name.split(".")[0])
        shifts_directories = os.listdir(shifts_path)
        init_spine = os.path.join(spines_root, folder_name, curr_lumbar_mesh_name)
        if (os.path.isfile(init_spine)):
            list.write(init_spine + "\n")

        for shift in range(int(num_shifts)):
            # add merged_spine_posx
            merged_spine_posx = os.path.join(shifts_path,
                                        shifts_directories[shift],
                                        namings.get_name_spine_lumbar_mesh_scaled_centered_posxmerged(spine))
            # add merged_spine_negx
            merged_spine_negx = os.path.join(shifts_path,
                                        shifts_directories[shift],
                                        namings.get_name_spine_lumbar_mesh_scaled_centered_negxmerged(spine))
            if (os.path.isfile(merged_spine_posx)):
                list.write(merged_spine_posx + "\n")
            if (os.path.isfile(merged_spine_negx)):
                list.write(merged_spine_negx + "\n")

def write_list_row_by_row_to_txt_file(list, file):
    for elem in list:
        file.write(elem + "\n")


def create_list_all_deformed_scaled_spines_from_spineid(list_of_spine_ids, save_to, root_spines, num_deform):
    with open(list_of_spine_ids) as file:
        spine_ids = file.read().splitlines()

    list = open(save_to, "w")
    for spine_id in spine_ids:
        # find the paths of all deformed spines
        paths_deformed_spine = namings.get_paths_deformed_scaled_lumbar_spines(root_spines, spine_id, num_deform)
        write_list_row_by_row_to_txt_file(paths_deformed_spine, list)

def create_list_all_scaled_spines_from_spineid(list_of_spine_ids, save_to, root_spines):
    with open(list_of_spine_ids) as file:
        spine_ids = file.read().splitlines()
    list = open(save_to, "w")
    for spine_id in spine_ids:
        # find the paths of all deformed spines
        list.write(namings.get_path_centered_scaled_lumbar_spine(root_spines, spine_id) + "\n")


def create_list_all_deformed_vert_and_spines_from_spineid(list_of_spine_ids, save_to, root_vert, root_spines,
                                                          num_deform):
    with open(list_of_spine_ids) as file:
        spine_ids = file.read().splitlines()

    list = open(save_to, "w")

    for spine_id in spine_ids:
        # find the paths of all deformed spines
        paths_deformed_spine = namings.get_paths_deformed_lumbar_spines(root_spines, spine_id, num_deform)
        paths_deformed_vertebrae = namings.get_paths_deformed_vertebrae(root_vert, spine_id, num_deform)

        for deform in range(int(num_deform)):
            list.write(paths_deformed_spine[deform] + "\n")
            vert_of_current_deform = [path for path in paths_deformed_vertebrae if "forces" + str(deform) in path]
            write_list_row_by_row_to_txt_file(vert_of_current_deform, list)

def create_list_all_vert_and_spines_from_spineid(list_of_spine_ids, save_to, root_vert, root_spines):
    with open(list_of_spine_ids) as file:
        spine_ids = file.read().splitlines()

    list = open(save_to, "w")

    for spine_id in spine_ids:
        # find the paths of all deformed spines
        paths_deformed_spine = namings.get_path_lumbar_spine_centered(root_spines, spine_id)
        paths_deformed_vertebrae = namings.get_paths_vertebrae_centered(root_vert, spine_id)

        list.write(paths_deformed_spine + "\n")
        write_list_row_by_row_to_txt_file(paths_deformed_vertebrae, list)
def create_list_all_deformed_scaled_vert_and_spines_from_spineid(list_of_spine_ids, save_to, root_vert, root_spines,
                                                                 num_deform):
    with open(list_of_spine_ids) as file:
        spine_ids = file.read().splitlines()

    list = open(save_to, "w")

    for spine_id in spine_ids:
        # find the paths of all deformed spines
        paths_deformed_spine = namings.get_paths_deformed_scaled_lumbar_spines(root_spines, spine_id, num_deform)
        paths_deformed_vertebrae = namings.get_paths_deformed_scaled_vertebrae(root_vert, spine_id, num_deform)

        for deform in range(int(num_deform)):
            list.write(paths_deformed_spine[deform] + "\n")
            vert_of_current_deform = [path for path in paths_deformed_vertebrae if "forces" + str(deform) in path]
            write_list_row_by_row_to_txt_file(vert_of_current_deform, list)

def create_list_all_scaled_vert_and_spines_from_spineid(list_of_spine_ids, save_to, root_vert, root_spines):
    with open(list_of_spine_ids) as file:
        spine_ids = file.read().splitlines()

    list = open(save_to, "w")

    for spine_id in spine_ids:
        # find the paths of all deformed spines
        path_spine = namings.get_path_centered_scaled_lumbar_spine(root_spines, spine_id)
        paths_vertebrae = namings.get_paths_vertebrae_scaled(root_vert, spine_id)
        list.write(path_spine + "\n")
        write_list_row_by_row_to_txt_file(paths_vertebrae, list)


def delete_paths(list_of_paths):
    with open(list_of_paths) as file:
        paths = file.read().splitlines()

    for path in paths:
        if "negx" in path or "posx" in path:
            os.remove(path)


