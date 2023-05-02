import os

### utils for spine namings
def get_name_spine_lumbar_mesh_scaled_centered(spine_id):
    return spine_id + "_lumbar_msh_centered_scaled.obj"

def get_name_spine_lumbar_mesh_deformed_scaled_centered(spine_id,deform):
    return spine_id + "_forcefield" + str(deform) + "_lumbar_deformed_centered_scaled.obj"

def get_name_spine_lumbar_mesh_scaled_centered_shifted(spine_id):
    return spine_id + "_shifted.obj"

def get_name_spine_lumbar_mesh_deformed_scaled_centered_shifted(spine_id,deform):
    return spine_id + "_forcefield" + str(deform) + "_lumbar_deformed_centered_scaled_shifted.obj"

def get_name_spine_lumbar_mesh_scaled_centered_merged(spine_id):
    return spine_id + "_merged.obj"

def get_name_spine_lumbar_mesh_deformed_scaled_centered_merged(spine_id,deform):
    return spine_id + "_forcefield" + str(deform) + "_lumbar_deformed_centered_scaled_merged.obj"

def get_name_spine_lumbar_mesh_deformed_scaled_centered_posxmerged(spine_id,deform):
    return spine_id + "_forcefield" + str(deform) + "_lumbar_deformed_centered_scaled_posx_merged.obj"

def get_name_spine_lumbar_mesh_scaled_centered_posxmerged(spine_id):
    return spine_id  + "_lumbar_msh_centered_scaled_posx_merged.obj"

def get_name_spine_lumbar_mesh_deformed_scaled_centered_negxmerged(spine_id,deform):
    return spine_id + "_forcefield" + str(deform) + "_lumbar_deformed_centered_scaled_negx_merged.obj"

def get_name_spine_lumbar_mesh_scaled_centered_negxmerged(spine_id):
    return spine_id + "_lumbar_msh_centered_scaled_negx_merged.obj"

def get_name_spine_lumbar_mesh_deformed(spine_id,deform):
    return spine_id + "_forcefield" + str(deform) + "_lumbar_deformed_centered.obj"

def get_name_spine_lumbar_mesh_deformed_scaled(spine_id,deform):
    return spine_id + "_forcefield" + str(deform) + "_lumbar_deformed_centered_scaled.obj"

def get_paths_deformed_lumbar_spines(root_spines,spine_id, num_deform):
    paths_deformed_lumbar_spines = []
    for deform in range(int(num_deform)):
        paths_deformed_lumbar_spines.append(os.path.join(root_spines,spine_id,get_name_spine_lumbar_mesh_deformed(spine_id,deform)))
    return sorted(paths_deformed_lumbar_spines)

def get_path_lumbar_spine_centered(root_spines,spine_id):
    return os.path.join(root_spines,spine_id,spine_id + "_lumbar_msh_centered.obj")

def get_path_lumbar_spine_initial(root_spines, spine_id):
    return os.path.join(root_spines,spine_id,spine_id + "_lumbar_msh.obj")


def get_paths_deformed_scaled_lumbar_spines(root_spines,spine_id, num_deform):
    paths_deformed_lumbar_spines = []
    for deform in range(int(num_deform)):
        paths_deformed_lumbar_spines.append(os.path.join(root_spines,spine_id,get_name_spine_lumbar_mesh_deformed_scaled(spine_id,deform)))
    return paths_deformed_lumbar_spines

def get_path_centered_scaled_lumbar_spine(root_spines,spine_id):
    return os.path.join(root_spines,spine_id,spine_id+"_lumbar_msh_centered_scaled.obj")

### utils for vert namings
def get_lumbar_vert_folder_name_from_spine(spine_id,id):
    return spine_id + "_verLev" + str(id)


def get_name_vert_before_deform(vert_folder_name):
    split = vert_folder_name.split("_")
    name = split[0] + "_seg-vert_msk_" + split[1] + "_msh.obj"
    return name

def get_name_vert_deform(vert_folder_name,deform):
    name = vert_folder_name + "_forces" + str(deform) + "_deformed_centered_20_0.obj"
    return name

def get_name_vert(vert_folder_name):
    name = vert_folder_name + "_msh.obj"
    return name
def get_name_vert_centered(vert_folder_name):
    name = vert_folder_name + "_msh_centered.obj"
    return name
def get_name_vert_scaled(vert_folder_name):
    name = vert_folder_name + "_msh_centered_scaled.obj"
    return name

def get_name_vert_deform_scaled(vert_folder_name,deform):
    name = vert_folder_name + "_forces" + str(deform) + "_deformed_centered_20_0_scaled.obj"
    return name

def get_name_undeformed_vert_deform_scaled(vert_folder_name):
    name = vert_folder_name + "_msh_centered_scaled.obj"
    return name


def get_name_polluted_vert_pcd(vert_folder_name,deform):
    name = vert_folder_name + "_forces" + str(deform) + "_deformed_centered_20_0_scaled.pcd"
    return name

def get_name_undeformed_polluted_vert_pcd(vert_folder_name):
    name = vert_folder_name + "_msh_centered_scaled.pcd"
    return name



def get_name_unpolluted_vert_pcd(vert_folder_name,deform):
    name = vert_folder_name + "_forces" + str(deform) + "_deformed_centered_20_0_scaled_clean.pcd"
    return name


def get_paths_vertebrae(root_vert,spine_id):
    paths_vertebrae = []
    for i in range(20,25):
        vert_folder_name = get_lumbar_vert_folder_name_from_spine(spine_id,i)
        path_vert = os.path.join(root_vert,vert_folder_name,get_name_vert_before_deform(vert_folder_name))
        paths_vertebrae.append(path_vert)

    return paths_vertebrae

def get_paths_deformed_vertebrae(root_vert,spine_id, num_deform):
    paths_vertebrae = []
    for i in range(20, 25):
        vert_folder_name = get_lumbar_vert_folder_name_from_spine(spine_id, i)
        for deform in range(int(num_deform)):
            path_vert = os.path.join(root_vert, vert_folder_name, get_name_vert_deform(vert_folder_name,deform))
            paths_vertebrae.append(path_vert)

    return sorted(paths_vertebrae)

def get_paths_vertebrae(root_vert,spine_id):
    paths_vertebrae = []
    for i in range(20, 25):
        vert_folder_name = get_lumbar_vert_folder_name_from_spine(spine_id, i)
        paths_vertebrae.append(os.path.join(root_vert,vert_folder_name,get_name_vert(vert_folder_name)))
    return paths_vertebrae

def get_paths_vertebrae_centered(root_vert,spine_id):
    paths_vertebrae = []
    for i in range(20, 25):
        vert_folder_name = get_lumbar_vert_folder_name_from_spine(spine_id, i)
        paths_vertebrae.append(os.path.join(root_vert,vert_folder_name,get_name_vert_centered(vert_folder_name)))
    return paths_vertebrae

def get_paths_vertebrae_scaled(root_vert,spine_id):
    paths_vertebrae = []
    for i in range(20, 25):
        vert_folder_name = get_lumbar_vert_folder_name_from_spine(spine_id, i)
        paths_vertebrae.append(os.path.join(root_vert,vert_folder_name,get_name_vert_scaled(vert_folder_name)))
    return paths_vertebrae


def get_paths_deformed_scaled_vertebrae(root_vert, spine_id, num_deform):
    paths_vertebrae = []
    for i in range(20, 25):
        vert_folder_name = get_lumbar_vert_folder_name_from_spine(spine_id, i)
        for deform in range(int(num_deform)):
            path_vert = os.path.join(root_vert, vert_folder_name, get_name_vert_deform_scaled(vert_folder_name,deform))
            paths_vertebrae.append(path_vert)

    return sorted(paths_vertebrae)

def get_paths_polluted_pcds(root_vert, spine_id, num_deform):
    paths_vertebrae = []
    for i in range(20, 25):
        vert_folder_name = get_lumbar_vert_folder_name_from_spine(spine_id, i)
        for deform in range(int(num_deform)):
            path_vert = os.path.join(root_vert, vert_folder_name, get_name_polluted_vert_pcd(vert_folder_name,deform))
            paths_vertebrae.append(path_vert)

    return sorted(paths_vertebrae)

def get_paths_one_deformation_scaled_vertebrae(root_vert, spine_id, deform):
    paths_vertebrae = []
    for i in range(20, 25):
        vert_folder_name = get_lumbar_vert_folder_name_from_spine(spine_id, i)
        path_vert = os.path.join(root_vert, vert_folder_name, get_name_vert_deform_scaled(vert_folder_name,deform))
        paths_vertebrae.append(path_vert)

    return sorted(paths_vertebrae)


