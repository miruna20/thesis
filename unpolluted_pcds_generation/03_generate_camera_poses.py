import argparse
import open3d as o3d
import os
from utils import namings

# prolly it would be a good idea to visualize all of the vertebrae together with the 3 camera positions
def generate_camera_poses( paths_ordered_vertebrae):
    """
    Generate the translation component of the camera poses. The three resulting poses are situated above L2, L3 and L4
    This function assumes that the vertebrae are already offset so that the spine that they form is centered
    """
    vertebrae_meshes = []

    # load L1, L2, L3 and L4, L5
    camera_poses = []
    for i in range(0, 5):
        # load vertebra
        vert = o3d.io.read_triangle_mesh(paths_ordered_vertebrae[i])
        vertebrae_meshes.append(vert)

        # find its center of mass
        center_vert = vert.get_center()

        # generate camera pose
        # we substract the center of spine because the individual vertebrae are not centered
        camera_pose = [center_vert[0], center_vert[2], 1]
        camera_poses.append(camera_pose)

    return camera_poses


def create_row(spine, camera_poses):
    row = spine + ";"
    nr_coord = 3
    for i in range(len(camera_poses)):
        for j in range(nr_coord):
            row = row + str(camera_poses[i][j]) + " "
        row += ";"
    row += "\n"
    return row


def create_csv_with_camera_poses(list_path_spines_and_vert, camera_poses_csv_path):

    # in this txt file the first entry is the path to the spine
    # the next 5 are the corresponding paths for the vertebrae
    with open(list_path_spines_and_vert) as file:
        spines_and_vert = file.read().splitlines()

    # open the csv file and write the first row
    csv_file = open(camera_poses_csv_path, 'w')
    csv_file.write("Name;L1;L2;L3;L4;L5;\n")

    dict = {}
    for i in range(0,int(len(spines_and_vert)/6)):
        vert_paths = []
        for vert in range(5):
            vert_paths.append(spines_and_vert[i*6+vert+1])
        camera_poses = generate_camera_poses(vert_paths)
        curr_spine_path = spines_and_vert[i*6]
        print("Generating camera pose for " + str(os.path.basename(curr_spine_path)))
        # create entry for the initial unshifted spine
        csv_file.write(create_row(os.path.basename(curr_spine_path).split(".")[0], camera_poses))

        # create entry for the merged spine
        spine = os.path.basename(curr_spine_path).split("_")[0]
        deform = os.path.basename(curr_spine_path).split("_")[1][10]

        # accounting for symmetry
        # creating entry for the posx merged spine
        name_spine_posxmerged = namings.get_name_spine_lumbar_mesh_deformed_scaled_centered_posxmerged(spine,deform)
        csv_file.write(create_row(name_spine_posxmerged.split(".")[0], camera_poses))
        
        # creating entry for the negx merged spine
        name_spine_negxmerged = namings.get_name_spine_lumbar_mesh_deformed_scaled_centered_negxmerged(spine,deform)
        csv_file.write(create_row(name_spine_negxmerged.split(".")[0], camera_poses))



if __name__ == '__main__':
    # obtain through shifting and merging
    arg_parser = argparse.ArgumentParser(
        description="Generate txt file with camera poses for raycasting")

    arg_parser.add_argument(
        "--list_spines_and_corresp_vertebrae",
        required=True,
        dest="list_spines_and_corresp_vertebrae",
        help="Txt file that contains path to the spines and the corresponding vertebrae"
    )
    arg_parser.add_argument(
        "--path_to_save_camera_poses_csv",
        required=True,
        dest="csv_file",
        help="Txt file that contains all spines for which we generate camera poses"
    )

    args = arg_parser.parse_args()
    print("Generating camera poses")

    create_csv_with_camera_poses(args.list_spines_and_corresp_vertebrae, args.csv_file)