import open3d as o3d
import numpy as np


def separate_US_pointcloud_into_vertebrae(path_to_poincloud):
    # load pointcloud with o3d
    partial_pc_spine = o3d.io.read_point_cloud(path_to_poincloud)

    # check z_max - z_min and divide it in 5 to determine the size of the box
    z_max = partial_pc_spine.get_max_bound()[2]
    z_min = partial_pc_spine.get_min_bound()[2]
    z_length = z_max - z_min
    size_box_along_z = z_length / 3  # because there are 5 vertebrae
    shift_box_by = z_length / 4
    # size of the bounding box in x and y shall be the same size of the bounding box of the
    # can we get an axis_aligned_bounding_box then create a new one that s smaller along z

    bounding_box = partial_pc_spine.get_axis_aligned_bounding_box()
    # get the 8 points of the bb
    points = bounding_box.get_box_points()

    # these are all corner points with positive z value
    corner_points_at_L1 = [point.tolist() for point in points if point[2] == z_max]

    # these are all corner points with negative z value
    corner_points_at_L5 = [point.tolist() for point in points if point[2] == z_min]

    # get box for L1:
    shifted_points = []
    for point in corner_points_at_L1:
        new_point = [point[0], point[1], point[2] - size_box_along_z]
        shifted_points.append(new_point)

    for point in corner_points_at_L1:
        new_point = [point[0], point[1], point[2]]
        shifted_points.append(new_point)

    points_to_create_bb_vect = o3d.utility.Vector3dVector(np.asarray(shifted_points))
    bb_vertebrae = o3d.geometry.AxisAlignedBoundingBox.create_from_points(points_to_create_bb_vect)
    o3d.visualization.draw([partial_pc_spine, bb_vertebrae])
    for i in range(3):
        # get box for L2:
        shifted_points_2 = []
        for point in shifted_points:
            new_point = [point[0], point[1], point[2] - shift_box_by]
            shifted_points_2.append(new_point)
        points_to_create_bb_vect = o3d.utility.Vector3dVector(np.asarray(shifted_points_2))
        bb_vertebrae = o3d.geometry.AxisAlignedBoundingBox.create_from_points(points_to_create_bb_vect)
        o3d.visualization.draw([partial_pc_spine, bb_vertebrae])
        shifted_points = shifted_points_2

    # Problem with this approach is that it does not center on the middle of the vertebral arch
    # since distances between vertebrae are different, we sometimees get 2 vert in one pointcloud
    # Idea: compute connected components, find the next highest (on y axis) connected component and closest (on z axis)


if __name__ == "__main__":
    separate_US_pointcloud_into_vertebrae(
        "/home/miruna20/Documents/Thesis/UltrasoundSimulation/SpineUltrasoundSimulation/us_extracted_pointcloud.pcd")
