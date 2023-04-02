'''
MIT License

Copyright (c) 2018 Wentao Yuan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

import Imath
import OpenEXR
import argparse
import array
import numpy as np
import os
import open3d


def read_exr(exr_path, height, width):
    file = OpenEXR.InputFile(exr_path)
    depth_arr = array.array('f', file.channel('R', Imath.PixelType(Imath.PixelType.FLOAT)))
    depth = np.array(depth_arr).reshape((height, width))
    depth[depth < 0] = 0
    depth[np.isinf(depth)] = 0
    return depth

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::
            angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            angle_between((1, 0, 0), (1, 0, 0))
            0.0
            angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def compute_incidence_angles(pose,points,filter_by_incidence_angle):
    camera_pos = pose[:,-1][0:3]

    # estimate normals for the pointcloud with open3d
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(points)
    pcd.estimate_normals()

    incidence_angles = []
    filtered_points = []

    # iterate over each point
    for indx in range(points.shape[0]):
        # compute ray
        curr_point = points[indx]
        ray = camera_pos - curr_point

        # get normal
        normal = np.asarray(pcd.normals)[indx]

        # get angle
        normal_angle = angle_between(ray, normal)
        tangent_angle = np.deg2rad(90) - normal_angle
        if(not filter_by_incidence_angle):
            incidence_angles.append([tangent_angle,tangent_angle,tangent_angle])
            filtered_points.append(curr_point.tolist())
        if(filter_by_incidence_angle and -np.deg2rad(90) < normal_angle < np.deg2rad(90)):
            incidence_angles.append([tangent_angle,tangent_angle,tangent_angle])
            filtered_points.append(curr_point.tolist())
    sphere = open3d.geometry.TriangleMesh.create_sphere(radius=0.1)
    coord_sys = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    sphere = sphere.translate(camera_pos)
    sphere.paint_uniform_color([1, 0.706, 0])

    #open3d.visualization.draw([pcd,sphere,coord_sys])

    return np.asarray(filtered_points), np.asarray(incidence_angles)

def depth_to_pcd(depth, intrinsics, pose, filter_by_incidence_angle):
    inv_K = np.linalg.inv(intrinsics)
    inv_K[2, 2] = -1
    depth = np.flipud(depth)
    y, x = np.where(depth > 0)
    # image coordinates -> camera coordinates
    points = np.dot(inv_K, np.stack([x, y, np.ones_like(x)] * depth[y, x], 0))
    # camera coordinates -> world coordinates
    points = np.dot(pose, np.concatenate([points, np.ones((1, points.shape[1]))], 0)).T[:, :3]

    # compute incidence angles
    points, incidence_angles = compute_incidence_angles(pose,points, filter_by_incidence_angle)

    return points, incidence_angles

def process_exr(path, output_dir, num_scans, filter_by_incidence_angle):
    """
    Process all .exr files for one model
    """
    print("Transforming depth image to pcd for: ")
    model_id = os.path.basename(path).split(".")[0]
    print(model_id)

    # create the directories for depth files and for the point clouds
    depth_dir = os.path.join(output_dir, model_id, 'depth')
    pcd_dir = os.path.join(output_dir, model_id, 'pcd')
    os.makedirs(depth_dir, exist_ok=True)
    os.makedirs(pcd_dir, exist_ok=True)

    # read the intrinsics file and get height and width
    intrinsics = np.loadtxt(os.path.join(output_dir,model_id,"intrinsics.txt"))
    width = int(intrinsics[0, 2] * 2)
    height = int(intrinsics[1, 2] * 2)

    for i in range(num_scans):
        print("Num_scan:" + str(i))
        # find paths where the exr and the corresponding paths were found
        exr_path = os.path.join(output_dir,model_id, 'exr', model_id + str(i) + ".exr")
        pose_path = os.path.join(output_dir,model_id, 'pose', model_id + str(i) + ".txt")

        # create and write depth image
        depth = read_exr(exr_path, height, width)
        depth_img = open3d.geometry.Image(np.uint16(depth * 1000))
        open3d.io.write_image(os.path.join(depth_dir, model_id + str(i) + ".png"), depth_img)

        pose = np.loadtxt(pose_path)
        points, incidence_angles = depth_to_pcd(depth, intrinsics, pose, filter_by_incidence_angle)
        pcd = open3d.geometry.PointCloud()
        pcd.points = open3d.utility.Vector3dVector(points)

        # save the incidence angle as normals to be able to save it in the pointcloud format
        pcd.normals = open3d.utility.Vector3dVector(incidence_angles)

        # save as ply to be able to also save angles
        open3d.io.write_point_cloud(os.path.join(pcd_dir, model_id + "_" + str(i) + ".pcd"), pcd)


if __name__ == '__main__':

    # get the parameters
    parser = argparse.ArgumentParser()
    parser.add_argument('list_file')
    parser.add_argument('num_scans', type=int)
    args = parser.parse_args()

    filter_by_incidence_angle = True

    # pass a list of paths instead of a list of model names
    with open(args.list_file) as file:
        path_list = file.read().splitlines()

    # for every path
    for path in path_list:
        output_dir = os.path.join(os.path.dirname(path),"rendering")
        process_exr(path,output_dir,args.num_scans,filter_by_incidence_angle)

