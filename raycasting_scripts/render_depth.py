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
import argparse
import ast
import bpy
import mathutils
import numpy as np
import os
import sys
import time
import math


def my_pose(tx, ty, tz):
    R = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    t = np.array([[tx], [ty], [tz]])
    pose = np.concatenate([np.concatenate([R, t], 1), [[0, 0, 0, 1]]], 0)
    return pose


def setup_blender(width, height, focal_length):
    # camera
    camera = bpy.data.objects['Camera']
    camera.data.angle = np.arctan(width / 2 / focal_length) * 2

    # render layer
    scene = bpy.context.scene
    scene.render.filepath = 'buffer'
    scene.render.image_settings.color_depth = '16'
    scene.render.resolution_percentage = 100
    scene.render.resolution_x = width
    scene.render.resolution_y = height

    # compositor nodes
    scene.use_nodes = True
    tree = scene.node_tree
    rl = tree.nodes.new('CompositorNodeRLayers')
    output = tree.nodes.new('CompositorNodeOutputFile')
    output.base_path = ''
    output.format.file_format = 'OPEN_EXR'
    tree.links.new(rl.outputs['Depth'], output.inputs[0])

    # remove default cube
    bpy.data.objects['Cube'].select = True
    bpy.ops.object.delete()

    return scene, camera, output


def render_obj(path, output_dir, camera_poses, intrinsics):
    """ Render an obj file given one or multiple translations of the camera poses """
    print("Path:" + path)

    model_id = os.path.basename(path).split(".")[0]
    # create exr dir and the pose dir
    exr_dir = os.path.join(output_dir, 'exr')
    pose_dir = os.path.join(output_dir, 'pose')
    os.makedirs(exr_dir,exist_ok=True)
    os.makedirs(pose_dir,exist_ok=True)

    np.savetxt(os.path.join(output_dir, 'intrinsics.txt'), intrinsics, '%f')

    bpy.ops.import_scene.obj(filepath=path)

    # this script rotates the mesh 90 degrees around X axis before raycasting. That's why even though we want to raycast from y=1 we raycast from z=1
    for i in range(len(camera_poses)):
        scene.frame_set(i)
        pose = my_pose(camera_poses[i][0], camera_poses[i][1], camera_poses[i][2])
        camera.matrix_world = mathutils.Matrix(pose)
        output.file_slots[0].path = os.path.join(exr_dir, model_id + '#.exr')
        bpy.ops.render.render(write_still=True)
        np.savetxt(os.path.join(pose_dir, model_id+str(i)+".txt"), pose, '%f')


def parse_camera_poses_from_string(camera_poses_string):
    print("Camera poses string: " + str(camera_poses_string))
    camera_pos = ast.literal_eval(camera_poses_string)
    return camera_pos


if __name__ == '__main__':

    # get passed para
    # path to a txt file that contains all of the paths that need to be processed
    list_paths = sys.argv[-2]
    # path to a csv file that contains the name of the spine_id together with camera positions from which
    # to perform the raycasting
    list_camera_poses = sys.argv[-1]

    print("Rendering depth for following meshes: ")

    # rendering parameters setup
    width = 160
    height = 120
    focal = 100
    scene, camera, output = setup_blender(width, height, focal)
    intrinsics = np.array([[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]])

    with open(list_paths) as file:
        path_list = file.read().splitlines()
    print("Path list" + str(path_list))
    open('blender.log', 'w+').close()

    # get a list with the camera poses for each model_id individually
    with open(list_camera_poses, "r") as f:
        lines = f.readlines()

    camera_poses = {}
    for l in lines[1:]:
        # a line looks like this: Name;CP1;CP2;CP3;CP4;CP5
        camera_poses[l.split(";")[0]] = [list(map(float, l.split(";")[1].split())),
                                         list(map(float, l.split(";")[2].split())),
                                         list(map(float, l.split(";")[3].split())),
                                         list(map(float, l.split(";")[4].split())),
                                         list(map(float, l.split(";")[5].split()))]

    # iterate over the list and render depth for each file individually
    for path in path_list:

        # get the basename from a path and remove the extension
        model_id = os.path.basename(path).split(".")[0]
        print("Model id:" + model_id)
        start = time.time()

        # the output directory is the base directory of our file + "rendering"
        output_dir = os.path.join(os.path.dirname(path), "rendering",model_id)
        os.makedirs(output_dir, exist_ok=True)

        # Redirect output to log file
        old_os_out = os.dup(1)
        os.close(1)
        os.open('blender.log', os.O_WRONLY)

        # get the camera poses for the current model_id from the csv file
        camera_poses_for_current_model_id = camera_poses[model_id]

        # render obj
        render_obj(path, output_dir, camera_poses_for_current_model_id, intrinsics)

        # Clean up
        bpy.ops.object.delete()
        for m in bpy.data.meshes:
            bpy.data.meshes.remove(m)
        for m in bpy.data.materials:
            m.user_clear()
            bpy.data.materials.remove(m)

        # Show time
        os.close(1)
        os.dup(old_os_out)
        os.close(old_os_out)
        print('%s done, time=%.4f sec' % (model_id, time.time() - start))
