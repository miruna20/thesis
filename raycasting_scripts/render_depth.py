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

import bpy
import mathutils
import numpy as np
import os
import sys
import time
import math
import glob

def my_pose(tx,ty,tz):
    R = np.array([[1,0,0],[0,1,0],[0,0,1]])
    t = np.array([[tx],[ty],[tz]])
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


if __name__ == '__main__':
    model_dir = sys.argv[-4]
    list_path = sys.argv[-3]
    output_dir = sys.argv[-2]
    num_scans = int(sys.argv[-1])

    width = 160
    height = 120
    focal = 100
    scene, camera, output = setup_blender(width, height, focal)
    intrinsics = np.array([[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]])

    with open(os.path.join(list_path)) as file:
        model_list = [line.strip() for line in file]
    open('blender.log', 'w+').close()
    os.system('rm -rf %s' % output_dir)
    os.makedirs(output_dir)
    np.savetxt(os.path.join(output_dir, 'intrinsics.txt'), intrinsics, '%f')

    for model_id in model_list:
        start = time.time()
        exr_dir = os.path.join(output_dir, 'exr', model_id)
        pose_dir = os.path.join(output_dir, 'pose', model_id)
        os.makedirs(exr_dir)
        os.makedirs(pose_dir)

        # Redirect output to log file
        old_os_out = os.dup(1)
        os.close(1)
        os.open('blender.log', os.O_WRONLY)

        # Import mesh model
        # look for model path 
        unique_identifier_mesh_vertebra = "*verLev*" + "*_msh_scaled_centered.obj"
        files = glob.glob(os.path.join(os.path.join(model_dir,model_id), unique_identifier_mesh_vertebra))
        if(len(files)!=1):
            print("More or less than one mesh found for" + str(model_id) + " skipping...")
            continue
        
        bpy.ops.import_scene.obj(filepath=files[0])

        # this script rotates the mesh 90 degrees around X axis before raycasting. That's why even though we want to raycast from y=1 we raycast from z=1

        # Render
        # setup camera position parameters
        x_min = -0.3
        x_max = +0.3

        y_min = -0.3
        y_max = +0.3

        #print("ymax-ymin" + str(y_max-y_min))
        #print("num_scans"  + str(num_scans))
        y = np.arange(y_min,y_max, ((y_max - y_min) / math.sqrt(num_scans)))
        x = np.arange(x_min,x_max, ((x_max - x_min) / math.sqrt(num_scans)))
        X, Y = np.meshgrid(x, y)
        XY = np.array([X.flatten(), Y.flatten()]).T

        z = 1

        for i in range(num_scans):
            scene.frame_set(i)
            pose = my_pose(XY[i][0] ,XY[i][1], z)
            camera.matrix_world = mathutils.Matrix(pose)
            output.file_slots[0].path = os.path.join(exr_dir, '#.exr')
            bpy.ops.render.render(write_still=True)
            np.savetxt(os.path.join(pose_dir, '%d.txt' % i), pose, '%f')

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
