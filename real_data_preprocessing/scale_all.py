import os
import sys
import argparse
import glob
import open3d as o3d
import numpy as np

def scale(mesh_path,scale=0.01):
    """
    Reads a mesh and scales it down to a certain scale
    """
    # read mesh
    print("Scaling " + str(mesh_path))
    mesh = o3d.io.read_triangle_mesh(mesh_path)

    # check approx that it is centered:
    vertices = np.asarray(mesh.vertices)
    pos_y_vertices = [vert for vert in vertices if vert[1] > 0]
    neg_y_vertices = [vert for vert in vertices if vert[1] < 0]
    if(len(pos_y_vertices) == 0 or len(neg_y_vertices) == 0):
        print("It might be that the following is not centered: " + mesh_path)


    # scale down
    mesh.scale(scale,center=np.asarray([0,0,0]))

    # save mesh
    o3d.io.write_triangle_mesh(mesh_path.replace(".obj", "_scaled.obj"), mesh)

if __name__ == '__main__':
    mesh_paths = ["/home/miruna20/Documents/Thesis/Dataset/Patients/CT_segm/spines/patient6_ct/patient6_mesh_centered.obj",
                  "/home/miruna20/Documents/Thesis/Dataset/Patients/CT_segm/vertebrae/patient6_ct_verLev20/patient6_ct_verLev20_msh_centered.obj",
                  "/home/miruna20/Documents/Thesis/Dataset/Patients/CT_segm/vertebrae/patient6_ct_verLev21/patient6_ct_verLev21_msh_centered.obj",
                  "/home/miruna20/Documents/Thesis/Dataset/Patients/CT_segm/vertebrae/patient6_ct_verLev22/patient6_ct_verLev22_msh_centered.obj",
                  "/home/miruna20/Documents/Thesis/Dataset/Patients/CT_segm/vertebrae/patient6_ct_verLev23/patient6_ct_verLev23_msh_centered.obj",
                  "/home/miruna20/Documents/Thesis/Dataset/Patients/CT_segm/vertebrae/patient6_ct_verLev24/patient6_ct_verLev24_msh_centered.obj"
                  ]
    for mesh in mesh_paths:
        scale(mesh)




