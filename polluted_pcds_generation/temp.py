import numpy
import open3d as o3d
import numpy as np

# load the numpy array
# create pcd with o3d
# visualize

sampled_pcd_as_numpy_array = np.load("/home/miruna20/Documents/input.npy")
sampled_pcd_as_numpy_array = numpy.swapaxes(sampled_pcd_as_numpy_array,0,1)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(sampled_pcd_as_numpy_array)

o3d.visualization.draw_geometries([pcd])

