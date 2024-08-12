import numpy as np
import h5py
import open3d as o3d
import math
import argparse
import copy

def visualize_side_by_side(pc_partial, pc_gt, pc_result1, pc_result2, pc_labelmap=None, shift_distance=0.5):
    pc_partial.paint_uniform_color([1, 0, 0])  # Red
    pc_gt.paint_uniform_color([0, 1, 0])  # Green
    pc_result1.paint_uniform_color([0, 0, 1])  # Blue
    pc_result2.paint_uniform_color([1, 0, 1])  # Yellow

    # Create copies and translate them along the x-axis
    pc_partial1 = copy.deepcopy(pc_partial).translate((-shift_distance, 0, 0))
    pc_gt1 = copy.deepcopy(pc_gt).translate((-shift_distance, 0, 0))
    pc_result1 = copy.deepcopy(pc_result1).translate((-shift_distance, 0, 0))

    pc_partial2 = copy.deepcopy(pc_partial).translate((shift_distance, 0, 0))
    pc_gt2 = copy.deepcopy(pc_gt).translate((shift_distance, 0, 0))
    pc_result2 = copy.deepcopy(pc_result2).translate((shift_distance, 0, 0))

    coord_sys = o3d.geometry.TriangleMesh.create_coordinate_frame()



    if pc_labelmap is not None:
        pc_labelmap.paint_uniform_color([0, 0, 0])  # Black
        pc_labelmap1 = copy.deepcopy(pc_labelmap).translate((-shift_distance, 0, 0))
        pc_labelmap2 = copy.deepcopy(pc_labelmap).translate((shift_distance, 0, 0))
        o3d.visualization.draw_geometries([pc_partial1, pc_gt1, pc_result1, pc_labelmap1, pc_partial2, pc_gt2, pc_result2, pc_labelmap2])
        #o3d.visualization.draw_geometries([pc_partial1, pc_result1, pc_labelmap1, pc_partial2, pc_result2, pc_labelmap2])
    else:
        o3d.visualization.draw_geometries([pc_partial1, pc_gt1, pc_result1, coord_sys, pc_partial2, pc_gt2, pc_result2, coord_sys])


if __name__ == "__main__":

    arg_parser = argparse.ArgumentParser(description="Visualize the input dataset and the predicted completions")
    arg_parser.add_argument(
        "--path_input_dataset",
        required=True,
        dest="path_input_dataset",
        help="Path to the input dataset e.g /path/to/vertebrae_test_lumbar_subsample.h5"
    )
    arg_parser.add_argument(
        "--path_result_dataset1",
        required=True,
        dest="path_result_dataset1",
        help="Path to the first results dataset e.g /path/to/results1.h5"
    )
    arg_parser.add_argument(
        "--path_result_dataset2",
        required=True,
        dest="path_result_dataset2",
        help="Path to the second results dataset e.g /path/to/results2.h5"
    )
    arg_parser.add_argument(
        "--nr_partial_pcds_per_sample",
        required=True,
        dest="nr_partial_pcds_per_sample",
        type=int,
        help="Number of partial point clouds per sample"
    )
    arg_parser.add_argument(
        "--sequential_visualization",
        action="store_true",
        help="Visualize the 3 point clouds (partial pc, completion, gt) one after the other (sequentially). Otherwise visualize them in one view "
    )

    args = arg_parser.parse_args()

    # Read input dataset
    inputs_inference = h5py.File(args.path_input_dataset, 'r')
    complete_pcds = np.array(inputs_inference['complete_pcds'][()])
    incomplete_pcds = np.array(inputs_inference['incomplete_pcds'][()])
    labels = np.array(inputs_inference['labels'][()]) if "labels" in inputs_inference else None
    datasets_ids = np.array(inputs_inference['datasets_ids'])
    labelmaps = np.array(inputs_inference['labelmaps']) if 'labelmaps' in inputs_inference else None

    # Read results datasets
    results1 = h5py.File(args.path_result_dataset1, 'r')
    results2 = h5py.File(args.path_result_dataset2, 'r')

    results_array1 = np.array(results1['results'][()])
    results_array2 = np.array(results2['results'][()])

    print("Red: partial pointclouds")
    print("Green: ground truth")
    print("Blue: Completion 1 --> without Xray")
    print("Yellow: Completion 2 --> with Xray segmentation before ReNet")

    factor = 10000

    try:
        emd1 = np.array(results1['emd'][()])
        emd1_arch = np.array(results1['emd_arch'][()])
        emd2 = np.array(results1['emd'][()])
        emd2_arch = np.array(results1['emd_arch'][()])
        emd_flag = True
    except:
        print("At least one of the datasets does not have emd metric")

    cd1_t = np.array(results1['cd_t'][()])
    cd1_t_arch = np.array(results1['cd_t_arch'][()])
    cd1_p = np.array(results1['cd_p'][()])
    cd1_p_arch = np.array(results1['cd_p_arch'][()])
    f1_1 = np.array(results1['f1'][()])
    f1_1_arch = np.array(results1['f1_arch'][()])

    cd2_t = np.array(results2['cd_t'][()])
    cd2_t_arch = np.array(results2['cd_t_arch'][()])
    cd2_p = np.array(results2['cd_p'][()])
    cd2_p_arch = np.array(results2['cd_p_arch'][()])
    f1_2 = np.array(results2['f1'][()])
    f1_2_arch = np.array(results2['f1_arch'][()])

    step = 4
    for i in range(0, incomplete_pcds.shape[0], step):

        if (args.path_result_dataset1 != None):
            print("Visualizing: " + str(datasets_ids[i]) + " red: partial pcd, blue: gt pcd, green: completed pcd")
            print("index: " + str(i) + ", label: " + str(labels[i]))
            if (emd_flag):
                print("emd:" + str(emd1[i] * factor))
                print("emd_arch:" + str(emd1_arch[i] * factor))
            print("cd_t:" + str(cd1_t[i] * factor))
            print("cd_t_arch:" + str(cd1_t_arch[i] * factor))
            print("f1:" + str(f1_1[i]))

        if (args.path_result_dataset2 != None):
            print("Visualizing: " + str(datasets_ids[i]) + " red: partial pcd, blue: gt pcd, green: completed pcd")
            print("index: " + str(i) + ", label: " + str(labels[i]))
            if (emd_flag):
                print("emd:" + str(emd2[i] * factor))
                print("emd_arch:" + str(emd2_arch[i] * factor))
            print("cd_t:" + str(cd2_t[i] * factor))
            print("cd_t_arch:" + str(cd2_t_arch[i] * factor))
            print("f1:" + str(f1_2[i]))


        pc_partial = o3d.geometry.PointCloud()
        pc_partial.points = o3d.utility.Vector3dVector(incomplete_pcds[i])

        pc_gt = o3d.geometry.PointCloud()
        pc_gt.points = o3d.utility.Vector3dVector(complete_pcds[math.floor(i / int(args.nr_partial_pcds_per_sample))])

        pc_result1 = o3d.geometry.PointCloud()
        pc_result1.points = o3d.utility.Vector3dVector(results_array1[i])

        pc_result2 = o3d.geometry.PointCloud()
        pc_result2.points = o3d.utility.Vector3dVector(results_array2[i])

        if labelmaps is not None:
            pc_labelmap = o3d.geometry.PointCloud()
            pc_labelmap.points = o3d.utility.Vector3dVector(labelmaps[i])
            visualize_side_by_side(pc_partial, pc_gt, pc_result1, pc_result2, pc_labelmap)
        else:
            visualize_side_by_side(pc_partial, pc_gt, pc_result1, pc_result2)
