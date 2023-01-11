
"""

Pipeline steps:
1. Make txt list with all spine ids of spines that contain lumbar vertebrae --> get_spines_lumbar_vertebrae.py
### For each of the spines in the list:
2. Separate spine segmentation into individual vertebrae segmentation (only for lumbar vertebrae) --> separate_spine_into_vertebrae.py
### For each vertebra:
3. Transform the vert segm(.nii.gz) into a mesh(.obj) --> convert_segmentation_into_mesh.py
4. Scale the vert mesh and center it --> scale_and_center_mesh.py
5. Create partial point clouds through raycasting that resemble partial point clouds extracted from US --> generate_partial_pointclouds.py
### Put dataset together
6. Combine partial and complete point clouds into one .h5 dataset which will be used by VRCNet for shape completion

"""