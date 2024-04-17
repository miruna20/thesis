# Synthetic Data generation for training a shape completion network on paired incomplete/complete vertebrae 

## Install and Download
```
conda env create -f environment.yml
conda activate createCompletionDataset
```

##  Folder structure: 
    The folder structure is the following
    - <root_path_spines>/<spine_id>/ folders are already created 
        - each folder contains a segmentation file of the spine e.g sub-verse835_dir-iso_seg-vert_msk.nii.gz
        - and a json file with the labelss e.g sub-verse835_dir-iso_seg-subreg_ctd.json (only needed if you want to use the get_spines_lumbar_vertebrae.py which gives a list of all spines that contain the 5 lumbar vertebrae)

## Run data generation pipeline 
Configure the unpolluted_undeformed_pcds_generation/config.yaml file
```
cd unpolluted_undeformed_pcds_generation
python 00_apply_pipeline_one_by_one.py -c config.yaml
```


