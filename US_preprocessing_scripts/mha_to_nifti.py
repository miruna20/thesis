import os
import sys
import argparse
import glob


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser(description="Convert a mha segmentation to nii gz")

    arg_parser.add_argument(
        "--root_nifti_results",
        required=True,
        dest="root_nifti_results",
        help="Directory where the resulting nifti files will be saved"
    )
    arg_parser.add_argument(
        "--root_patients_mha_files",
        required=True,
        dest="root_patients_mha_files",
        help="Directory where initial mha segmentations can be found"
    )

    iws_file = os.path.join("imfusion_workspaces","mha_to_nifti.iws")
    args = arg_parser.parse_args()

    placeholders = ['PathToMha', 'PathToNifti']

    # gather all mha files that have CT in the path name
    unique_identifier =  "*/**/***" + "*CT*" + "*.mha"
    mha_segm_paths = sorted(glob.glob(os.path.join(args.root_patients_mha_files, unique_identifier), recursive=True))

    for mha_segm_path in mha_segm_paths:
        arguments_imfusion = ""
        for p in placeholders:

            if p == 'PathToMha':
                value = mha_segm_path

            if p == 'PathToNifti':
                name_ct = os.path.basename(mha_segm_path)
                name_patient = os.path.basename(os.path.dirname(os.path.dirname(mha_segm_path)))
                value = os.path.join(args.root_nifti_results, (name_patient + "_" + name_ct.replace(".mha",".nii.gz")).lower())

            arguments_imfusion += p + "=" + value + " "

        print('ARGUMENTS: ', arguments_imfusion)
        os.system("ImFusionSuite" + " " + iws_file + " " + arguments_imfusion)
        print('################################################### ')

