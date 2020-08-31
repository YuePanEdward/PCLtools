#!/bin/sh

#data path
input_pointcloud_file=./data/test_pointcloud_cut_S3.pcd
grid_def_file=./data/test_grids_def.txt
output_pointcloud_folder=./data/test_pointcloud_grids_S3

./bin/gseg_pcd2grid ${input_pointcloud_file} ${grid_def_file} ${output_pointcloud_folder} 