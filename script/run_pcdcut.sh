#!/bin/sh

#data path
input_pointcloud_file=./data/test_pointcloud_S3.pcd
bounding_polygon_file=./data/test_polygon.txt
plane_coefficients_file=./data/test_plane_coefficients.txt
output_pointcloud_file=./data/test_pointcloud_cut_S3.pcd

./bin/gseg_pcdcut ${input_pointcloud_file} ${bounding_polygon_file} ${output_pointcloud_file} ${plane_coefficients_file}