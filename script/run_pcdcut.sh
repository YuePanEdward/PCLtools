#!/bin/sh

#data path
input_pointcloud_file=./data/test_pointcloud_in.pcd;
bounding_polygon_file=./data/polygon.txt;
plane_coefficients_file=./data/plane_coefficients.txt;
output_pointcloud_file=./data/test_pointcloud_cut.pcd;

./bin/gseg_pcdcut ${input_pointcloud_file} ${bounding_polygon_file} ${output_pointcloud_file} ${plane_coefficients_file}