#!/bin/sh

sequence_id=07
semantic_kitti_input_folder=/home/panyue/datadisk/Kitti_odom/dataset/sequences/${sequence_id}/
semantic_kitti_output_folder=/home/panyue/datadisk/Kitti_odom/dataset/sequences/${sequence_id}/label_pcd/

mkdir ${semantic_kitti_output_folder}

./bin/labelbin2pcd ${semantic_kitti_input_folder} ${semantic_kitti_output_folder}