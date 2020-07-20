#!/bin/bash
##########################################################################################
#                                      MMLLS-LO                                          #
############################## part to configure (down)###################################
#test on KITTI Odometry Dataset (seq 00 - 10) in batch
#Download data from here: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
#Covert the point cloud from *.bin to *.pcd using ./script/tools/run_kittibin2pcd

#Ubran Scenario: 00, 02, 06, 07, 08, 09
#Country road Scenario: 03, 04, 05, 10
#Highway Scenario: 01

seq_array=(00 02 06 07 08 09 03 04 05 10 01)

for id in ${seq_array}
do

sequence_id=${id}

#experiment unique name
exp_id=${sequence_id}_f4

#project_folder=/media/edward/BackupPlus/Data/kitti-dataset/sequences/${sequence_id}
project_folder=/home/panyue/datadisk/Kitti_odom/dataset/sequences/${sequence_id}

#input path
pc_folder=${project_folder}/pcd

gt_body_pose_file=${project_folder}/${sequence_id}.txt

calib_file=${project_folder}/calib.txt

#config file
config_file=./script/config/lo_gflag_list_kitti.txt

############################### part to configure (up) ###################################

#output path
lo_adjacent_tran_file=${project_folder}/result/Rt_lo_${exp_id}.txt
lo_lidar_pose_file=${project_folder}/result/pose_l_lo_${exp_id}.txt
gt_lidar_pose_file=${project_folder}/result/pose_l_gt.txt
map_pc_folder=${project_folder}/result/map_point_clouds

mkdir ./log
mkdir ./log/test
mkdir ${project_folder}/result
mkdir ${map_pc_folder}
mkdir ${map_pc_folder}_gt

rm ${pc_folder}_filelist.txt
ls ${pc_folder} >> ${pc_folder}_filelist.txt

## assign the calib matrix for KITTI when you run it at the first time(if you use other dataset, please specify here)
#rm ${project_folder}/calib.txt
#echo "Tr: 4.276802385584e-04 -9.999672484946e-01 -8.084491683471e-03 -1.198459927713e-02 -7.210626507497e-03 8.081198471645e-03 -9.999413164504e-01 -5.403984729748e-02 9.999738645903e-01 4.859485810390e-04 -7.206933692422e-03 -2.921968648686e-01" >> ${project_folder}/calib.txt

#run (you can comment this part to directly evaluate the already calculated lidar odometry's result)
#gdb --args \
./bin/mmlls_lo \
--colorlogtostderr=true \
-stderrthreshold 0 \
-log_dir ./log/test \
--v=10 \
--point_cloud_folder=${pc_folder} \
--gt_body_pose_file_path=${gt_body_pose_file} \
--calib_file_path=${calib_file} \
--output_adjacent_lo_pose_file_path=${lo_adjacent_tran_file} \
--output_lo_lidar_pose_file_path=${lo_lidar_pose_file} \
--output_gt_lidar_pose_file_path=${gt_lidar_pose_file} \
--output_map_point_cloud_folder_path=${map_pc_folder} \
--frame_num_used=99999 \
--real_time_viewer_on=1 \
--write_out_map_on=0 \
--write_out_gt_map_on=0 \
--flagfile=${config_file}
# set the parameters in the config file 


#simple evaluation
#you need to install numpy, transforms3d, matplotlib and evo
#pip install numpy transforms3d matplotlib 
#pip install evo --upgrade --no-binary evo

ignore_z_error_or_not=false  #you'd like to invlove the error on Z direction or not (true or false)

python ./python/kitti_eval.py ${lo_adjacent_tran_file} ${gt_body_pose_file} ${calib_file} ${ignore_z_error_or_not}

evaluation_file=${project_folder}/result/Rt_lo_${exp_id}_evaluation.txt
cat ${config_file} >> ${evaluation_file}

# results (figures, table) would be output in the ./results folder

done