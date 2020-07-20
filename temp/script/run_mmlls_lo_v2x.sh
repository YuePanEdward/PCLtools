#!/bin/sh
##########################################################################################
#                                      MMLLS-LO                                          #
############################## part to configure (down)###################################
exp_id=v2x_aa

#data path
project_folder=/home/panyue/datadisk/v2x/ZhuguangRoad_middle_2019-12-3-angle_1

#input path
pc_folder=${project_folder}/pcd

gt_body_pose_file=${project_folder}/${sequence_id}.txt

calib_file=${project_folder}/calib.txt

#config file
config_file=./script/config/lo_gflag_list_v2x.txt
#config_file=./script/config/lo_gflag_list_test.txt

############################### part to configure (up) ###################################

#output path
lo_adjacent_tran_file=${project_folder}/result/Rt_lo_${exp_id}.txt
lo_lidar_pose_file=${project_folder}/result/pose_l_lo_${exp_id}.txt
gt_lidar_pose_file=${project_folder}/result/pose_l_gt.txt

mkdir ./log
mkdir ./log/test
mkdir ${project_folder}/result

rm ${pc_folder}_filelist.txt
ls ${pc_folder} >> ${pc_folder}_filelist.txt

## assign the calib matrix for KITTI when you run it at the first time(if you use other dataset, please specify here)
rm ${project_folder}/calib.txt
echo "Tr: 1 0 0 0 0 1 0 0 0 0 1 0" >> ${project_folder}/calib.txt

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
--frame_num_used=99999 \
--real_time_viewer_on=1 \
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

# replay the lidar odometry if you want
# gdb --args \
./bin/replay_slam ${pc_folder} ${lo_lidar_pose_file} ${gt_lidar_pose_file} ${evaluation_file}
