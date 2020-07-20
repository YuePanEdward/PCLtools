#!/bin/sh
##########################################################################################
#                                      MMLLS-LO                                          #
############################## part to configure (down)###################################
sequence_id=07

#experiment unique name
#exp_id=${sequence_id}_f14
exp_id=test_1

#data path (base folder)
project_folder=/media/edward/BackupPlus/Data/kitti-dataset/sequences/${sequence_id}
#project_folder=/home/panyue/datadisk/Kitti_odom/dataset/sequences/${sequence_id}
#project_folder=/home/panyue/data/${sequence_id}
#project_folder=/home/panyue/data/1557540003
#project_folder=/home/panyue/data/15740669
#project_folder=/home/panyue/datadisk/ros_folder/loam_hdl32/nsh_indoor_outdoor
#project_folder=/home/panyue/pandarqt_pcap/test_04
#project_folder=/home/panyue/pandarqt_pcap/test_05
#project_folder=/home/panyue/datadisk/XT_test/floor1
#project_folder=/home/panyue/pandarqt_pcap/XT_test/xijiaojiayuan1
#project_folder=/home/panyue/pandarqt_pcap/XT_test/road
#project_folder=/home/panyue/pandarqt_pcap/XT_test/1588
#project_folder=/home/panyue/pandarqt_pcap/XT_test/1588_underground
#project_folder=/home/panyue/datadisk/XT_test/underground_parkinglot
#project_folder=/home/panyue/pandarqt_pcap/QT_test/floor2_room2_QT
#project_folder=/home/panyue/pandarqt_pcap/QT_test/outdoor_1
#project_folder=/home/panyue/pandarqt_pcap/P128_test

#point cloud format (selecting from pcd, ply, las, txt, h5)
pc_format=pcd
#pc_format=ply
#pc_format=h5

#input point cloud folder path
#pc_folder=${project_folder}/label_pcd
pc_folder=${project_folder}/pcd
#pc_folder=${project_folder}/ply
#pc_folder=${project_folder}/qt_51
#pc_folder=${project_folder}/h5

#input ground truth pose file path (optional)
gt_body_pose_file=${project_folder}/${sequence_id}.txt
#gt_body_pose_file=${project_folder}/transfomed_gps_info_kitti_format.txt

#input calibration file path (optional)
calib_file=${project_folder}/calib.txt

#input config file path
config_file=./script/config/lo_gflag_list_kitti_urban.txt
#config_file=./script/config/lo_gflag_list_kitti_countryroad.txt
#config_file=./script/config/lo_gflag_list_kitti_highway.txt
#config_file=./script/config/lo_gflag_list_se_kitti.txt
#config_file=./script/config/lo_gflag_list_kitti_urban_fast.txt
#config_file=./script/config/lo_gflag_list_hesai.txt
#config_file=./script/config/lo_gflag_list_16.txt
#config_file=./script/config/lo_gflag_list_64.txt
#config_file=./script/config/lo_gflag_list_qt_lite.txt
#config_file=./script/config/lo_gflag_list_xt.txt
#config_file=./script/config/lo_gflag_list_p128.txt

#input the frame you'd like to use
frame_begin=0
frame_end=9999
frame_step=1

############################### part to configure (up) ###################################

############################### no need to edit (down) ###################################

#output path
lo_adjacent_tran_file=${project_folder}/result/Rt_lo_${exp_id}.txt
lo_lidar_pose_file=${project_folder}/result/pose_l_lo_${exp_id}.txt
lo_body_pose_file=${project_folder}/result/pose_b_lo_${exp_id}.txt
gt_lidar_pose_file=${project_folder}/result/pose_l_gt.txt
lo_lidar_pose_point_cloud=${project_folder}/result/traj_l_lo_${exp_id}.pcd
gt_lidar_pose_point_cloud=${project_folder}/result/traj_l_gt.pcd
map_pc_folder=${project_folder}/result/map_point_clouds

mkdir ./log
mkdir ./log/test
mkdir ${project_folder}/result
mkdir ${map_pc_folder}
mkdir ${map_pc_folder}_gt

rm ${pc_folder}_filelist.txt
ls ${pc_folder} >> ${pc_folder}_filelist.txt

#run (you can comment this part to directly evaluate the already calculated lidar odometry's result)
#--v : log_vebose_level, increase it to remove the logs
#gdb --args \
./bin/mmlls_lo \
--colorlogtostderr=true \
-stderrthreshold 0 \
-log_dir ./log/test \
--v=1 \
--point_cloud_folder=${pc_folder} \
--pc_format=.${pc_format} \
--gt_body_pose_file_path=${gt_body_pose_file} \
--calib_file_path=${calib_file} \
--output_adjacent_lo_pose_file_path=${lo_adjacent_tran_file} \
--output_lo_lidar_pose_file_path=${lo_lidar_pose_file} \
--output_lo_body_pose_file_path=${lo_body_pose_file} \
--output_gt_lidar_pose_file_path=${gt_lidar_pose_file} \
--output_map_point_cloud_folder_path=${map_pc_folder} \
--lo_lidar_pose_point_cloud=${lo_lidar_pose_point_cloud} \
--gt_lidar_pose_point_cloud=${gt_lidar_pose_point_cloud} \
--frame_num_begin=${frame_begin} \
--frame_num_end=${frame_end} \
--frame_step=${frame_step} \
--real_time_viewer_on=1 \
--write_out_map_on=0 \
--write_out_gt_map_on=0 \
--flagfile=${config_file}
# set the parameters in the config file 

#2010 begin

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
vis_down_rate=1
#gdb --args \
./bin/replay_slam ${pc_folder} ${lo_lidar_pose_file} ${gt_lidar_pose_file} ${evaluation_file} .${pc_format} ${frame_begin} ${frame_end} ${frame_step} ${vis_down_rate}

############################### no need to edit (up) ###################################