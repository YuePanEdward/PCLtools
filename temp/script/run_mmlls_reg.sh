#!/bin/sh

#data path (*.pcd, *.las, *.ply, *.txt, *.h5)
#test for global registration (--is_global_reg=true)
#tpc_path=/media/edward/Seagate/Data/Highway_dataset/test_reg/L001-1-V2-S1-C1_r_051.las
#spc_path=/media/edward/Seagate/Data/Highway_dataset/test_reg/tls2-1-geo.las
#opc_path=/media/edward/Seagate/Data/Highway_dataset/test_reg/tls2-1-geo_t.las
#station_pos=/media/edward/Seagate/Data/Highway_dataset/test_reg/tls2-1-sc.txt

tpc_path=/home/panyue/pandarqt_pcap/test_reg/128_1.h5
spc_path=/home/panyue/pandarqt_pcap/test_reg/128_2.h5
opc_path=/home/panyue/pandarqt_pcap/test_reg/128_2_t.pcd

#tpc_path=/home/panyue/pandarqt_pcap/test_reg/s6.pcd
#spc_path=/home/panyue/pandarqt_pcap/test_reg/s5.pcd
#opc_path=/home/panyue/pandarqt_pcap/test_reg/s5_t.pcd

#run
#gdb --args \
./bin/mmlls_reg \
--colorlogtostderr=true \
-stderrthreshold 0 \
-log_dir ./log/test \
--v=10 \
--point_cloud_1_path=${tpc_path} \
--point_cloud_2_path=${spc_path} \
--output_point_cloud_path=${opc_path} \
--appro_coordinate_file=${station_pos} \
--cloud_1_down_res=0.03 \
--cloud_2_down_res=0.03 \
--pca_neighbor_radius=0.9 \
--pca_neighbor_count=30 \
--gf_grid_size=3.0 \
--gf_in_grid_h_thre=0.3 \
--gf_neigh_grid_h_thre=1.5 \
--gf_max_h=5.0 \
--gf_ground_down_rate=10 \
--gf_nonground_down_rate=3 \
--linearity_thre=0.65 \
--planarity_thre=0.65 \
--curvature_thre=0.1 \
--corr_num=1000 \
--corr_dis_thre=2.5 \
--converge_tran=0.0005 \
--converge_rot_d=0.002 \
--reg_max_iter_num=25 \
--heading_change_step_degree=15 \
--realtime_viewer_on=true \
--is_global_reg=false \
--teaser_on=false
