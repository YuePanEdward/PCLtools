#!/bin/sh
sequence_num=013

pc_folder=/home/panyue/data/kitti_demo/demo_pc/${sequence_num}
bbx_folder=/home/panyue/data/kitti_demo/demo_bbx/${sequence_num}

rm ${pc_folder}_filelist.txt
ls ${pc_folder} >> ${pc_folder}_filelist.txt

rm ${bbx_folder}_filelist.txt
ls ${bbx_folder} >> ${bbx_folder}_filelist.txt

./bin/odetect ${pc_folder} ${bbx_folder}
