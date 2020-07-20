#!/bin/sh

sequence_id=21
infolder=/home/panyue/datadisk/Kitti_odom/dataset/sequences/${sequence_id}
mkdir ${infolder}/pcd
for file in ${infolder}/velodyne/*.bin
do 
	./bin/bin2pcd $file ${infolder}/pcd/`basename $file .bin`.pcd
done
