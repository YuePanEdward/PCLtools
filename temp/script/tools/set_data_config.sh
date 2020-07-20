#!/bin/sh

station_position_available_=0;
station_pose_available_=0;
is_single_scanline_=0;

#############################################
echo 'process begin'
for folder in ./blocks/*
do

touch ${folder}_dataconfig.txt                                                                                                                                             

echo 'station_position_available' ${station_position_available_} >> ${folder}_dataconfig.txt
echo 'station_pose_available' ${station_pose_available_} >> ${folder}_dataconfig.txt
echo 'is_single_scanline' ${is_single_scanline_} >> ${folder}_dataconfig.txt
echo '------------------------------------------' >>${folder}_dataconfig.txt

ls ${folder} >> ${folder}_dataconfig.txt  

done
echo 'process done'




