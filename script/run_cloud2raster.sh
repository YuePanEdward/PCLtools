#data path
input_cloud_file=/media/edward/Seagate/Data/BonnWall/S1F1.txt;
raster_coor_file=/media/edward/Seagate/Data/BonnWall/raster.txt;
plane_coeff_file=/media/edward/Seagate/Data/BonnWall/plane_coefficients.txt;
raster_cloud_output_folder=/media/edward/Seagate/Data/BonnWall/Raster_S1F1;
georef_tran_file=/home/edward/testdata/georef_result_by_cloudcompare/S1F1_s2g.txt
std_file=/media/edward/Seagate/Data/BonnWall/raster_s1f1_std.txt;
dis_file=/media/edward/Seagate/Data/BonnWall/raster_s1f1_dis.txt;
ia_file=/media/edward/Seagate/Data/BonnWall/raster_s1f1_ia.txt;
intensity_file=/media/edward/Seagate/Data/BonnWall/raster_s1f1_i.txt;
x_base=1.0;
x_dis_thre=0.12;

#gdb --args \ 
./bin/cloud2raster ${input_cloud_file} ${raster_coor_file} ${plane_coeff_file} ${raster_cloud_output_folder} \
${georef_tran_file} ${std_file} ${dis_file} ${ia_file} ${intensity_file} ${x_base} ${x_dis_thre}
