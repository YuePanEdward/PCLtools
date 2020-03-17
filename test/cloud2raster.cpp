// Function: Cut the wall into many subdivisions according to the raster
// By Yue Pan @ ETH D-BAUG
// 3rd Dependent Lib: PCL

#include <iostream>
#include <fstream>
#include "utility.h"
#include "dataio.hpp"

typedef pcl::PointXYZI Point_T;

int main(int argc, char **argv)
{
    std::cout << "Begin." << std::endl;

    std::string pointcloud_file_in = argv[1];
    std::string raster_file = argv[2];
    std::string plane_coeff_file = argv[3];
    std::string result_files_out_folder = argv[4];
    std::string georef_tran_file = argv[5];
    std::string std_file = argv[6];
    std::string dis_file = argv[7];
    std::string ia_file = argv[8]; //incidence angle
    std::string intensity_file = argv[9];
    float x_base = atof(argv[10]);
    float x_thre_dis = atof(argv[11]);

    pcl::PointCloud<Point_T>::Ptr cloud_in(new pcl::PointCloud<Point_T>);
    std::vector<pcl::PointCloud<Point_T>::Ptr> clouds_raster;

    // Import points
    DataIo<Point_T> io;
    //io.readLasFile(pointcloud_file_in, cloud_in);
    io.readTxtFile(pointcloud_file_in, cloud_in);

    // Import raster coordinates
    std::vector<bounds_t> bounds;
    io.readRasterFile(raster_file, x_base, x_thre_dis, bounds);

    CloudUtility<Point_T> cu;

    for (int i = 0; i < bounds.size(); i++)
    {
        pcl::PointCloud<Point_T>::Ptr cloud_raster(new pcl::PointCloud<Point_T>);
        cu.BbxFilter(cloud_in, cloud_raster, bounds[i]);
        clouds_raster.push_back(cloud_raster);
    }

    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
    io.readPlaneCoeff(plane_coeff_file, coeff);

    std::vector<float> raster_dist_std(bounds.size(),0);
    std::vector<float> raster_mean_intensity(bounds.size(),0);
    std::vector<float> raster_mean_distance(bounds.size(),0);
    std::vector<float> raster_mean_incidence_angle(bounds.size(),0);

    Eigen::Matrix4f tran_s2g;
    io.readTransMat(georef_tran_file, tran_s2g);

    
    for (int i = 0; i < bounds.size(); i++)
    {
        std::vector<float> dist_list;
        pcl::PointCloud<Point_T>::Ptr cloud_proj(new pcl::PointCloud<Point_T>);
        cu.projectCloud2Plane(clouds_raster[i], coeff, cloud_proj, dist_list);
        raster_dist_std[i] = cu.get_std(dist_list);
        raster_mean_intensity[i] = cu.get_mean_i(clouds_raster[i]);
        raster_mean_distance[i] = cu.get_mean_dis(clouds_raster[i], tran_s2g);
        raster_mean_incidence_angle[i] = cu.get_mean_ia(clouds_raster[i], coeff, tran_s2g);
    }

    io.outputRasterProperty(std_file, raster_dist_std);
    io.outputRasterProperty(intensity_file, raster_mean_intensity);
    io.outputRasterProperty(dis_file, raster_mean_distance);
    io.outputRasterProperty(ia_file, raster_mean_incidence_angle);

    //io.batchWritePcdFileswithStd(result_files_out_folder, clouds_raster, raster_dist_std);
    //io.batchWritePcdFiles(result_files_out_folder, clouds_raster);

    return 1;
}