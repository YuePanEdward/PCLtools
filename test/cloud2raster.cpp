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
    std::string std_file = argv[5];
    float x_base = atof(argv[6]);
    float x_thre_dis = atof(argv[7]);

    pcl::PointCloud<Point_T>::Ptr cloud_in(new pcl::PointCloud<Point_T>);
    std::vector<pcl::PointCloud<Point_T>::Ptr> clouds_raster;

    // Import points
    DataIo<Point_T> io;
    io.readLasFile(pointcloud_file_in, cloud_in);

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

    std::vector<float> raster_dist_std;
    raster_dist_std.resize(bounds.size());
    for (int i = 0; i < bounds.size(); i++)
    {
        std::vector<float> dist_list;
        pcl::PointCloud<Point_T>::Ptr cloud_proj(new pcl::PointCloud<Point_T>);
        cu.projectCloud2Plane(clouds_raster[i], coeff, cloud_proj, dist_list);
        raster_dist_std[i] = cu.get_std(dist_list);
    }

    io.outputRasterDistStd(std_file, raster_dist_std);

    io.batchWritePcdFileswithStd(result_files_out_folder, clouds_raster, raster_dist_std);
    //io.batchWritePcdFiles(result_files_out_folder, clouds_raster);

    return 1;
}