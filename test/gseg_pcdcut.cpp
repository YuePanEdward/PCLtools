// Function: Cut a point cloud defined by a projected bounding polygon
// By Yue Pan @ ETH D-BAUG GSEG (yuepan@student.ethz.ch)
// 3rd Dependent Lib: PCL

//TODO:
//1. Solve the intensity loss issue for pcd file
//2. Compile on Windows
//3. Complete Manual
//4. Add the example
//5. Add more formal comments

#include <iostream>
#include "utility.h"
#include "dataio.hpp"

typedef pcl::PointXYZI Point_T;

int main(int argc, char **argv)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); //Ban pcl warnings

    std::string pointcloud_file_in, bounding_polygon_file, pointcloud_file_out, ref_plane_coeff_file;
    bool fit_ref_plane_from_polygon = true;

    if (argc < 4 || argc > 5)
    {
        std::cout << "[ERROR] [MAIN] Wrong input sequence" << std::endl
                  << "The correct input sequence should be:" << std::endl
                  << "gseg_pcdcut [input point cloud pcd file] [bounding polygon file] [output point cloud pcd file] [(optional)reference plane definition file]" << std::endl
                  << "Example: gseg_pcdcut  " << std::endl; //TODO: complete
        return 0;
    }
    else
    {
        pointcloud_file_in = argv[1];
        bounding_polygon_file = argv[2];
        pointcloud_file_out = argv[3];
        if (argc == 5)
        {
            ref_plane_coeff_file = argv[4];
            fit_ref_plane_from_polygon = false;
        }
    }

    DataIo<Point_T> io;
    CloudUtility<Point_T> cu;

    pcl::PointCloud<Point_T>::Ptr cloud_in(new pcl::PointCloud<Point_T>);
    pcl::PointCloud<Point_T>::Ptr polygon_in(new pcl::PointCloud<Point_T>);
    pcl::PointCloud<Point_T>::Ptr polygon_projected(new pcl::PointCloud<Point_T>);
    pcl::PointCloud<Point_T>::Ptr cloud_cut(new pcl::PointCloud<Point_T>);

    io.readPcdPointCloud(pointcloud_file_in, cloud_in); //import the point cloud waiting for cutting

    io.readTxtPointCloud(bounding_polygon_file, polygon_in); //import the vertices of the bounding polygon

    pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients());
    if (!fit_ref_plane_from_polygon)                          //if the input reference plane coefficients are available
        io.readPlaneCoeff(ref_plane_coeff_file, plane_coeff); //import the reference plane coefficients
    else                                                      //or we need to fit the plane from the polygon vertices
    {
        if (!cu.fitPlane(polygon_in, plane_coeff, 1e-4)) //0.1 mm
            return 0;
    }

    std::vector<float> proj_dist_list;
    cu.projectCloud2Plane(polygon_in, plane_coeff, polygon_projected, proj_dist_list); //project the polygon vertices to the ref. plane

    cu.cutHull(cloud_in, polygon_projected, cloud_cut); //do the cutting

    io.writePcdPointCloud(pointcloud_file_out, cloud_cut); //output the point cloud after cutting

    return 1;
}