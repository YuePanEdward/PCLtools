// GSEG_PCD2GRID
// Function: Split a point cloud into several grid point clouds according to a file defining the grids
// By Yue Pan @ ETH D-BAUG GSEG (yuepan@student.ethz.ch)
// 3rd Dependent Lib: PCL

#include <iostream>
#include "utility.h"
#include "dataio.hpp"

typedef pcl::PointXYZI Point_T;

int main(int argc, char **argv)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); //Ban pcl warnings

    std::string pointcloud_file_in, grid_def_file, grid_pointcloud_folder_out;

    if (argc != 4)
    {
        std::cout << "[ERROR] [MAIN] Wrong input sequence" << std::endl
                  << "The correct input sequence should be:" << std::endl
                  << "gseg_pcd2grid [input point cloud pcd file] [grid definition file] [output folder for the grid point clouds]" << std::endl;

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
        std::cout << "Example: gesg_pcd2grid [basefolder]\\test_pointcloud_cut_S1.pcd [basefolder]\\test_grids_def.txt [basefolder]\\test_pointcloud_grids_S1" << std::endl;
#else
        std::cout << "Example: gesg_pcd2grid [basefolder]/test_pointcloud_cut_S1.pcd [basefolder]/test_grids_def.txt [basefolder]/test_pointcloud_grids_S1" << std::endl;
#endif
        return 0;
    }
    else
    {
        pointcloud_file_in = argv[1];
        grid_def_file = argv[2];
        grid_pointcloud_folder_out = argv[3];
    }

    DataIo<Point_T> io;
    CloudUtility<Point_T> cu;

    pcl::PointCloud<Point_T>::Ptr cloud_in(new pcl::PointCloud<Point_T>);
    std::vector<pcl::PointCloud<Point_T>::Ptr> grids_vertice;
    std::vector<pcl::PointCloud<Point_T>::Ptr> clouds_grid;

    io.readPcdPointCloud(pointcloud_file_in, cloud_in); //import the point cloud waiting for cutting

    io.readGridFile(grid_def_file, grids_vertice);

    for (int i = 0; i < grids_vertice.size(); i++)
    {
        pcl::PointCloud<Point_T>::Ptr cloud_grid(new pcl::PointCloud<Point_T>);
        cu.cutHull(cloud_in, grids_vertice[i], cloud_grid, true); //do the cutting (without printing messages)
        clouds_grid.push_back(cloud_grid);
    }
    std::cout << "[INFO] [MAIN] Divide the point cloud into [" << grids_vertice.size() << "] grids done" << std::endl;

    io.batchWritePcdPointClouds(grid_pointcloud_folder_out, clouds_grid);

    return 1;
}