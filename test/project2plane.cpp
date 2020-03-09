// Function: Project a point set to a plane with known coefficients
// By Yue Pan @ ETH D-BAUG
// 3rd Dependent Lib: PCL

#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

int main(int argc, char **argv)
{
    
    std::string point_file_in = argv[1];
    std::string plane_file = argv[2];
    std::string point_file_out = argv[3];

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
   
    // Import points
    std::ifstream in_point(point_file_in, std::ios::in);
    if (!in_point)
    {
        return 0;
    }
    double x_ = 0, y_ = 0, z_ = 0;
    int i = 0;
    while (!in_point.eof())
    {
        in_point >> x_ >> y_ >> z_;
        if (in_point.fail())
        {
            break;
        }
        pcl::PointXYZ Pt;
        Pt.x = x_;
        Pt.y = y_;
        Pt.z = z_;
        cloud->points.push_back(Pt);
        ++i;
    }
    in_point.close();
    std::cout << "Import points for projection done ... ..." << std::endl;

    std::cout << "Points before projection: " << std::endl;
    for (std::size_t i = 0; i < cloud->points.size(); ++i)
        std::cout << "    " << cloud->points[i].x << " "
                  << cloud->points[i].y << " "
                  << cloud->points[i].z << std::endl;

    // Import the plane coefficients (a,b,c,d)
    // Plane equation: ax+by+cz+d=0
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);

    std::ifstream in_plane(plane_file, std::ios::in);
    if (!in_plane)
    {
        return 0;
    }
    in_plane >> coefficients->values[0] >> coefficients->values[1] >> coefficients->values[2] >> coefficients->values[3];
    
    in_plane.close();
    std::cout << "Import plane coefficients done ... ..." << std::endl;
    std::cout << "plane coefficients: "<<std::endl<<"a:"<<coefficients->values[0]<<" ,b:"<<coefficients->values[1] 
    <<" ,c:"<<coefficients->values[2] <<" ,d:"<<coefficients->values[3]<<std::endl;


    // Create the projection object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    std::cout << "Points after projection: " << std::endl;
    for (std::size_t i = 0; i < cloud_projected->points.size(); ++i)
        std::cout << "    " << cloud_projected->points[i].x << " "
                  << cloud_projected->points[i].y << " "
                  << cloud_projected->points[i].z << std::endl;

    std::ofstream ofs;
    ofs.open(point_file_out);
    if (ofs.is_open())
    {
        for (size_t i = 0; i < cloud_projected->size(); ++i)
        {
            ofs << setiosflags(std::ios::fixed) << std::setprecision(6) << cloud_projected->points[i].x << "  "
                << setiosflags(std::ios::fixed) << std::setprecision(6) << cloud_projected->points[i].y << "  "
                << setiosflags(std::ios::fixed) << std::setprecision(6) << cloud_projected->points[i].z
                << std::endl;
        }
        ofs.close();
    }
    else
    {
        return 0;
    }
    std::cout << "Output projected points done ... ..." << std::endl;

    return 1;
}