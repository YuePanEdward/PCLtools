#ifndef _INCLUDE_DATA_IO_HPP
#define _INCLUDE_DATA_IO_HPP

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

//liblas
#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>

#include <boost/filesystem.hpp>
#include <boost/function.hpp>

#include <string>
#include <fstream>
#include <vector>

#include "utility.h"

using namespace boost::filesystem;
using namespace std;

template <typename PointT>
class DataIo : public CloudUtility<PointT>
{
  public:
    bool readPcdFile(const std::string &fileName, typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        if (pcl::io::loadPCDFile<PointT>(fileName, *pointCloud) == -1)
        {
            PCL_ERROR("Couldn't read file\n");
            return false;
        }
        return true;
    }

    bool writePcdFile(const std::string &fileName, typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        if (pcl::io::savePCDFileBinary(fileName, *pointCloud) == -1)
        {
            PCL_ERROR("Couldn't write file\n");
            return false;
        }
        return true;
    }

    bool batchWritePcdFiles(const std::string &folderName, std::vector<typename pcl::PointCloud<PointT>::Ptr> &pointClouds)
    {
        if (!boost::filesystem::exists(folderName))
        {
            boost::filesystem::create_directory(folderName);
        }

        for (int i = 0; i < pointClouds.size(); i++)
        {
            ostringstream oss;
            oss.setf(ios::right);
            oss.fill('0');
            oss.width(3);
            oss << i + 1;

            std::string filename = folderName + "/" + oss.str() + ".pcd";
            writePcdFile(filename, pointClouds[i]);
        }

        std::cout << "Batch output done (" << pointClouds.size() << " files)" << std::endl;

        return 1;
    }

    bool batchWritePcdFileswithStd(const std::string &folderName, std::vector<typename pcl::PointCloud<PointT>::Ptr> &pointClouds,
                                   std::vector<float> &raster_dist_std)
    {
        if (!boost::filesystem::exists(folderName))
        {
            boost::filesystem::create_directory(folderName);
        }

        for (int i = 0; i < pointClouds.size(); i++)
        {
            ostringstream oss;
            oss.setf(ios::right);
            oss.fill('0');
            oss.width(3);
            oss << i + 1;

            std::string filename = folderName + "/" + oss.str() + ".pcd";

            for (int j = 0; j < pointClouds[i]->points.size(); j++)
            {
                pointClouds[i]->points[j].intensity = raster_dist_std[i];
            }

            writePcdFile(filename, pointClouds[i]);
        }

        std::cout << "Batch output done (" << pointClouds.size() << " files)" << std::endl;

        return 1;
    }

    bool batchWriteLasFileswithStd(const std::string &folderName, std::vector<typename pcl::PointCloud<PointT>::Ptr> &pointClouds, std::vector<float> &raster_dist_std)
    {
        if (!boost::filesystem::exists(folderName))
        {
            boost::filesystem::create_directory(folderName);
        }

        for (int i = 0; i < pointClouds.size(); i++)
        {
            ostringstream oss;
            oss.setf(ios::right);
            oss.fill('0');
            oss.width(3);
            oss << i + 1;
            std::string filename = folderName + "/" + oss.str() + ".las";

            bounds_t bound;
            this->getCloudBound(*pointClouds[i], bound);

            std::ofstream ofs;
            ofs.open(filename.c_str(), std::ios::out | std::ios::binary);
            if (ofs.is_open())
            {
                liblas::Header header;
                header.SetDataFormatId(liblas::ePointFormat2);
                header.SetVersionMajor(1);
                header.SetVersionMinor(2);
                header.SetMin(bound.min_x, bound.min_y, bound.min_z);
                header.SetMax(bound.max_x, bound.max_y, bound.max_z);
                header.SetOffset((bound.min_x + bound.max_x) / 2.0, (bound.min_y + bound.max_y) / 2.0, (bound.min_z + bound.max_z) / 2.0);
                header.SetScale(0.01, 0.01, 0.01);
                header.SetPointRecordsCount(pointClouds[i]->points.size());

                liblas::Writer writer(ofs, header);
                liblas::Point pt(&header);

                for (int j = 0; j < pointClouds[i]->points.size(); j++)
                {
                    pt.SetCoordinates(double(pointClouds[i]->points[j].x), double(pointClouds[i]->points[j].y), double(pointClouds[i]->points[j].z));
                    pt.SetIntensity(raster_dist_std[i]);

                    // bool intensity_available = pcl::traits::has_field<PointT, pcl::fields::intensity>::value;
                    // if (intensity_available)
                    // {
                    //     pt.SetIntensity(pointCloud->points[i].intensity);
                    // }

                    //If the Point template PointT is without RGB, you should comment the line.
                    //liblas::Color lasColor;
                    //lasColor.SetRed(pointCloud->points[i].r);
                    //lasColor.SetGreen(pointCloud->points[i].g);
                    //lasColor.SetBlue(pointCloud->points[i].b);
                    //pt.SetColor(lasColor);

                    writer.WritePoint(pt);
                }
                ofs.flush();
                ofs.close();
            }
        }
        std::cout << "Batch output done (" << pointClouds.size() << " files)" << std::endl;
        return 1;
    }

    bool readLasFileHeader(const std::string &fileName, liblas::Header &header)
    {
        if (fileName.substr(fileName.rfind('.')).compare(".las"))
        {
            return 0;
        }
        else
        {
            std::ifstream ifs;
            ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
            if (ifs.bad())
            {
                return 0;
            }

            liblas::ReaderFactory f;
            liblas::Reader reader = f.CreateWithStream(ifs);

            header = reader.GetHeader();
        }
        return 1;
    }

    bool readLasFile(const std::string &fileName, typename pcl::PointCloud<PointT>::Ptr &pointCloud) //Without translation
    {
        //cout << "A global translation or gravitization should be done to keep the precision of point cloud when adopting pcl to do las file point cloud processing" << endl;

        if (fileName.substr(fileName.rfind('.')).compare(".las"))
        {
            return 0;
        }

        std::ifstream ifs;
        ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
        if (ifs.bad())
        {
            std::cout << "Matched Terms are not found." << std::endl;
        }
        liblas::ReaderFactory f;
        liblas::Reader reader = f.CreateWithStream(ifs);
        liblas::Header const &header = reader.GetHeader();

        //Bounding box Information
        double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
        Xmin = header.GetMinX();
        Ymin = header.GetMinY();
        Zmin = header.GetMinZ();
        Xmax = header.GetMaxX();
        Ymax = header.GetMaxY();
        Zmax = header.GetMaxZ();

        while (reader.ReadNextPoint())
        {
            const liblas::Point &p = reader.GetPoint();
            PointT pt;
            pt.x = p.GetX();
            pt.y = p.GetY();
            pt.z = p.GetZ();

            //------------------------------------------------Assign Intensity--------------------------------------------------//
            //If the Point template PointT has intensity, you can assign the intensity with any feature of the point cloud in las.
            // bool intensity_available = pcl::traits::has_field<PointT, pcl::fields::intensity>::value;
            // if (intensity_available)
            // {
            //     pt.intensity = p.GetIntensity();
            // }

            pt.intensity = p.GetIntensity();
            //pt.intensity = p.GetTime();
            //pt.intensity = p.GetScanAngleRank();
            //pt.intensity = p.GetNumberOfReturns();
            //pt.intensity = p.GetScanDirection();

            //---------------------------------------------------Assign Color--------------------------------------------------//
            //If the Point template PointT has RGB, you can assign the Color according to the point cloud in las.
            //If the Point template PointT is without RGB, you should comment the line.
            //liblas::Color lasColor;
            //lasColor= p.GetColor();
            //pt.r = lasColor.GetRed();
            //pt.g = lasColor.GetGreen();
            //pt.b = lasColor.GetBlue();

            pointCloud->points.push_back(pt);
        }
        return 1;
    }

    bool writeLasFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud) //Without translation
    {

        bounds_t bound;
        this->getCloudBound(*pointCloud, bound);

        std::ofstream ofs;
        ofs.open(fileName.c_str(), std::ios::out | std::ios::binary);
        if (ofs.is_open())
        {
            liblas::Header header;
            header.SetDataFormatId(liblas::ePointFormat2);
            header.SetVersionMajor(1);
            header.SetVersionMinor(2);
            header.SetMin(bound.min_x, bound.min_y, bound.min_z);
            header.SetMax(bound.max_x, bound.max_y, bound.max_z);
            header.SetOffset((bound.min_x + bound.max_x) / 2.0, (bound.min_y + bound.max_y) / 2.0, (bound.min_z + bound.max_z) / 2.0);
            header.SetScale(0.01, 0.01, 0.01);
            header.SetPointRecordsCount(pointCloud->points.size());

            liblas::Writer writer(ofs, header);
            liblas::Point pt(&header);

            for (int i = 0; i < pointCloud->points.size(); i++)
            {
                pt.SetCoordinates(double(pointCloud->points[i].x), double(pointCloud->points[i].y), double(pointCloud->points[i].z));

                // bool intensity_available = pcl::traits::has_field<PointT, pcl::fields::intensity>::value;
                // if (intensity_available)
                // {
                //     pt.SetIntensity(pointCloud->points[i].intensity);
                // }

                //If the Point template PointT is without RGB, you should comment the line.
                //liblas::Color lasColor;
                //lasColor.SetRed(pointCloud->points[i].r);
                //lasColor.SetGreen(pointCloud->points[i].g);
                //lasColor.SetBlue(pointCloud->points[i].b);
                //pt.SetColor(lasColor);

                writer.WritePoint(pt);
            }
            ofs.flush();
            ofs.close();
        }
        return 1;
    }

    bool readRasterFile(const std::string &raster_file, float x_base, float x_thre_dis, std::vector<bounds_t> &bounds)
    {
        //data format
        //ID y_bl z_bl y_tl z_tl y_tr z_tr y_br z_br
        //1 1.5609 0.5399 1.5609 1.5399 2.5609 1.5399 2.5609 0.5399

        std::ifstream in(raster_file.c_str(), std::ios::in);
        if (!in)
        {
            return 0;
        }
        int id;
        float y_min, y_max, z_min, z_max;
        float s1, s2, s3, s4;
        float x_min = x_base - x_thre_dis;
        float x_max = x_base + x_thre_dis;
        int i = 0;
        std::string head;
        //std::getline (std::cin,head);
        std::getline(in, head);

        while (!in.eof())
        {
            in >> id >> y_min >> z_min >> s1 >> s2 >> y_max >> z_max >> s3 >> s4;
            if (in.fail())
            {
                break;
            }
            bounds_t bound;
            bound.min_x = x_min;
            bound.max_x = x_max;
            bound.min_y = y_min;
            bound.max_y = y_max;
            bound.min_z = z_min;
            bound.max_z = z_max;

            std::cout << y_min << "\t" << z_min << "\t" << y_max << "\t" << z_max << "\n";

            bounds.push_back(bound);

            ++i;
        }
        in.close();

        std::cout << "Import finished ... ... " << i << " subdivisions." << std::endl;

        return 1;
    }

    bool readPlaneCoeff(const std::string &plane_file, pcl::ModelCoefficients::Ptr &coefficients)
    {
        coefficients->values.resize(4);

        std::ifstream in_plane(plane_file, std::ios::in);
        if (!in_plane)
        {
            return 0;
        }
        in_plane >> coefficients->values[0] >> coefficients->values[1] >> coefficients->values[2] >> coefficients->values[3];

        in_plane.close();
        std::cout << "Import plane coefficients done ... ..." << std::endl;
        std::cout << "plane coefficients: " << std::endl
                  << "a:" << coefficients->values[0] << " ,b:" << coefficients->values[1]
                  << " ,c:" << coefficients->values[2] << " ,d:" << coefficients->values[3] << std::endl;

        return 1;
    }

    bool outputRasterDistStd(const std::string &std_file, std::vector<float> &raster_std_list)
    {
        std::ofstream ofs;
        ofs.open(std_file);
        if (ofs.is_open())
        {
            for (int i = 0; i < raster_std_list.size(); ++i)
            {
                ofs << i + 1 << "  "
                    << setiosflags(std::ios::fixed) << std::setprecision(6) << raster_std_list[i]
                    << std::endl;
            }
            ofs.close();
        }
        else
        {
            return 0;
        }
        std::cout << "Output rasters' std of the distance to reference plane done ... ..." << std::endl;
    }
};

#endif //_INCLUDE_DATA_IO_HPP