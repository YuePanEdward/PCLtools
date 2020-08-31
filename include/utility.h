#ifndef _INCLUDE_UTILITY_H
#define _INCLUDE_UTILITY_H

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/point_representation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

#include <vector>
#include <list>

#define max_(a, b) (((a) > (b)) ? (a) : (b))
#define min_(a, b) (((a) < (b)) ? (a) : (b))

using namespace std;

struct centerpoint_t
{
    double x;
    double y;
    double z;
    centerpoint_t(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z)
    {
        z = 0.0;
        x = y = 0.0;
    }
};

struct bounds_t
{
    double min_x;
    double min_y;
    double min_z;
    double max_x;
    double max_y;
    double max_z;
    bounds_t()
    {
        min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
    }
};

template <typename PointT>
class CloudUtility
{
  public:
    //Get Center of a Point Cloud
    void getCloudCenterpoint(const typename pcl::PointCloud<PointT> &cloud, centerpoint_t &cp)
    {
        double cx = 0, cy = 0, cz = 0;

        for (int i = 0; i < cloud.size(); i++)
        {
            cx += cloud.points[i].x / cloud.size();
            cy += cloud.points[i].y / cloud.size();
            cz += cloud.points[i].z / cloud.size();
        }
        cp.x = cx;
        cp.y = cy;
        cp.z = cz;
    }

    //Get Bound of a Point Cloud
    void getCloudBound(const typename pcl::PointCloud<PointT> &cloud, bounds_t &bound)
    {
        double min_x = cloud[0].x;
        double min_y = cloud[0].y;
        double min_z = cloud[0].z;
        double max_x = cloud[0].x;
        double max_y = cloud[0].y;
        double max_z = cloud[0].z;

        for (int i = 0; i < cloud.size(); i++)
        {
            if (min_x > cloud.points[i].x)
                min_x = cloud.points[i].x;
            if (min_y > cloud.points[i].y)
                min_y = cloud.points[i].y;
            if (min_z > cloud.points[i].z)
                min_z = cloud.points[i].z;
            if (max_x < cloud.points[i].x)
                max_x = cloud.points[i].x;
            if (max_y < cloud.points[i].y)
                max_y = cloud.points[i].y;
            if (max_z < cloud.points[i].z)
                max_z = cloud.points[i].z;
        }
        bound.min_x = min_x;
        bound.max_x = max_x;
        bound.min_y = min_y;
        bound.max_y = max_y;
        bound.min_z = min_z;
        bound.max_z = max_z;
    }

    //Get Bound and Center of a Point Cloud
    void getBoundAndCenter(const typename pcl::PointCloud<PointT> &cloud, bounds_t &bound, centerpoint_t &cp)
    {
        getCloudBound(cloud, bound);
        cp.x = 0.5 * (bound.min_x + bound.max_x);
        cp.y = 0.5 * (bound.min_y + bound.max_y);
        cp.z = 0.5 * (bound.min_z + bound.max_z);
    }

    bool bbxFilter(const typename pcl::PointCloud<PointT>::Ptr cloud_in,
                   typename pcl::PointCloud<PointT>::Ptr cloud_out, bounds_t &bbx)
    {
        for (int i = 0; i < cloud_in->points.size(); i++)
        {
            //In the bounding box
            if (cloud_in->points[i].x > bbx.min_x && cloud_in->points[i].x < bbx.max_x &&
                cloud_in->points[i].y > bbx.min_y && cloud_in->points[i].y < bbx.max_y &&
                cloud_in->points[i].z > bbx.min_z && cloud_in->points[i].z < bbx.max_z)
            {
                cloud_out->points.push_back(cloud_in->points[i]);
            }
        }
        std::cout << "Get " << cloud_out->points.size() << " points" << std::endl;
        return true;
    }

    bool fitPlane(const typename pcl::PointCloud<PointT>::Ptr cloud_in, pcl::ModelCoefficients::Ptr plane_coeff,
                  float dist_thre, int max_iter_ransac = 200)
    {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        typename pcl::SACSegmentation<PointT> seg; // Create the segmentation object

        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(max_iter_ransac);
        seg.setDistanceThreshold(dist_thre);

        seg.setInputCloud(cloud_in);
        seg.segment(*inliers, *plane_coeff);

        if (inliers->indices.size() != cloud_in->points.size())
        {
            std::cout << "[ERROR] [FITPLANE] The distance from a point to the best fit plane is larger than the threshold [" << dist_thre * 1000.0 << "] mm." << std::endl;
            return false;
        }
        else
        {
            std::cout << "[INFO] [FITPLANE] The plane is successfully fitted" << std::endl;
            std::cout << "Plane function: " << plane_coeff->values[0] << " x + " << plane_coeff->values[1]
                      << " y + " << plane_coeff->values[2] << " z + " << plane_coeff->values[3] << " = 0" << std::endl;
        }
        return true;
    }

    bool cutHull(const typename pcl::PointCloud<PointT>::Ptr cloud_in,
                 const typename pcl::PointCloud<PointT>::Ptr polygon_vertices,
                 typename pcl::PointCloud<PointT>::Ptr pointcloud_cut, bool keep_silent = false)
    {
        std::vector<pcl::Vertices> polygons;

        int vertex_count = polygon_vertices->size();
        pcl::Vertices polygon;
        polygon.vertices.resize(vertex_count); //set the vertex number of the bounding polygon
        for (int i = 0; i < vertex_count; i++) //set the indice of polygon vertices (assuming the input is already in order)
            polygon.vertices[i] = i;
        polygons.push_back(polygon); //just one polygon

        pcl::CropHull<PointT> bp_filter; //bounding polygon filter
        bp_filter.setDim(2);
        bp_filter.setInputCloud(cloud_in);        //the point cloud waiting for cut
        bp_filter.setHullCloud(polygon_vertices); //the point cloud storing the coordinates of the bounding polygon
        bp_filter.setHullIndices(polygons);       //the indice of the vertex of the bounding polygon corresponding to the point cloud
        bp_filter.filter(*pointcloud_cut);        //get the cutted point cloud

        if (!keep_silent)
        {
            std::cout << "[INFO] [CutHull] Point number before bounding polygon cutting: " << cloud_in->size() << std::endl;
            std::cout << "[INFO] [CutHull] Point number after bounding polygo cutting: " << pointcloud_cut->size() << std::endl;
        }
        return true;
    }

    bool projectCloud2Plane(typename pcl::PointCloud<PointT>::Ptr cloud_in, pcl::ModelCoefficients::Ptr coefficients,
                            typename pcl::PointCloud<PointT>::Ptr cloud_proj, std::vector<float> &dist_list)
    {
        // Create the projection object
        typename pcl::ProjectInliers<PointT> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(cloud_in);
        proj.setModelCoefficients(coefficients);
        proj.filter(*cloud_proj);

        dist_list.resize(cloud_in->points.size());
        for (int i = 0; i < cloud_in->points.size(); i++)
        {
            dist_list[i] = pcl::geometry::distance(cloud_in->points[i], cloud_proj->points[i]);
        }
        std::cout << "[INFO] [Proj] Points projected to the reference plane done" << std::endl;

        return true;
    }

    float getStdDist(std::vector<float> &dist_list)
    {
        float dist_sum = 0;
        float distc2_sum = 0;
        float dist_std = 0;
        for (int i = 0; i < dist_list.size(); i++)
        {
            dist_sum += dist_list[i];
        }
        float dist_mean = dist_sum / dist_list.size();

        for (int i = 0; i < dist_list.size(); i++)
        {
            distc2_sum += ((dist_list[i] - dist_mean) * (dist_list[i] - dist_mean));
        }

        dist_std = std::sqrt(distc2_sum / dist_list.size());

        return dist_std;
    }

    float getMeanIntensity(typename pcl::PointCloud<PointT>::Ptr cloud_in)
    {
        float sum_intensity = 0;

        for (int i = 0; i < cloud_in->points.size(); i++)
        {
            sum_intensity += cloud_in->points[i].intensity;
        }
        float mean_intensity = 1.0 * sum_intensity / cloud_in->points.size();
        return mean_intensity;
    }

    float getMeanDist(typename pcl::PointCloud<PointT>::Ptr cloud_g, Eigen::Matrix4f &tran_s2g)
    {
        float sum_dis = 0;
        Eigen::Matrix4f tran_g2s = tran_s2g.inverse();
        typename pcl::PointCloud<PointT>::Ptr cloud_s(new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*cloud_g, *cloud_s, tran_g2s);

        for (int i = 0; i < cloud_s->points.size(); i++)
        {
            Eigen::Vector3f pt(cloud_s->points[i].x, cloud_s->points[i].y, cloud_s->points[i].z);

            sum_dis += pt.norm();
        }

        float mean_dis = 1.0 * sum_dis / cloud_g->points.size();
        return mean_dis;
    }

    float getMeanIncidenceAngle(typename pcl::PointCloud<PointT>::Ptr cloud_g, pcl::ModelCoefficients::Ptr &coeff, Eigen::Matrix4f &tran_s2g)
    {
        float sum_ia = 0;
        Eigen::Matrix4f tran_g2s = tran_s2g.inverse();
        typename pcl::PointCloud<PointT>::Ptr cloud_s(new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*cloud_g, *cloud_s, tran_g2s);

        Eigen::Vector3f normal_g(coeff->values[0], coeff->values[1], coeff->values[2]);
        Eigen::Vector3f normal_s = tran_g2s.block<3, 3>(0, 0) * normal_g;

        for (int i = 0; i < cloud_s->points.size(); i++)
        {
            Eigen::Vector3f pt(cloud_s->points[i].x, cloud_s->points[i].y, cloud_s->points[i].z);

            float cos_ia = std::abs(pt.dot(normal_s)) / pt.norm() / normal_s.norm();

            sum_ia += (std::acos(cos_ia) / M_PI * 180.0);
        }

        float mean_ia = 1.0 * sum_ia / cloud_g->points.size();
        return mean_ia;
    }

    bool fitHull(typename pcl::PointCloud<PointT>::Ptr cloud_in,
                 typename pcl::PointCloud<PointT>::Ptr polygon_vertices,
                 std::vector<pcl::Vertices> &polygons,
                 bool convex_or_not = true, float alpha_shape_value = 0.1)
    {
        // backup
        // computing the convex (or concave hull) from the point cloud

        if (convex_or_not)
        {
            pcl::ConvexHull<PointT> convex_hull;
            convex_hull.setInputCloud(cloud_in);
            convex_hull.setDimension(2);
            convex_hull.reconstruct(*polygon_vertices, polygons);
        }
        else
        {
            pcl::ConcaveHull<PointT> concave_hull;
            concave_hull.setInputCloud(cloud_in);
            concave_hull.setDimension(2);
            concave_hull.setAlpha(alpha_shape_value);
            concave_hull.reconstruct(*polygon_vertices, polygons);
        }

        return true;
    }

    //SOR (Statisics Outliers Remover);
    bool SORFilter(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
                   typename pcl::PointCloud<PointT>::Ptr &cloud_out, int MeanK, double std)
    {
        // Create the filtering object
        pcl::StatisticalOutlierRemoval<PointT> sor;

        sor.setInputCloud(cloud_in);
        sor.setMeanK(MeanK);         //50
        sor.setStddevMulThresh(std); //2.0
        sor.filter(*cloud_out);

        return true;
    }

  protected:
  private:
};

#endif //_INCLUDE_UTILITY_H