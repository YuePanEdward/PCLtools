//
// This file is a General Definition and Tool for Point Cloud Processing based on PCL.
// Dependent 3rd Libs: PCL (>1.7)
// Author: Yue Pan
//

#ifndef _INCLUDE_UTILITY_HPP_
#define _INCLUDE_UTILITY_HPP_

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/point_representation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/transformation_estimation_svd.h>

//Eigen
#include <Eigen/Core>

#include <vector>
#include <list>
#include <chrono>
#include <limits>

#include <glog/logging.h>

#include "stereo_binary_feature.h"

#define max_(a, b) (((a) > (b)) ? (a) : (b))
#define min_(a, b) (((a) < (b)) ? (a) : (b))

using namespace std;

//TypeDef

//Select from these two (with/without intensity)
//typedef pcl::PointNormal Point_T;
typedef pcl::PointXYZINormal Point_T; //mind that 'curvature' here is used as ring number for spining scanner
//typedef ccn::PointXYZINTRL Point_T; //TODO : with time stamp, label and ring number property
//typedef pcl::PointSurfel Point_T;

typedef pcl::PointCloud<Point_T>::Ptr pcTPtr;
typedef pcl::PointCloud<Point_T> pcT;

typedef pcl::search::KdTree<Point_T>::Ptr pcTreePtr;
typedef pcl::search::KdTree<Point_T> pcTree;

typedef pcl::PointCloud<pcl::PointXYZI>::Ptr pcXYZIPtr;
typedef pcl::PointCloud<pcl::PointXYZI> pcXYZI;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pcXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZ> pcXYZ;

typedef pcl::PointCloud<pcl::PointXY>::Ptr pcXYPtr;
typedef pcl::PointCloud<pcl::PointXY> pcXY;

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcXYZRGBPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> pcXYZRGB;

typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcXYZRGBAPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBA> pcXYZRGBA;

typedef pcl::PointCloud<pcl::PointNormal>::Ptr pcXYZNPtr;
typedef pcl::PointCloud<pcl::PointNormal> pcXYZN;

typedef pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcXYZINPtr;
typedef pcl::PointCloud<pcl::PointXYZINormal> pcXYZIN;

typedef pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhPtr;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfh;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> Matrix4ds;

namespace lo
{

	struct centerpoint_t
	{
		double x;
		double y;
		double z;
		centerpoint_t(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
	};

	//regular bounding box whose edges are parallel to x,y,z axises
	struct bounds_t
	{
		double min_x;
		double min_y;
		double min_z;
		double max_x;
		double max_y;
		double max_z;
		int type;

		bounds_t()
		{
			min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
		}
		void inf_x()
		{
			min_x = -DBL_MAX;
			max_x = DBL_MAX;
		}
		void inf_y()
		{
			min_y = -DBL_MAX;
			max_y = DBL_MAX;
		}
		void inf_z()
		{
			min_z = -DBL_MAX;
			max_z = DBL_MAX;
		}
		void inf_xyz()
		{
			inf_x();
			inf_y();
			inf_z();
		}
	};

	enum DataType
	{
		ALS,
		TLS,
		MLS,
		BPLS,
		RGBD
	};

	enum ConstraintType
	{
		REGISTRATION,
		ADJACENT,
		NONE
	};

	struct pose_qua_t //copyright: Jingwei Li
	{
		Eigen::Vector3d trans;
		Eigen::Quaterniond quat;

		pose_qua_t()
		{
			trans << 0, 0, 0;
			quat = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
			quat.normalize();
		}
		pose_qua_t(const pose_qua_t &pose)
		{
			this->copyFrom(pose);
		}
		pose_qua_t(Eigen::Quaterniond quat,
				   Eigen::Vector3d trans) : trans(trans), quat(quat)
		{
			quat.normalize();
		}
		pose_qua_t operator*(const pose_qua_t &pose) const
		{
			return pose_qua_t(this->quat.normalized() * pose.quat.normalized(), this->quat.normalized() * pose.trans + this->trans);
		}
		bool operator==(const pose_qua_t &pose) const
		{
			if (this->quat.x() == pose.quat.x() &&
				this->quat.y() == pose.quat.y() &&
				this->quat.z() == pose.quat.z() &&
				this->quat.w() == pose.quat.w() &&
				this->trans == pose.trans)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		Eigen::Matrix4d GetMatrix() const
		{
			//CHECK(quat.norm() == 1) << "NO EQUAL";
			Eigen::Matrix4d transformation_matrix;
			transformation_matrix.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix(); //You need to gurantee the quat is normalized
			transformation_matrix.block<3, 1>(0, 3) = trans;
			transformation_matrix.block<1, 4>(3, 0) << 0, 0, 0, 1;
			return transformation_matrix;
		}
		void SetPose(Eigen::Matrix4d transformation)
		{
			quat = Eigen::Quaterniond(transformation.block<3, 3>(0, 0)).normalized();
			trans << transformation(0, 3), transformation(1, 3), transformation(2, 3);
		}
		void copyFrom(const pose_qua_t &pose)
		{
			trans = pose.trans;
			//  trans << pose.trans[0], pose.trans[1], pose.trans[2];
			quat = Eigen::Quaterniond(pose.quat);
		}
		// inverse and return
		pose_qua_t inverse()
		{
			Eigen::Matrix4d transformation_matrix = GetMatrix();
			SetPose(transformation_matrix.inverse());
			return *this;
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	//Basic processing unit(node) for Cloud Control Net
	struct cloudblock_t
	{
		int unique_id;		//Unique ID  //(for submap, unique_id is the frame index of the last frame of the submap)
		int strip_id;		//Strip ID
		int id_in_strip;	//ID in the strip
		DataType data_type; //Datatype

		bounds_t bound;				  //Bounding Box in geo-coordinate system
		centerpoint_t cp;			  //Center Point in geo-coordinate system
		centerpoint_t station;		  //Station position in geo-coordinate system
		Eigen::Matrix4d station_pose; //Station pose in geo-coordinate system

		bounds_t local_bound;				//Bounding Box in local coordinate system
		centerpoint_t local_cp;				//Center Point in local coordinate system
		centerpoint_t local_station;		//Station position in local coordinate system
		Eigen::Matrix4d local_station_pose; //Station pose in local coordinate system

		bool station_position_available; //If the approximate position of the station is provided
		bool station_pose_available;	 //If the approximate pose of the station is provided
		bool is_single_scanline;		 //If the scanner is a single scanline sensor, determining if adjacent cloud blocks in a strip would have overlap

		bool pose_fixed = false;  //the pose is fixed or not
		bool pose_stable = false; //the pose is stable or not after the optimization

		//poses
		Eigen::Matrix4d pose_lo;		//used for lidar odometry
		Eigen::Matrix4d pose_gt;		//used for lidar odometry (ground turth)
		Eigen::Matrix4d pose_optimized; //optimized pose
		Eigen::Matrix4d pose_init;		//used for the init guess for pgo

		Matrix6d information_matrix_to_next;

		std::string filename;			//full path of the original point cloud file
		std::string filenmae_processed; //full path of the processed point cloud file

		//Raw point cloud
		pcTPtr pc_raw;

		//Downsampled point cloud
		pcTPtr pc_down;
		pcTPtr pc_sketch; //very sparse point cloud

		pcTPtr pc_raw_w; //in world coordinate system (for lidar odometry)

		//unground point cloud
		pcTPtr pc_unground;

		// All kinds of geometric feature points
		pcTPtr pc_ground;
		pcTPtr pc_facade;
		pcTPtr pc_roof;
		pcTPtr pc_pillar;
		pcTPtr pc_beam;
		pcTPtr pc_vertex;

		pcTPtr pc_ground_down;
		pcTPtr pc_facade_down;
		pcTPtr pc_roof_down;
		pcTPtr pc_pillar_down;
		pcTPtr pc_beam_down;

		//keypoints's feature
		doubleVectorSBF keypoint_bsc;

		//Kdtree of the feature points (denser ones)
		pcTreePtr tree_ground;
		pcTreePtr tree_pillar;
		pcTreePtr tree_beam;
		pcTreePtr tree_facade;
		pcTreePtr tree_roof;
		pcTreePtr tree_vertex;

		//actually, it's better to save the indices of feature_points_down instead of saving another feature point cloud

		int down_feature_point_num;
		int feature_point_num;

		//for blunder registration removal (copyright: Xing Zhao)
		// int reg_source_con_number = 0; //record the count of taking this cloud block as the source cloud
		// //int reg_target_con_number = 0; //record the count of taking this cloud block as the target cloud
		// //int reg_con_number = 0;        //record the count of the registration edges connecting to this cloud block
		// //int adj_con_number = 0;        //record the count of the adjacent edges connecting to this cloud block
		// int out_reg_count = 0; //record the final count of registration for the block
		// //std::vector<int> target_unique_id; //record the unique ID of the connected point clouds(as target point cloud)
		// //std::vector<int> source_unique_id; //record the unique ID of the connected point clouds(as source point cloud)
		// std::vector<int> as_source_con_unique_id; //record the unique ID of the edges when taking this cloud block as source cloud block
		// //std::vector<int> as_target_con_unique_id; //record the unique ID of the edges when taking this cloud block as target cloud block
		// bool cal_regsource_con_number = false; //record if the reg_source_con_number has been calulated for this cloud block
		// bool elimination_flag = false;		   //record if the registration blunder removal has been done

		cloudblock_t()
		{
			init();
			//default value
			station_position_available = false;
			station_pose_available = false;
			is_single_scanline = true;
			pose_lo.setIdentity();
			pose_gt.setIdentity();
			pose_optimized.setIdentity();
			pose_init.setIdentity();
			information_matrix_to_next.setIdentity();
		}

		cloudblock_t(const cloudblock_t &in_block, bool clone_feature = false, bool clone_raw = false)
		{
			init();
			clone_metadata(in_block);

			if (clone_feature)
			{
				//clone point cloud (instead of pointer)
				*pc_ground = *(in_block.pc_ground);
				*pc_pillar = *(in_block.pc_pillar);
				*pc_facade = *(in_block.pc_facade);
				*pc_beam = *(in_block.pc_beam);
				*pc_roof = *(in_block.pc_roof);
				*pc_vertex = *(in_block.pc_vertex);
				// keypoint_bsc = in_block.keypoint_bsc;
			}
			if (clone_raw)
				*pc_raw = *(in_block.pc_raw);
		}

		void init()
		{
			pc_raw = boost::make_shared<pcT>();
			pc_down = boost::make_shared<pcT>();
			pc_raw_w = boost::make_shared<pcT>();
			pc_sketch = boost::make_shared<pcT>();
			pc_unground = boost::make_shared<pcT>();

			pc_ground = boost::make_shared<pcT>();
			pc_facade = boost::make_shared<pcT>();
			pc_roof = boost::make_shared<pcT>();
			pc_pillar = boost::make_shared<pcT>();
			pc_beam = boost::make_shared<pcT>();
			pc_vertex = boost::make_shared<pcT>();

			pc_ground_down = boost::make_shared<pcT>();
			pc_facade_down = boost::make_shared<pcT>();
			pc_roof_down = boost::make_shared<pcT>();
			pc_pillar_down = boost::make_shared<pcT>();
			pc_beam_down = boost::make_shared<pcT>();

			init_tree();

			doubleVectorSBF().swap(keypoint_bsc);

			down_feature_point_num = 0;
			feature_point_num = 0;
		}

		void init_tree()
		{
			tree_ground = boost::make_shared<pcTree>();
			tree_facade = boost::make_shared<pcTree>();
			tree_pillar = boost::make_shared<pcTree>();
			tree_beam = boost::make_shared<pcTree>();
			tree_roof = boost::make_shared<pcTree>();
			tree_vertex = boost::make_shared<pcTree>();
		}

		void free_raw_cloud()
		{
			pc_raw.reset(new pcT());
			pc_down.reset(new pcT());
			pc_unground.reset(new pcT());
		}

		void free_tree()
		{
			tree_ground.reset(new pcTree());
			tree_facade.reset(new pcTree());
			tree_pillar.reset(new pcTree());
			tree_beam.reset(new pcTree());
			tree_roof.reset(new pcTree());
			tree_vertex.reset(new pcTree());
		}

		void free_all()
		{
			free_raw_cloud();
			free_tree();
			pc_ground.reset(new pcT());
			pc_facade.reset(new pcT());
			pc_pillar.reset(new pcT());
			pc_beam.reset(new pcT());
			pc_roof.reset(new pcT());
			pc_vertex.reset(new pcT());
			pc_ground_down.reset(new pcT());
			pc_facade_down.reset(new pcT());
			pc_pillar_down.reset(new pcT());
			pc_beam_down.reset(new pcT());
			pc_roof_down.reset(new pcT());
			pc_sketch.reset(new pcT());
			pc_raw_w.reset(new pcT());
			doubleVectorSBF().swap(keypoint_bsc);
		}

		void clone_metadata(const cloudblock_t &in_cblock)
		{
			feature_point_num = in_cblock.feature_point_num;
			bound = in_cblock.bound;
			local_bound = in_cblock.local_bound;
			local_cp = in_cblock.local_cp;
			pose_lo = in_cblock.pose_lo;
			pose_init = in_cblock.pose_init;
			pose_optimized = in_cblock.pose_optimized;
			unique_id = in_cblock.unique_id;
			id_in_strip = in_cblock.id_in_strip;
			filename = in_cblock.filename;
		}

		void append_feature(const cloudblock_t &in_cblock, bool append_down)
		{
			//pc_raw->points.insert(pc_raw->points.end(), in_cblock.pc_raw->points.begin(), in_cblock.pc_raw->points.end());
			if (!append_down)
			{
				pc_ground->points.insert(pc_ground->points.end(), in_cblock.pc_ground->points.begin(), in_cblock.pc_ground->points.end());
				pc_facade->points.insert(pc_facade->points.end(), in_cblock.pc_facade->points.begin(), in_cblock.pc_facade->points.end());
				pc_pillar->points.insert(pc_pillar->points.end(), in_cblock.pc_pillar->points.begin(), in_cblock.pc_pillar->points.end());
				pc_beam->points.insert(pc_beam->points.end(), in_cblock.pc_beam->points.begin(), in_cblock.pc_beam->points.end());
				pc_roof->points.insert(pc_roof->points.end(), in_cblock.pc_roof->points.begin(), in_cblock.pc_roof->points.end());
				pc_vertex->points.insert(pc_vertex->points.end(), in_cblock.pc_vertex->points.begin(), in_cblock.pc_vertex->points.end());
			}
			else
			{
				pc_ground->points.insert(pc_ground->points.end(), in_cblock.pc_ground_down->points.begin(), in_cblock.pc_ground_down->points.end());
				pc_facade->points.insert(pc_facade->points.end(), in_cblock.pc_facade_down->points.begin(), in_cblock.pc_facade_down->points.end());
				pc_pillar->points.insert(pc_pillar->points.end(), in_cblock.pc_pillar_down->points.begin(), in_cblock.pc_pillar_down->points.end());
				pc_beam->points.insert(pc_beam->points.end(), in_cblock.pc_beam_down->points.begin(), in_cblock.pc_beam_down->points.end());
				pc_roof->points.insert(pc_roof->points.end(), in_cblock.pc_roof_down->points.begin(), in_cblock.pc_roof_down->points.end());
				pc_vertex->points.insert(pc_vertex->points.end(), in_cblock.pc_vertex->points.begin(), in_cblock.pc_vertex->points.end());
			}
		}

		void merge_feature_points(pcTPtr &pc_out, bool merge_down, bool with_out_ground = false)
		{
			if (!merge_down)
			{
				if (!with_out_ground)
					pc_out->points.insert(pc_out->points.end(), pc_ground->points.begin(), pc_ground->points.end());
				pc_out->points.insert(pc_out->points.end(), pc_facade->points.begin(), pc_facade->points.end());
				pc_out->points.insert(pc_out->points.end(), pc_pillar->points.begin(), pc_pillar->points.end());
				pc_out->points.insert(pc_out->points.end(), pc_beam->points.begin(), pc_beam->points.end());
				pc_out->points.insert(pc_out->points.end(), pc_roof->points.begin(), pc_roof->points.end());
				pc_out->points.insert(pc_out->points.end(), pc_vertex->points.begin(), pc_vertex->points.end());
			}
			else
			{
				if (!with_out_ground)
					pc_out->points.insert(pc_out->points.end(), pc_ground_down->points.begin(), pc_ground_down->points.end());
				pc_out->points.insert(pc_out->points.end(), pc_facade_down->points.begin(), pc_facade_down->points.end());
				pc_out->points.insert(pc_out->points.end(), pc_pillar_down->points.begin(), pc_pillar_down->points.end());
				pc_out->points.insert(pc_out->points.end(), pc_beam_down->points.begin(), pc_beam_down->points.end());
				pc_out->points.insert(pc_out->points.end(), pc_roof_down->points.begin(), pc_roof_down->points.end());
				pc_out->points.insert(pc_out->points.end(), pc_vertex->points.begin(), pc_vertex->points.end());
			}
		}

		void transform_feature(const Eigen::Matrix4d &trans_mat, bool transform_down)
		{
			pcl::transformPointCloudWithNormals(*pc_ground, *pc_ground, trans_mat);
			pcl::transformPointCloudWithNormals(*pc_pillar, *pc_pillar, trans_mat);
			pcl::transformPointCloudWithNormals(*pc_beam, *pc_beam, trans_mat);
			pcl::transformPointCloudWithNormals(*pc_facade, *pc_facade, trans_mat);
			pcl::transformPointCloudWithNormals(*pc_roof, *pc_roof, trans_mat);
			pcl::transformPointCloudWithNormals(*pc_vertex, *pc_vertex, trans_mat);
			if (transform_down)
			{
				pcl::transformPointCloudWithNormals(*pc_ground_down, *pc_ground_down, trans_mat);
				pcl::transformPointCloudWithNormals(*pc_pillar_down, *pc_pillar_down, trans_mat);
				pcl::transformPointCloudWithNormals(*pc_beam_down, *pc_beam_down, trans_mat);
				pcl::transformPointCloudWithNormals(*pc_facade_down, *pc_facade_down, trans_mat);
				pcl::transformPointCloudWithNormals(*pc_roof_down, *pc_roof_down, trans_mat);
			}
		}

		void clone_cloud(pcTPtr &pc_out, bool get_pc_done)
		{
			if (get_pc_done)
				pc_out->points.insert(pc_out->points.end(), pc_down->points.begin(), pc_down->points.end());
			else
				pc_out->points.insert(pc_out->points.end(), pc_raw->points.begin(), pc_raw->points.end());
		}

		void clone_feature(pcTPtr &pc_ground_out,
						   pcTPtr &pc_pillar_out,
						   pcTPtr &pc_beam_out,
						   pcTPtr &pc_facade_out,
						   pcTPtr &pc_roof_out,
						   pcTPtr &pc_vertex_out, bool get_feature_down)

		{
			if (get_feature_down)
			{
				*pc_ground_out = *pc_ground_down;
				*pc_pillar_out = *pc_pillar_down;
				*pc_beam_out = *pc_beam_down;
				*pc_facade_out = *pc_facade_down;
				*pc_roof_out = *pc_roof_down;
				*pc_vertex_out = *pc_vertex;
			}
			else
			{
				*pc_ground_out = *pc_ground;
				*pc_pillar_out = *pc_pillar;
				*pc_beam_out = *pc_beam;
				*pc_facade_out = *pc_facade;
				*pc_roof_out = *pc_roof;
				*pc_vertex_out = *pc_vertex;
			}
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	typedef std::vector<cloudblock_t, Eigen::aligned_allocator<cloudblock_t>> strip;
	typedef std::vector<strip> strips;
	typedef boost::shared_ptr<cloudblock_t> cloudblock_Ptr;
	typedef std::vector<cloudblock_Ptr> cloudblock_Ptrs;

	//the edge of pose(factor) graph
	struct constraint_t
	{
		int unique_id;				   //Unique ID
		cloudblock_Ptr block1, block2; //Two block  //Target: block1,  Source: block2
		ConstraintType con_type;	   //ConstraintType
		Eigen::Matrix4d Trans1_2;	   //transformation from 2 to 1 (in global shifted map coordinate system)
		Matrix6d information_matrix;
		float overlapping_ratio; //overlapping ratio (not bbx IOU) of two cloud blocks
		float confidence;
		float sigma;            //standard deviation of the edge

		constraint_t()
		{
			block1 = cloudblock_Ptr(new cloudblock_t);
			block2 = cloudblock_Ptr(new cloudblock_t);
			Trans1_2.setIdentity();
			information_matrix.setIdentity();
			sigma = FLT_MAX;
		}

		void free_cloud()
		{
			block1->free_all();
			block2->free_all();
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	typedef std::vector<constraint_t, Eigen::aligned_allocator<constraint_t>> constraints;

	//2d ring map(image) for the point cloud collected by mechanical(spining) multiple-scanline LiDAR
	struct ring_map_t
	{
		int width;
		int height;

		float f_up;
		float f_down;

		std::vector<std::vector<unsigned int>> ring_array;

		void init_hdl64() //check it later
		{
			width = 1800;
			height = 64;
			f_up = 15;
			f_down = 15;
			init_ring_map();
		}

		void init_pandar64()
		{
			width = 1800;
			height = 64;
			f_up = 15;
			f_down = 25;
			init_ring_map();
		}

		void init_ring_map()
		{
			ring_array.resize(height);
			for (int i = 0; i < height; i++)
				ring_array[i].resize(width);
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	//parameter lists
	struct constraint_param_t
	{
		std::string constraint_output_file;
		std::string registration_output_file;

		int find_constraint_knn;
		double find_constraint_overlap_ratio;
		int cloud_block_capacity;

		double vf_downsample_resolution_als;
		double vf_downsample_resolution_tls;
		double vf_downsample_resolution_mls;
		double vf_downsample_resolution_bpls;

		double gf_grid_resolution;
		double gf_max_grid_height_diff;
		double gf_neighbor_height_diff;
		int ground_downsample_rate;
		int nonground_downsample_rate;

		bool normal_downsampling_on;
		int normal_down_ratio;

		double pca_neigh_r;
		int pca_neigh_k;
		double pca_linearity_thre;
		double pca_planarity_thre;
		double pca_stablity_thre;

		double reg_corr_dis_thre;
		int reg_max_iteration_num;
		double converge_tran;
		double converge_rot_d;
	};

	struct pgo_param_t
	{
		std::string block_tran_file = "";

		std::string trust_region_strategy = "levenberg_marquardt";
		std::string linear_solver = "dense_schur";
		std::string sparse_linear_algebra_library = "suite_sparse";
		std::string dense_linear_algebra_library = "eigen";
		std::string robust_kernel_strategy = "huber";

		std::string ordering;  //marginalization
		bool robustify = true; //robust kernel function
		bool use_equal_weight = false;
		float robust_delta = 1.0;

		bool verbose = true; //show the detailed logs or not

		int num_threads = omp_get_max_threads(); //default = max
		//int num_threads = 4;
		int num_iterations = 50;

		//error estimation of each cloud block (for assigning the adjacent edge's information matrix [vcm^-1])
		float tx_std = 0.01;
		float ty_std = 0.01;
		float tz_std = 0.01;
		float roll_std = 0.05;
		float pitch_std = 0.05;
		float yaw_std = 0.05;
	};

	//basic common functions of point cloud
	template <typename PointT>
	class CloudUtility
	{
	public:
		//Get Center of a Point Cloud
		void get_cloud_cpt(const typename pcl::PointCloud<PointT>::Ptr &cloud, centerpoint_t &cp)
		{
			double cx = 0, cy = 0, cz = 0;
			int point_num = cloud->points.size();

			for (int i = 0; i < point_num; i++)
			{
				cx += cloud->points[i].x / point_num;
				cy += cloud->points[i].y / point_num;
				cz += cloud->points[i].z / point_num;
			}
			cp.x = cx;
			cp.y = cy;
			cp.z = cz;
		}

		//Get Bound of a Point Cloud
		void get_cloud_bbx(const typename pcl::PointCloud<PointT>::Ptr &cloud, bounds_t &bound)
		{
			double min_x = DBL_MAX;
			double min_y = DBL_MAX;
			double min_z = DBL_MAX;
			double max_x = -DBL_MAX;
			double max_y = -DBL_MAX;
			double max_z = -DBL_MAX;

			for (int i = 0; i < cloud->points.size(); i++)
			{
				if (min_x > cloud->points[i].x)
					min_x = cloud->points[i].x;
				if (min_y > cloud->points[i].y)
					min_y = cloud->points[i].y;
				if (min_z > cloud->points[i].z)
					min_z = cloud->points[i].z;
				if (max_x < cloud->points[i].x)
					max_x = cloud->points[i].x;
				if (max_y < cloud->points[i].y)
					max_y = cloud->points[i].y;
				if (max_z < cloud->points[i].z)
					max_z = cloud->points[i].z;
			}
			bound.min_x = min_x;
			bound.max_x = max_x;
			bound.min_y = min_y;
			bound.max_y = max_y;
			bound.min_z = min_z;
			bound.max_z = max_z;
		}

		//Get Bound and Center of a Point Cloud
		void get_cloud_bbx_cpt(const typename pcl::PointCloud<PointT>::Ptr &cloud, bounds_t &bound, centerpoint_t &cp)
		{
			get_cloud_bbx(cloud, bound);
			cp.x = 0.5 * (bound.min_x + bound.max_x);
			cp.y = 0.5 * (bound.min_y + bound.max_y);
			cp.z = 0.5 * (bound.min_z + bound.max_z);
		}

		void get_intersection_bbx(bounds_t &bbx_1, bounds_t &bbx_2, bounds_t &bbx_intersection, float bbx_boundary_pad = 2.0)
		{
			bbx_intersection.min_x = max_(bbx_1.min_x, bbx_2.min_x) - bbx_boundary_pad;
			bbx_intersection.min_y = max_(bbx_1.min_y, bbx_2.min_y) - bbx_boundary_pad;
			bbx_intersection.min_z = max_(bbx_1.min_z, bbx_2.min_z) - bbx_boundary_pad;
			bbx_intersection.max_x = min_(bbx_1.max_x, bbx_2.max_x) + bbx_boundary_pad;
			bbx_intersection.max_y = min_(bbx_1.max_y, bbx_2.max_y) + bbx_boundary_pad;
			bbx_intersection.max_z = min_(bbx_1.max_z, bbx_2.max_z) + bbx_boundary_pad;
		}

		void merge_bbx(std::vector<bounds_t> &bbxs, bounds_t &bbx_merged)
		{
			bbx_merged.min_x = DBL_MAX;
			bbx_merged.min_y = DBL_MAX;
			bbx_merged.min_z = DBL_MAX;
			bbx_merged.max_x = -DBL_MAX;
			bbx_merged.max_y = -DBL_MAX;
			bbx_merged.max_z = -DBL_MAX;

			for (int i = 0; i < bbxs.size(); i++)
			{
				bbx_merged.min_x = min_(bbx_merged.min_x, bbxs[i].min_x);
				bbx_merged.min_y = min_(bbx_merged.min_y, bbxs[i].min_y);
				bbx_merged.min_z = min_(bbx_merged.min_z, bbxs[i].min_z);
				bbx_merged.max_x = max_(bbx_merged.max_x, bbxs[i].max_x);
				bbx_merged.max_y = max_(bbx_merged.max_y, bbxs[i].max_y);
				bbx_merged.max_z = max_(bbx_merged.max_z, bbxs[i].max_z);
			}
		}

		//Get Bound of Subsets of a Point Cloud
		void get_sub_bbx(typename pcl::PointCloud<PointT>::Ptr &cloud, vector<int> &index, bounds_t &bound)
		{
			typename pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>);
			for (int i = 0; i < index.size(); i++)
			{
				temp_cloud->push_back(cloud->points[index[i]]);
			}
			get_cloud_bbx(temp_cloud, bound);
		}

		bool get_ring_map(const typename pcl::PointCloud<PointT>::Ptr &cloud_in, ring_map_t &ring_map) //check it later
		{
			for (int i = 0; i < cloud_in->points.size(); i++)
			{
				PointT pt = cloud_in->points[i];
				float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
				float hor_ang = std::atan2(pt.y, pt.x);
				float ver_ang = std::asin(pt.z / dist);

				float u = 0.5 * (1 - hor_ang / M_PI) * ring_map.width;
				float v = (1 - (ver_ang + ring_map.f_up) / (ring_map.f_up + ring_map.f_down)) * ring_map.height;

				ring_map.ring_array[(int)v][(int)u] = i; //save the indice of the point
			}
		}

	protected:
	private:
	};
} // namespace lo

#endif //_INCLUDE_UTILITY_HPP_