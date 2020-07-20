//
// This file is for the general implements of Multi-Metrics Linear Least Square Lidar Odometry And Mapping (MMLLS-LOAM)
// Dependent 3rd Libs: PCL (>1.7), glog, gflags
// Author: Yue Pan @ ETH Zurich
//

#include "dataio.hpp"
#include "cfilter.hpp"
#include "cregistration.hpp"
#include "map_manager.h"
#include "map_viewer.h"
#include "utility.hpp"
#include "binary_feature_extraction.hpp"
#include "odom_error_compute.h"
#include "common_nav.h"
#include "build_pose_graph.h"
#include "graph_optimizer.h"

#include <glog/logging.h>
#include <gflags/gflags.h>

//For visualizer's size (fit pc monitor)
//#define screen_width 2560
//#define screen_height 1440
#define screen_width 1920
#define screen_height 1080

using namespace std;
using namespace lo;

//static void CheckCudaErrorAux(const char *, unsigned, const char *, cudaError_t);
//#define CUDA_CHECK(value) CheckCudaErrorAux(__FILE__, __LINE__, #value, value)

//Parameter Lists:
//-------------------------------------------------------------------------------------------
//GFLAG Template: DEFINE_TYPE(Flag_variable_name, default_value, "Comments")
//data path
DEFINE_string(point_cloud_folder, "", "folder containing the point cloud of each frame");
DEFINE_string(pc_format, ".pcd", "input point cloud format (select from .pcd, .ply, .txt, .las, .h5 ...");
DEFINE_string(output_adjacent_lo_pose_file_path, "", "the file for saving the adjacent transformation of lidar odometry");
DEFINE_string(gt_body_pose_file_path, "", "optional: the file containing the ground truth pose in body coor. sys. (as the format of transformation matrix)");
DEFINE_string(calib_file_path, "", "optional: the file containing the calibration matrix (as the format of transformation matrix)");
DEFINE_string(output_gt_lidar_pose_file_path, "", "optional: the file for saving the groundtruth pose in the lidar coor. sys.");
DEFINE_string(output_lo_body_pose_file_path, "", "optional: the file for saving the pose estimated by the lidar odometry in the body coor. sys.");
DEFINE_string(output_lo_lidar_pose_file_path, "", "optional: the file for saving the pose estimated by the lidar odometry in the lidar coor. sys.");
DEFINE_string(output_map_point_cloud_folder_path, "", "optional: the folder for saving the transformed point clouds in world coordinate system");
DEFINE_string(lo_lidar_pose_point_cloud, "", "optional: save the lidar odometry trajectory as point cloud");
DEFINE_string(gt_lidar_pose_point_cloud, "", "optional: save the ground truth trajectory as point cloud");
//used frame
DEFINE_int32(frame_num_begin, 0, "begin from this frame (file sequence in the folder)");
DEFINE_int32(frame_num_end, 99999, "end at this frame (file sequence in the folder)");
DEFINE_int32(frame_step, 1, "use one in ${frame_step} frames");
//map related
DEFINE_bool(write_out_map_on, false, "output map point clouds or not");
DEFINE_bool(write_out_gt_map_on, false, "output map point clouds generated from the gnssins pose or not");
DEFINE_bool(write_map_each_frame, false, "output each frame's point cloud in map coordinate system");
DEFINE_int32(map_downrate_output, 5, "downsampling rate for output map point cloud");
DEFINE_bool(map_filter_on, false, "clean the map point clpud before output");
DEFINE_bool(apply_dist_filter, false, "Use only the points inside a distance range or not");
DEFINE_double(min_dist_used, 1.0, "only the points whose distance to the scanner is larger than this value would be used for scan matching (m)");
DEFINE_double(max_dist_used, 150.0, "only the points whose distance to the scanner is smaller than this value would be used for scan matching (m)");
DEFINE_double(min_dist_mapping, 2.0, "only the points whose distance to the scanner is larger than this value would be used for map merging (m)");
DEFINE_double(max_dist_mapping, 50.0, "only the points whose distance to the scanner is smaller than this value would be used for map merging (m)");
//viusalization related
DEFINE_bool(real_time_viewer_on, false, "launch real time viewer or not");
DEFINE_double(vis_intensity_scale, 256.0, "max intensity value of your data");
DEFINE_int32(vis_map_history_down_rate, 300, "downsampling rate of the map point cloud kept in the memory");
DEFINE_int32(vis_map_history_keep_frame_num, 150, "frame number of the dense map that would be kept in the memory");
DEFINE_int32(vis_initial_color_type, 0, "map viewer's rendering color style: (0: single color & semantic mask, 1: frame-wise, 2: height, 3: intensity)");
//lidar odometry related
DEFINE_bool(scan_to_scan_module_on, true, "apply scan-to-scan registration or just scan-to-localmap matching");
DEFINE_int32(motion_compensation_method, 0, "method for motion compensation of lidar (0: disabled, 1: uniform motion model (from point-wise timestamp), 2: from azimuth, 3: from azimuth (rotation-only), 4: imu-assisted)");
DEFINE_bool(vertical_ang_calib_on, false, "apply vertical intrinsic angle correction for sepecific lidar");
DEFINE_double(vertical_ang_correction_deg, 0.0, "the intrinsic correction of the vertical angle of lidar");
DEFINE_bool(zupt_on_or_not, false, "enable zupt (zero velocity updating) or not");
DEFINE_bool(apply_scanner_filter, false, "enable scanner based distance filtering or not");
DEFINE_bool(semantic_assist_on, true, "apply semantic mask to assist the geometric feature points extraction");
DEFINE_double(cloud_down_res, 0.04, "voxel size(m) of downsample for target point cloud");
DEFINE_int32(dist_inverse_sampling_method, 2, "use distance inverse sampling or not (0: disabled, 1: linear weight, 2: quadratic weight)");
DEFINE_double(unit_dist, 15, "distance that correspoinding to unit weight in inverse distance downsampling");
DEFINE_bool(adaptive_parameters_on, false, "use self-adaptive parameters for different surroundings and road situation");
DEFINE_double(cloud_pca_neigh_r, 0.5, "pca neighborhood searching radius(m) for target point cloud");
DEFINE_int32(cloud_pca_neigh_k, 20, "use only the k nearest neighbor in the r-neighborhood to do PCA");
DEFINE_int32(cloud_pca_neigh_k_min, 8, "the min number of points in the neighborhood for doing PCA");
DEFINE_bool(sharpen_with_nms_on, true, "using non-maximum supression to get the sharpen feature points from unsharpen points or not (use higher threshold)");
DEFINE_bool(fixed_num_downsampling_on, false, "enable/disable the fixed point number downsampling (processing time's standard deviation would br smaller)");
DEFINE_int32(ground_down_fixed_num, 500, "fixed number of the detected ground feature points (for source)");
DEFINE_int32(pillar_down_fixed_num, 150, "fixed number of the detected pillar feature points (for source)");
DEFINE_int32(facade_down_fixed_num, 350, "fixed number of the detected facade feature points (for source)");
DEFINE_int32(beam_down_fixed_num, 350, "fixed number of the detected beam feature points (for source)");
DEFINE_int32(roof_down_fixed_num, 0, "fixed number of the detected roof feature points (for source)");
DEFINE_int32(unground_down_fixed_num, 15000, "fixed number of the unground points used for PCA calculation");
DEFINE_double(gf_grid_size, 2.0, "grid size(m) of ground segmentation");
DEFINE_double(gf_in_grid_h_thre, 0.3, "height threshold(m) above the lowest point in a grid for ground segmentation");
DEFINE_double(gf_neigh_grid_h_thre, 2.2, "height threshold(m) among neighbor grids for ground segmentation");
DEFINE_double(gf_max_h, 20.0, "max height(m) allowed for ground point");
DEFINE_int32(ground_normal_method, 0, "method for estimating the ground points' normal vector ( 0: directly use (0,0,1), 1: estimate normal in fix radius neighborhood , 2: estimate normal in k nearest neighborhood, 3: use ransac to estimate plane coeffs in a grid)");
DEFINE_double(gf_normal_estimation_radius, 2.0, "neighborhood radius for local normal estimation of ground points (only enabled when ground_normal_method=1)");
DEFINE_int32(gf_ground_down_rate, 25, "downsampling decimation rate for target ground point cloud");
DEFINE_int32(gf_nonground_down_rate, 3, "downsampling decimation rate for non-ground point cloud");
DEFINE_double(intensity_thre_nonground, FLT_MAX, "Points whose intensity is larger than this value would be regarded as highly reflective object so that downsampling would not be applied.");
DEFINE_int32(gf_grid_min_pt_num, 10, "min number of points in a grid (if < this value, the grid would not be considered");
DEFINE_int32(gf_reliable_neighbor_grid_thre, 0, "min number of neighboring grid whose point number is larger than gf_grid_min_pt_num-1");
DEFINE_int32(gf_down_down_rate, 2, "downsampling rate based on the already downsampled ground point clouds used for source point cloud");
DEFINE_double(linearity_thre, 0.65, "pca linearity threshold for target point cloud");
DEFINE_double(planarity_thre, 0.65, "pca planarity threshold for target point cloud");
DEFINE_double(linearity_thre_down, 0.75, "pca linearity threshold for source point cloud");
DEFINE_double(planarity_thre_down, 0.75, "pca planarity threshold for source point cloud");
DEFINE_double(curvature_thre, 0.4, "pca local curvature threshold");
DEFINE_double(beam_direction_ang, 25, "the verticle angle threshold for the direction vector of beam-type feature points");
DEFINE_double(pillar_direction_ang, 60, "the verticle angle threshold for the direction vector of pillar-type feature points");
DEFINE_double(facade_normal_ang, 30, "the verticle angle threshold for the normal vector of facade-type feature points");
DEFINE_double(roof_normal_ang, 75, "the verticle angle threshold for the normal vector roof-type feature points");
DEFINE_double(beam_max_height, FLT_MAX, "max bearable height for beam points");
DEFINE_int32(vertex_extraction_method, 1, "extraction method of vertex points (0: disabled, 1: maximum local curvature within stable points, 2: intersection points of pillar and beams)");
DEFINE_bool(detect_curb_or_not, false, "detect curb feature for urban scenarios or not");
DEFINE_bool(apply_roi_filter, false, "use the region of interest filter to remove dynamic objects or not");
DEFINE_double(roi_min_y, -FLT_MAX, "region of interest (delete part): min_y");
DEFINE_double(roi_max_y, FLT_MAX, "region of interest (delete part): max_y");
DEFINE_string(used_feature_type, "111111", "used_feature_type (1: on, 0: off, order: ground, pillar, beam, facade, roof, vetrex)");
DEFINE_bool(reg_intersection_filter_on, true, "filter the points outside the intersection aera of two point cloud during registration");
DEFINE_double(corr_dis_thre_init, 1.0, "distance threshold between correspondence points at begining");
DEFINE_double(corr_dis_thre_min, 0.5, "minimum distance threshold between correspondence points at begining");
DEFINE_double(dis_thre_update_rate, 1.1, "update rate (divided by this value at each iteration) for distance threshold between correspondence points");
DEFINE_string(corr_weight_strategy, "1101", "weighting strategy for correspondences (1: on, 0: off, order: x,y,z balanced weight, residual weight, distance weight, intensity weight)");
DEFINE_double(z_xy_balance_ratio, 1.0, "the weight ratio of the error along z and x,y direction when balanced weight is enabled");
DEFINE_double(pt2pt_res_window, 0.1, "residual window size for the residual robust kernel function of point to point correspondence");
DEFINE_double(pt2pl_res_window, 0.1, "residual window size for the residual robust kernel function of point to plane correspondence");
DEFINE_double(pt2li_res_window, 0.1, "residual window size for the residual robust kernel function of point to line correspondence");
DEFINE_int32(reg_max_iter_num_s2s, 15, "max iteration number for icp-based registration (scan to scan)");
DEFINE_int32(reg_max_iter_num_s2m, 15, "max iteration number for icp-based registration (scan to map)");
DEFINE_int32(reg_max_iter_num_m2m, 20, "max iteration number for icp-based registration (map to map)");
DEFINE_double(converge_tran, 0.001, "convergence threshold for translation (in m)");
DEFINE_double(converge_rot_d, 0.01, "convergence threshold for rotation (in degree)");
DEFINE_double(local_map_radius, 100.0, "the radius of the local map (regarded as a sphere aera)");
DEFINE_int32(local_map_max_pt_num, 20000, "max point number allowed for the local map");
DEFINE_int32(local_map_max_vertex_pt_num, 1000, "max vertex point number allowed for the local map");
DEFINE_double(append_frame_radius, 30.0, "the radius of the frame that used to append into the local map");
DEFINE_bool(apply_map_based_dynamic_removal, false, "use map based dynamic object removal or not");
DEFINE_double(dynamic_removal_radius, 30.0, "the radius of the map based dynamic object removing");
DEFINE_double(dynamic_dist_thre_min, 0.3, "the distance threshold to judge if a point is dynamic or not");
DEFINE_int32(s2m_frequency, 8, "frequency of scan to map registration (Hz)");
DEFINE_int32(initial_guess_mode, 0, "Use which kind of initial guess(0: no initial guess, 1: uniform motion(translation only), 2: uniform motion(translation+rotation), 3:imu based)");
//prior knowledge
DEFINE_double(approx_scanner_height, 1.0, "approximate height of the scanner (m)");
DEFINE_double(underground_height_thre, -7.0, "z-axis threshold for rejecting underground ghost points (lines)");
//loop closure and pose graph optimization related
DEFINE_bool(loop_closure_detection_on, false, "do loop closure detection and pose graph optimization or not");
DEFINE_double(submap_accu_tran, 40.0, "accumulated translation (m) for generating a new submap");
DEFINE_double(submap_accu_rot, 120.0, "accumulated rotation (deg) for generating a new submap");
DEFINE_int32(submap_accu_frame, 150, "accumulated frame number for generating a new submap");
DEFINE_double(min_iou_thre, 0.5, "min boundingbox iou for candidate registration edge");
DEFINE_double(neighbor_search_dist, 50.0, "max distance for candidate registration edge");
DEFINE_int32(cooling_submap_num, 3, "waiting for several submaps (without loop closure detection) after applying a successful pgo");
DEFINE_int32(feature_corr_num, 1000, "number of the correspondence for global coarse registration");
DEFINE_bool(teaser_based_global_registration_on, true, "Using TEASER++ to do the global coarse registration or not");
DEFINE_string(pose_graph_optimization_method, "ceres", "use which library to do pgo (select from g2o, ceres and gtsam)");
//baseline method options
DEFINE_string(baseline_reg_method, "", "name of the baseline lidar odometery method (ndt, gicp, pclicp, etc.)");
DEFINE_double(reg_voxel_size, 1.0, "the grid size of ndt or vgicp");
DEFINE_bool(ndt_searching_method, true, "using direct searching or kdtree (0: kdtree, 1: direct7)");
DEFINE_bool(voxel_gicp_on, true, "using voxel based gicp (faster)");
//-------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging("Mylog_testlo");
    LOG(INFO) << "Launch the program!";
    LOG(INFO) << "Logging is written to " << FLAGS_log_dir;

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); //Ban pcl warnings

    CHECK(FLAGS_point_cloud_folder != "") << "Need to specify the point cloud's folder.";
    CHECK(FLAGS_output_lo_lidar_pose_file_path != "") << "Need to specify the output lidar odometery pose (in lidar coor. sys.) file's path.";

    //Import configuration
    //Data path (now only the *.pcd format is available)
    std::string pc_folder = FLAGS_point_cloud_folder;
    std::string pc_format = FLAGS_pc_format;
    std::string gt_body_pose_file = FLAGS_gt_body_pose_file_path;
    std::string calib_file = FLAGS_calib_file_path;
    std::string output_adjacent_lo_pose_file = FLAGS_output_adjacent_lo_pose_file_path;
    std::string output_lo_body_pose_file = FLAGS_output_lo_body_pose_file_path;
    std::string output_lo_lidar_pose_file = FLAGS_output_lo_lidar_pose_file_path;
    std::string output_gt_lidar_pose_file = FLAGS_output_gt_lidar_pose_file_path;
    std::string output_pc_folder = FLAGS_output_map_point_cloud_folder_path;
    std::string gt_lidar_pose_point_cloud_file = FLAGS_gt_lidar_pose_point_cloud;
    std::string lo_lidar_pose_point_cloud_file = FLAGS_lo_lidar_pose_point_cloud;
    //used frame number
    int frame_num_begin = FLAGS_frame_num_begin;
    int frame_num_end = FLAGS_frame_num_end;
    int frame_step = FLAGS_frame_step;
    bool write_out_map = FLAGS_write_out_map_on;
    bool write_out_gt_map = FLAGS_write_out_gt_map_on;
    //visualization settings
    bool launch_real_time_viewer = FLAGS_real_time_viewer_on;
    float intensity_scale = FLAGS_vis_intensity_scale;
    int display_color_style = FLAGS_vis_initial_color_type;
    int downsamping_rate_map_vis = FLAGS_vis_map_history_down_rate;
    int dense_map_keep_frame_num = FLAGS_vis_map_history_keep_frame_num;
    int downsamping_rate_scan_vis = 5;
    int display_time_ms = 10;
    //parameters (mainly for experiment)
    bool scan_to_scan_module = FLAGS_scan_to_scan_module_on;
    bool vertical_ang_calib = FLAGS_vertical_ang_calib_on;
    float vertical_ang_correction_deg = FLAGS_vertical_ang_correction_deg;
    int motion_compensation_method = FLAGS_motion_compensation_method;
    bool apply_zupt = FLAGS_zupt_on_or_not;
    bool apply_scanner_filter = FLAGS_apply_scanner_filter;
    bool semantic_assisted = FLAGS_semantic_assist_on;
    float vf_downsample_resolution = FLAGS_cloud_down_res;
    int use_dist_inverse_sampling = FLAGS_dist_inverse_sampling_method;
    float unit_weight_dist2station = FLAGS_unit_dist;
    bool self_adapative_parameters_on = FLAGS_adaptive_parameters_on;
    float gf_grid_resolution = FLAGS_gf_grid_size;
    float gf_max_grid_height_diff = FLAGS_gf_in_grid_h_thre;
    float gf_neighbor_height_diff = FLAGS_gf_neigh_grid_h_thre;
    float gf_max_height = FLAGS_gf_max_h;
    int estimate_ground_normal_method = FLAGS_ground_normal_method;
    float ground_normal_estimation_radius = FLAGS_gf_normal_estimation_radius;
    int ground_down_rate = FLAGS_gf_ground_down_rate;
    int nonground_down_rate = FLAGS_gf_nonground_down_rate;
    float intensity_thre = FLAGS_intensity_thre_nonground;
    int gf_grid_min_pt_num = FLAGS_gf_grid_min_pt_num;
    int gf_reliable_neighbor_grid_thre = FLAGS_gf_reliable_neighbor_grid_thre;
    int gf_down_down_rate = FLAGS_gf_down_down_rate;
    float pca_neigh_r = FLAGS_cloud_pca_neigh_r;
    int pca_neigh_k = FLAGS_cloud_pca_neigh_k;
    int pca_neigh_k_min = FLAGS_cloud_pca_neigh_k_min;
    float feature_neighbor_radius = 2.0 * pca_neigh_r;
    bool fixed_num_downsampling = FLAGS_fixed_num_downsampling_on;
    int ground_down_fixed_num = FLAGS_ground_down_fixed_num;
    int pillar_down_fixed_num = FLAGS_pillar_down_fixed_num;
    int facade_down_fixed_num = FLAGS_facade_down_fixed_num;
    int beam_down_fixed_num = FLAGS_beam_down_fixed_num;
    int roof_down_fixed_num = FLAGS_roof_down_fixed_num;
    int unground_down_fixed_num = FLAGS_unground_down_fixed_num;
    float pca_linearity_thre = FLAGS_linearity_thre;
    float pca_planarity_thre = FLAGS_planarity_thre;
    float pca_linearity_thre_down = FLAGS_linearity_thre_down;
    float pca_planarity_thre_down = FLAGS_planarity_thre_down;
    float beam_direction_sin = std::sin(FLAGS_beam_direction_ang / 180.0 * M_PI);
    float pillar_direction_sin = std::sin(FLAGS_pillar_direction_ang / 180.0 * M_PI);
    float facade_normal_sin = std::sin(FLAGS_facade_normal_ang / 180.0 * M_PI);
    float roof_normal_sin = std::sin(FLAGS_roof_normal_ang / 180.0 * M_PI);
    float beam_max_height = FLAGS_beam_max_height;
    float pca_curvature_thre = FLAGS_curvature_thre;
    bool use_curb_as_vertex = FLAGS_detect_curb_or_not;
    bool apply_roi_filter = FLAGS_apply_roi_filter;
    float roi_min_y = FLAGS_roi_min_y;
    float roi_max_y = FLAGS_roi_max_y;
    int vertex_extraction_method = FLAGS_vertex_extraction_method;
    std::string used_feature_type = FLAGS_used_feature_type;
    bool apply_intersection_filter = FLAGS_reg_intersection_filter_on;
    float reg_corr_dis_thre_init = FLAGS_corr_dis_thre_init;
    float reg_corr_dis_thre_min = FLAGS_corr_dis_thre_min;
    float dis_thre_update_rate = FLAGS_dis_thre_update_rate;
    std::string weight_strategy = FLAGS_corr_weight_strategy;
    float z_xy_balance_ratio = FLAGS_z_xy_balance_ratio;
    float converge_tran = FLAGS_converge_tran;
    float converge_rot_d = FLAGS_converge_rot_d;
    float pt2pt_residual_window = FLAGS_pt2pt_res_window;
    float pt2pl_residual_window = FLAGS_pt2pl_res_window;
    float pt2li_residual_window = FLAGS_pt2li_res_window;
    int max_iteration_num_s2s = FLAGS_reg_max_iter_num_s2s;
    int max_iteration_num_s2m = FLAGS_reg_max_iter_num_s2m;
    int max_iteration_num_m2m = FLAGS_reg_max_iter_num_m2m;
    int initial_guess_mode = FLAGS_initial_guess_mode;
    float local_map_radius = FLAGS_local_map_radius;
    float append_frame_radius = FLAGS_append_frame_radius;
    int local_map_max_pt_num = FLAGS_local_map_max_pt_num;
    int vertex_keeping_num = FLAGS_local_map_max_vertex_pt_num;
    bool map_based_dynamic_removal_on = FLAGS_apply_map_based_dynamic_removal;
    float dynamic_removal_radius = FLAGS_dynamic_removal_radius;
    float dynamic_dist_thre_min = FLAGS_dynamic_dist_thre_min;
    int s2m_frequency = FLAGS_s2m_frequency;
    float underground_thre = FLAGS_underground_height_thre;
    float approx_scanner_height = FLAGS_approx_scanner_height;
    bool loop_closure_detection_on = FLAGS_loop_closure_detection_on;
    float min_iou_thre = FLAGS_min_iou_thre;
    float neighbor_search_dist = FLAGS_neighbor_search_dist;
    int cooling_submap_num = FLAGS_cooling_submap_num;
    float reg_voxel_size = FLAGS_reg_voxel_size;
    bool ndt_direct_searching_on = FLAGS_ndt_searching_method;
    bool voxel_gicp_on = FLAGS_voxel_gicp_on;

    DataIo<Point_T> dataio;
    MapViewer<Point_T> mviewer(0.0, 1, 0, 0, intensity_scale, display_color_style); //downsampling ratio
    CFilter<Point_T> cfilter;
    CRegistration<Point_T> creg;
    BSCEncoder<Point_T> bsc_0(feature_neighbor_radius, 5, true);
    BSCEncoder<Point_T> bsc(feature_neighbor_radius, 5);
    MapManager mmanager;
    Navigation nav;
    Constraint_Finder confinder;
    GlobalOptimize pgoptimizer;

    //Launch visualization module
    boost::shared_ptr<pcl::visualization::PCLVisualizer> feature_viewer;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> reg_viewer;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> map_viewer;
    if (launch_real_time_viewer)
    {
        feature_viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Feature Viewer"));
        reg_viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Registration Viewer"));
        map_viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Map Viewer"));
        mviewer.set_interactive_events(feature_viewer, screen_width / 2, screen_height / 2);
        mviewer.set_interactive_events(reg_viewer, screen_width / 2, screen_height / 2);
        mviewer.set_interactive_events(map_viewer, screen_width / 2, screen_height);
    }
    double time_count = 0.0;

    std::vector<std::string> filenames;
    dataio.batch_read_filenames_in_folder(pc_folder, "_filelist.txt", pc_format, filenames, frame_num_begin, frame_num_end, frame_step);
    Matrix4ds poses_gt_body_cs;  //in vehicle body (gnssins) coordinate system
    Matrix4ds poses_lo_body_cs;  //in vehicle body (gnssins) coordinate system
    Matrix4ds poses_gt_lidar_cs; //in lidar coordinate system
    Matrix4ds poses_lo_lidar_cs; //in lidar coordinate system
    Matrix4ds poses_lo_adjacent;
    Eigen::Matrix4d calib_mat;
    poses_gt_body_cs = dataio.load_poses_from_transform_matrix(gt_body_pose_file, frame_num_begin, frame_step);
    dataio.load_calib_mat(calib_file, calib_mat);

    int frame_num = filenames.size();

    cloudblock_Ptr cblock_target(new cloudblock_t());
    cloudblock_Ptr cblock_source(new cloudblock_t());
    cloudblock_Ptr cblock_history(new cloudblock_t());
    cloudblock_Ptr cblock_local_map(new cloudblock_t());
    cloudblock_Ptrs cblock_submaps;
    cloudblock_Ptrs cblock_frames;

    constraint_t scan2scan_reg_con;
    constraint_t scan2map_reg_con;
    constraint_t scan2ground_reg_con;
    constraint_t scan2history_reg_con;
    constraints pgo_edges;

    Eigen::Matrix4d initial_guess_tran = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d adjacent_pose_out = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d first_frame_body = Eigen::Matrix4d::Identity();

    LOG(WARNING) << "[" << omp_get_max_threads() << "] threads availiable in total";

    bool seg_new_submap = false;
    int submap_count = 0;
    int cooling_index = 0;
    float accu_tran = 0.0;
    float accu_rot_deg = 0.0;
    int accu_frame = 0;
    float current_linear_velocity = 0.0;
    float current_angular_velocity = 0.0;
    bool local_map_recalculate_feature_on = false;
    int local_map_recalculation_frequency = 10;
    bool apply_motion_compensation_while_registration = false;
    float add_length = 0.0;
    int initial_scan2scan_frame_num = 2;
    bool lo_status_healthy = true;
    float reliable_sigma_thre = 0.01; //TODO: too small, fix later
    float non_max_suppresssion_radius = pca_neigh_r;

    if (motion_compensation_method > 0)
        apply_motion_compensation_while_registration = true;

    cblock_target->filename = filenames[0];
    cblock_target->unique_id = frame_num_begin;
    cblock_target->is_single_scanline = false; //multi-scanline lidar
    cblock_target->pose_lo.setIdentity();
    cblock_target->pose_gt.setIdentity();
    dataio.check_overwrite_exsiting_file_or_not(output_adjacent_lo_pose_file);
    dataio.write_lo_pose_overwrite(cblock_target->pose_lo, output_lo_lidar_pose_file);
    dataio.write_lo_pose_overwrite(cblock_target->pose_gt, output_gt_lidar_pose_file);
    dataio.write_lo_pose_overwrite(first_frame_body, output_lo_body_pose_file);
    poses_lo_lidar_cs.push_back(cblock_target->pose_lo);
    poses_gt_lidar_cs.push_back(cblock_target->pose_gt);
    poses_lo_body_cs.push_back(first_frame_body);

    dataio.read_pc_cloud_block(cblock_target);

    if (poses_gt_body_cs.size() > 0)
        first_frame_body = poses_gt_body_cs[0];
    if (FLAGS_apply_dist_filter)
        cfilter.dist_filter(cblock_target->pc_raw, FLAGS_min_dist_used, FLAGS_max_dist_used);
    if (vertical_ang_calib) //intrinsic angle correction
        cfilter.vertical_intrinsic_calibration(cblock_target->pc_raw, vertical_ang_correction_deg);
    cfilter.extract_semantic_pts(cblock_target, vf_downsample_resolution, gf_grid_resolution, gf_max_grid_height_diff,
                                 gf_neighbor_height_diff, gf_max_height, ground_down_rate,
                                 nonground_down_rate, pca_neigh_r, pca_neigh_k,
                                 pca_linearity_thre, pca_planarity_thre, pca_curvature_thre,
                                 pca_linearity_thre_down, pca_planarity_thre_down,
                                 self_adapative_parameters_on, apply_scanner_filter,
                                 estimate_ground_normal_method, ground_normal_estimation_radius,
                                 use_dist_inverse_sampling, unit_weight_dist2station, use_curb_as_vertex,
                                 vertex_extraction_method, gf_grid_min_pt_num, gf_reliable_neighbor_grid_thre,
                                 gf_down_down_rate, pca_neigh_k_min, intensity_thre,
                                 pillar_direction_sin, beam_direction_sin, roof_normal_sin, facade_normal_sin,
                                 FLAGS_sharpen_with_nms_on, fixed_num_downsampling, ground_down_fixed_num, pillar_down_fixed_num,
                                 facade_down_fixed_num, beam_down_fixed_num, roof_down_fixed_num, unground_down_fixed_num,
                                 beam_max_height, approx_scanner_height + 0.5,
                                 approx_scanner_height, underground_thre, semantic_assisted, apply_roi_filter, roi_min_y, roi_max_y);

    initial_guess_tran(0, 3) = 0.5; //initialization

    for (int i = 1; i < frame_num; i++) //throughout all the frames
    {
        std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();
        if (i == 1) // for the first scan matching (no initial guess avaliable --> larger corr distance)
            add_length = 1.0;
        cblock_source->filename = filenames[i];
        cblock_source->unique_id = i * frame_step + frame_num_begin;
        cblock_source->is_single_scanline = false;
        if (poses_gt_body_cs.size() > 0)                                                                                      //check if gt pose is availiable
            cblock_source->pose_gt = calib_mat.inverse() * (poses_gt_body_cs[0].inverse() * poses_gt_body_cs[i]) * calib_mat; //set ground truth pose (from body to lidar coordinate system)
        dataio.read_pc_cloud_block(cblock_source);
        std::chrono::steady_clock::time_point tic_without_import_pc = std::chrono::steady_clock::now();

        if (FLAGS_apply_dist_filter)
            cfilter.dist_filter(cblock_source->pc_raw, FLAGS_min_dist_used, FLAGS_max_dist_used);

        if (vertical_ang_calib) //intrinsic angle correction
            cfilter.vertical_intrinsic_calibration(cblock_source->pc_raw, vertical_ang_correction_deg);

        // motion compensation [first step: using the last frame's transformation]
        if (motion_compensation_method == 1) //calculate from time-stamp
            cfilter.get_pts_timestamp_ratio_in_frame(cblock_source->pc_raw, true);
        else if (motion_compensation_method == 2)                                         //calculate from azimuth
            cfilter.get_pts_timestamp_ratio_in_frame(cblock_source->pc_raw, false, 90.0); //HESAI Lidar: 90.0 (y+ axis, clockwise), Velodyne Lidar: 180.0

        if (!strcmp(FLAGS_baseline_reg_method.c_str(), "ndt")) //baseline_method
            cfilter.voxel_downsample(cblock_source->pc_raw, cblock_source->pc_down, vf_downsample_resolution);
        else if (!strcmp(FLAGS_baseline_reg_method.c_str(), "gicp")) //baseline_method
            cfilter.voxel_downsample(cblock_source->pc_raw, cblock_source->pc_down, vf_downsample_resolution);
        else
            cfilter.extract_semantic_pts(cblock_source, vf_downsample_resolution, gf_grid_resolution, gf_max_grid_height_diff,
                                         gf_neighbor_height_diff, gf_max_height, ground_down_rate,
                                         nonground_down_rate, pca_neigh_r, pca_neigh_k,
                                         pca_linearity_thre, pca_planarity_thre, pca_curvature_thre,
                                         pca_linearity_thre_down, pca_planarity_thre_down,
                                         self_adapative_parameters_on, apply_scanner_filter,
                                         estimate_ground_normal_method, ground_normal_estimation_radius,
                                         use_dist_inverse_sampling, unit_weight_dist2station, use_curb_as_vertex,
                                         vertex_extraction_method, gf_grid_min_pt_num, gf_reliable_neighbor_grid_thre,
                                         gf_down_down_rate, pca_neigh_k_min, intensity_thre,
                                         pillar_direction_sin, beam_direction_sin, roof_normal_sin, facade_normal_sin,
                                         FLAGS_sharpen_with_nms_on, fixed_num_downsampling, ground_down_fixed_num, pillar_down_fixed_num,
                                         facade_down_fixed_num, beam_down_fixed_num, roof_down_fixed_num, unground_down_fixed_num,
                                         beam_max_height, approx_scanner_height + 0.5,
                                         approx_scanner_height, underground_thre, semantic_assisted, apply_roi_filter, roi_min_y, roi_max_y);

        //update local map //TODO: according to residual
        if (i % local_map_recalculation_frequency == 0)
            local_map_recalculate_feature_on = true;
        else
            local_map_recalculate_feature_on = false;

        if (i > initial_scan2scan_frame_num + 1)
            mmanager.update_local_map(cblock_local_map, cblock_target, local_map_radius, local_map_max_pt_num, vertex_keeping_num, append_frame_radius,
                                      map_based_dynamic_removal_on, used_feature_type, dynamic_removal_radius, dynamic_dist_thre_min, current_linear_velocity * 0.2,
                                      0.03, local_map_recalculate_feature_on); //2 * 0.1 * velocity (set as the max distance threshold for dynamic obejcts)
        else
            mmanager.update_local_map(cblock_local_map, cblock_target, local_map_radius, local_map_max_pt_num, vertex_keeping_num, append_frame_radius, false, used_feature_type);

        std::chrono::steady_clock::time_point tic_pgo = std::chrono::steady_clock::now();

        if (loop_closure_detection_on) //determine if we can add a new submap
            seg_new_submap = mmanager.judge_new_submap(accu_tran, accu_rot_deg, accu_frame, FLAGS_submap_accu_tran, FLAGS_submap_accu_rot, FLAGS_submap_accu_frame);
        else
            seg_new_submap = false;

        if (seg_new_submap) //add nodes and edges in pose graph for pose graph optimization (pgo)
        {
            LOG(INFO) << "Create new submap [" << submap_count << "]";
            cloudblock_Ptr current_cblock_local_map(new cloudblock_t(*cblock_local_map, true));
            current_cblock_local_map->id_in_strip = submap_count;
            current_cblock_local_map->unique_id = i - 1;                                                                                //save the last frame index
            current_cblock_local_map->pose_init = current_cblock_local_map->pose_lo;                                                    //init guess for pgo
            current_cblock_local_map->information_matrix_to_next = 1.0 / FLAGS_submap_accu_frame * scan2map_reg_con.information_matrix; //TODO: use the correct function by deducing Jacbobian of transformation

            //encode features in the submap
            current_cblock_local_map->merge_feature_points(current_cblock_local_map->pc_down, false, true);
            cfilter.non_max_suppress(current_cblock_local_map->pc_vertex, non_max_suppresssion_radius, false, current_cblock_local_map->tree_vertex);
            //pc_vertex need to be farther from the lidar (junyun fengbu) //TODO
            bsc.extract_bsc_features(current_cblock_local_map->pc_down, current_cblock_local_map->pc_vertex, 4, current_cblock_local_map->keypoint_bsc); //6DOF feature (4 features/CSs for one keypoints)
            current_cblock_local_map->pc_down.reset(new pcT());
            if (submap_count == 0)
                current_cblock_local_map->pose_fixed = true; //fixed the first submap
            LOG(INFO) << "node id [" << submap_count << "]";
            cblock_submaps.push_back(current_cblock_local_map); //add new node
            submap_count++;
            cooling_index--;
            if (submap_count > 1)
            {
                //add adjacent edge between current submap and the last submap
                confinder.add_adjacent_constraint(cblock_submaps, pgo_edges, submap_count);
                int registration_status_map2map = creg.mm_lls_icp(pgo_edges[pgo_edges.size() - 1], max_iteration_num_m2m, 1.5 * reg_corr_dis_thre_init,
                                                                  converge_tran, converge_rot_d, 1.5 * reg_corr_dis_thre_min, dis_thre_update_rate,
                                                                  used_feature_type, "1101", z_xy_balance_ratio,
                                                                  2 * pt2pt_residual_window, 2 * pt2pl_residual_window, 2 * pt2li_residual_window,
                                                                  pgo_edges[pgo_edges.size() - 1].Trans1_2, apply_intersection_filter, false, true, true); //use its information matrix for pgo
                if (registration_status_map2map <= 0)                                                                                                      //candidate wrong registration
                    mviewer.keep_visualize(reg_viewer);
                if (pgo_edges[pgo_edges.size() - 1].sigma > reliable_sigma_thre)                                                                                            // if the estimated posterior standard deviation of map to map registration is a bit large
                    pgo_edges[pgo_edges.size() - 1].Trans1_2 = pgo_edges[pgo_edges.size() - 1].block1->pose_lo.inverse() * pgo_edges[pgo_edges.size() - 1].block2->pose_lo; //still use the scan-to-map odometry's prediction (but we will keep the information matrix)
                else
                {
                    LOG(WARNING) << "we would trust the map to map registration, update currect pose";
                    cblock_local_map->pose_lo = pgo_edges[pgo_edges.size() - 1].block1->pose_lo * pgo_edges[pgo_edges.size() - 1].Trans1_2; //update current local map's pose
                    cblock_target->pose_lo = cblock_local_map->pose_lo;                                                                     //update target frame
                }

                constraints current_registration_edges;
                if (cooling_index < 0) //find registration edges and then do pgo
                {
                    confinder.clear_registration_constraint(pgo_edges);
                    int reg_edge_count = confinder.find_overlap_registration_constraint(cblock_submaps, current_registration_edges, neighbor_search_dist, min_iou_thre, 4, true);
                    int reg_edge_successful_count = 0;
                    for (int j = 0; j < reg_edge_count; j++)
                    {
                        pcTPtr cur_map_origin(new pcT()), cur_map_guess(new pcT()), cur_map_guess_kp(new pcT()), cur_map_tran(new pcT()), hist_map(new pcT());
                        pcTPtr target_cor(new pcT()), source_cor(new pcT());
                        current_registration_edges[j].block2->merge_feature_points(cur_map_origin, false);
                        current_registration_edges[j].block1->merge_feature_points(hist_map, false);
                        Eigen::Matrix4d init_mat = current_registration_edges[j].Trans1_2;
                        // global (coarse) registration by teaser or ransac (using bsc or fpfh as feature)
                        LOG(INFO) << "Transformation initial guess predicted by lidar odometry:\n"
                                  << init_mat;
                        if (FLAGS_teaser_based_global_registration_on)
                        {
                            creg.find_putable_feature_correspondence_bsc(current_registration_edges[j].block1->keypoint_bsc, current_registration_edges[j].block2->keypoint_bsc, 4,
                                                                         current_registration_edges[j].block1->pc_vertex, current_registration_edges[j].block2->pc_vertex, target_cor, source_cor, FLAGS_feature_corr_num);

                            bool teaser_status = creg.coarse_reg_teaser(target_cor, source_cor, init_mat, non_max_suppresssion_radius);

                            // if (launch_real_time_viewer)
                            // {
                            //     pcl::transformPointCloud(*cur_map_origin, *cur_map_guess, init_mat);
                            //     pcl::transformPointCloud(*current_registration_edges[j].block2->pc_vertex, *cur_map_guess_kp, init_mat);
                            //     mviewer.display_correspondences_compare(feature_viewer, current_registration_edges[j].block2->pc_vertex, current_registration_edges[j].block1->pc_vertex,
                            //                                             cur_map_origin, hist_map, cur_map_guess_kp, cur_map_guess, current_registration_edges[j].Trans1_2);
                            // }
                            if (teaser_status)
                                init_mat = confinder.double_check_tran(init_mat, current_registration_edges[j].Trans1_2); //if the difference of the transformation of teaser and lo initial guess is too large, we will trust lidar odometry
                        }
                        int registration_status_map2map = creg.mm_lls_icp(current_registration_edges[j], max_iteration_num_m2m, 3.0 * reg_corr_dis_thre_init,
                                                                          converge_tran, converge_rot_d, 2.0 * reg_corr_dis_thre_min, dis_thre_update_rate,
                                                                          used_feature_type, "1101", z_xy_balance_ratio,
                                                                          2 * pt2pt_residual_window, 2 * pt2pl_residual_window, 2 * pt2li_residual_window,
                                                                          init_mat, apply_intersection_filter, false, true, true);
                        if (registration_status_map2map > 0)
                        {
                            pgo_edges.push_back(current_registration_edges[j]);
                            reg_edge_successful_count++;
                            mviewer.keep_visualize(reg_viewer);
                        }
                        // else //candidate wrong registration
                        //     mviewer.keep_visualize(reg_viewer);
                        LOG(INFO) << "map to map registration done\nsubmap [" << current_registration_edges[j].block1->id_in_strip << "] - [" << current_registration_edges[j].block2->id_in_strip << "]:\n"
                                  << current_registration_edges[j].Trans1_2;
                        if (launch_real_time_viewer) //for visualization
                        {
                            pcl::transformPointCloud(*cur_map_origin, *cur_map_guess, init_mat);
                            pcl::transformPointCloud(*cur_map_origin, *cur_map_tran, current_registration_edges[j].Trans1_2);
                            mviewer.set_show_reg();
                            mviewer.display_2_pc_compare_realtime(cur_map_guess, hist_map, cur_map_tran, hist_map, reg_viewer, display_time_ms);
                            mviewer.set_show_reg(0);
                        }
                        pcT().swap(*cur_map_origin);
                        pcT().swap(*cur_map_guess);
                        pcT().swap(*cur_map_tran);
                        pcT().swap(*hist_map);
                    }
                    if (reg_edge_successful_count > 0) //apply pose graph optimization (pgo)
                    {
                        bool pgo_successful;
                        if (!strcmp(FLAGS_pose_graph_optimization_method.c_str(), "g2o"))
                            pgo_successful = pgoptimizer.optimize_pose_graph_g2o(cblock_submaps, pgo_edges);
                        else if (!strcmp(FLAGS_pose_graph_optimization_method.c_str(), "ceres"))
                            pgo_successful = pgoptimizer.optimize_pose_graph_ceres(cblock_submaps, pgo_edges, 1.0, 0.02);
                        else if (!strcmp(FLAGS_pose_graph_optimization_method.c_str(), "gtsam"))
                            pgo_successful = pgoptimizer.optimize_pose_graph_gtsam(cblock_submaps, pgo_edges); //TODO: you'd better use gtsam instead (just like lego-loam)
                        else                                                                                   //default: ceres
                            pgo_successful = pgoptimizer.optimize_pose_graph_ceres(cblock_submaps, pgo_edges, 1.0, 0.02);
                        if (pgo_successful)
                        {
                            pgoptimizer.update_optimized_nodes(cblock_submaps, true);              //let pose_lo = = pose_init = pose_optimized
                            cblock_local_map->pose_lo = cblock_submaps[submap_count - 1]->pose_lo; //update current local map's pose
                            cblock_target->pose_lo = cblock_local_map->pose_lo;                    //update target frame
                            cooling_index = cooling_submap_num;                                    //wait for several submap (without loop closure detection)
                            for (int k = 0; k < cblock_submaps.size(); k++)
                                cblock_submaps[k]->pose_stable = true;
                        }
                    }
                }
                constraints().swap(current_registration_edges);
            }
        }
        std::chrono::steady_clock::time_point toc_pgo = std::chrono::steady_clock::now();

        //scan to scan registration
        if (scan_to_scan_module || i <= initial_scan2scan_frame_num)
        {
            creg.assign_source_target_cloud(cblock_target, cblock_source, scan2scan_reg_con);
            if (!strcmp(FLAGS_baseline_reg_method.c_str(), "ndt")) //baseline_method
                creg.omp_ndt(scan2scan_reg_con, reg_voxel_size, ndt_direct_searching_on,
                             initial_guess_tran, apply_intersection_filter);
            else if (!strcmp(FLAGS_baseline_reg_method.c_str(), "gicp")) //baseline_method
                creg.omp_gicp(scan2scan_reg_con, max_iteration_num_s2s, reg_corr_dis_thre_init + add_length, voxel_gicp_on,
                              reg_voxel_size, initial_guess_tran, apply_intersection_filter);
            else
            {
                int registration_status_scan2scan = creg.mm_lls_icp(scan2scan_reg_con, max_iteration_num_s2s, reg_corr_dis_thre_init + add_length,
                                                                    converge_tran, converge_rot_d, reg_corr_dis_thre_min, dis_thre_update_rate,
                                                                    used_feature_type, weight_strategy, z_xy_balance_ratio,
                                                                    pt2pt_residual_window, pt2pl_residual_window, pt2li_residual_window,
                                                                    initial_guess_tran, apply_intersection_filter, apply_motion_compensation_while_registration);
                if (registration_status_scan2scan < 0) //candidate wrong registration
                {
                    add_length = 0.8;
                    lo_status_healthy = false;
                    mviewer.keep_visualize(map_viewer); //pause
                }
                else
                    add_length = 1.0;
            }
            if (apply_zupt)
                nav.zupt_simple(scan2scan_reg_con.Trans1_2);
            cblock_source->pose_lo = cblock_target->pose_lo * scan2scan_reg_con.Trans1_2;
            LOG(INFO) << "scan to scan registration done\nframe [" << i - 1 << "] - [" << i << "]:\n"
                      << scan2scan_reg_con.Trans1_2;
            initial_guess_tran = scan2scan_reg_con.Trans1_2;
        }
        //scan to map registration
        if (i % s2m_frequency == 0 && i > initial_scan2scan_frame_num)
        {
            creg.assign_source_target_cloud(cblock_local_map, cblock_source, scan2map_reg_con);

            if (!strcmp(FLAGS_baseline_reg_method.c_str(), "ndt")) //baseline_method
                creg.omp_ndt(scan2map_reg_con, reg_voxel_size, ndt_direct_searching_on,
                             initial_guess_tran, apply_intersection_filter);
            if (!strcmp(FLAGS_baseline_reg_method.c_str(), "gicp")) //baseline_method
                creg.omp_gicp(scan2map_reg_con, max_iteration_num_s2s, reg_corr_dis_thre_init + add_length,
                              voxel_gicp_on, reg_voxel_size, initial_guess_tran, apply_intersection_filter);
            else //TODO: update according to feature point residual
            {
                int registration_status_scan2map = creg.mm_lls_icp(scan2map_reg_con, max_iteration_num_s2m, reg_corr_dis_thre_init + add_length,
                                                                   converge_tran, converge_rot_d, reg_corr_dis_thre_min, dis_thre_update_rate,
                                                                   used_feature_type, weight_strategy, z_xy_balance_ratio,
                                                                   pt2pt_residual_window, pt2pl_residual_window, pt2li_residual_window,
                                                                   initial_guess_tran, apply_intersection_filter, apply_motion_compensation_while_registration);
                if (registration_status_scan2map < 0) //candidate wrong registration
                {
                    add_length = 1.0;
                    lo_status_healthy = false;
                    mviewer.keep_visualize(map_viewer); //pause
                }
                else
                    add_length = 0.0;
            }
            if (apply_zupt)
                nav.zupt_simple(scan2map_reg_con.Trans1_2);
            cblock_source->pose_lo = cblock_local_map->pose_lo * scan2map_reg_con.Trans1_2;
            LOG(INFO) << "scan to map registration done\nframe [" << i << "]:\n"
                      << scan2map_reg_con.Trans1_2;
        }

        adjacent_pose_out = cblock_source->pose_lo.inverse() * cblock_target->pose_lo; //transformation from k to k+1 frame, the inverse of motion

        std::chrono::steady_clock::time_point toc_lo = std::chrono::steady_clock::now();
        if (apply_motion_compensation_while_registration)
        {
            std::chrono::steady_clock::time_point tic_undistortion = std::chrono::steady_clock::now();
            cfilter.apply_motion_compensation(cblock_source->pc_raw, adjacent_pose_out);
            cfilter.batch_apply_motion_compensation(cblock_source->pc_ground, cblock_source->pc_pillar, cblock_source->pc_facade,
                                                    cblock_source->pc_beam, cblock_source->pc_roof, cblock_source->pc_vertex, adjacent_pose_out);
            cfilter.batch_apply_motion_compensation(cblock_source->pc_ground_down, cblock_source->pc_pillar_down, cblock_source->pc_facade_down,
                                                    cblock_source->pc_beam_down, cblock_source->pc_roof_down, cblock_source->pc_vertex, adjacent_pose_out); //TODO: we do not need vertex feature
            std::chrono::steady_clock::time_point toc_undistortion = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_used_undistortion = std::chrono::duration_cast<std::chrono::duration<double>>(toc_undistortion - tic_undistortion);
            //LOG(INFO) << "map motion compensation done in [" << 1000.0 * time_used_undistortion.count() << "] ms.\n";
        }
        std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();

        if (launch_real_time_viewer) //visualization
        {
            std::chrono::steady_clock::time_point tic_vis = std::chrono::steady_clock::now();
            pcTPtr pointCloudS_reg(new pcT());
            pcl::transformPointCloud(*cblock_source->pc_raw, *pointCloudS_reg, adjacent_pose_out.inverse().cast<float>());
            pcl::transformPointCloud(*cblock_source->pc_raw, *cblock_source->pc_raw_w, cblock_source->pose_lo); //get raw point cloud in world coordinate system of current frame
            if (i % s2m_frequency == 0)
                mviewer.display_feature_pts_compare_realtime(cblock_local_map, cblock_source, feature_viewer, display_time_ms);
            else
                mviewer.display_feature_pts_compare_realtime(cblock_target, cblock_source, feature_viewer, display_time_ms);
            mviewer.display_2_pc_compare_realtime(cblock_source->pc_raw, cblock_target->pc_raw,
                                                  pointCloudS_reg, cblock_target->pc_raw, reg_viewer, display_time_ms);
            mviewer.display_lo_realtime(cblock_source, map_viewer, display_time_ms, downsamping_rate_scan_vis, downsamping_rate_map_vis);
            mviewer.display_dense_map_realtime(cblock_source, map_viewer, dense_map_keep_frame_num, display_time_ms);
            mviewer.display_feature_map_realtime(cblock_local_map, map_viewer, display_time_ms);
            if (seg_new_submap)
            {
                mviewer.display_pg_realtime(pgo_edges, map_viewer, display_time_ms);
                mviewer.display_2d_bbx_realtime(cblock_submaps, map_viewer, display_time_ms);
            }
            pcT().swap(*pointCloudS_reg);
            std::chrono::steady_clock::time_point toc_vis = std::chrono::steady_clock::now();
            std::chrono::duration<double> vis_time_used_per_frame = std::chrono::duration_cast<std::chrono::duration<double>>(toc_vis - tic_vis);
            LOG(INFO) << "Render frame [" << i << "] in [" << 1000.0 * vis_time_used_per_frame.count() << "] ms.\n";
        }
        std::chrono::steady_clock::time_point tic_output = std::chrono::steady_clock::now();

        if (i == 1) //write out pose
        {
            dataio.write_lo_pose_overwrite(adjacent_pose_out, output_adjacent_lo_pose_file);
            mviewer.is_seed_origin_ = false;
        }
        else
            dataio.write_lo_pose_append(adjacent_pose_out, output_adjacent_lo_pose_file);

        Eigen::Matrix4d pose_lo_body_frame;
        pose_lo_body_frame = poses_lo_body_cs[0] * calib_mat * cblock_source->pose_lo * calib_mat.inverse();
        dataio.write_lo_pose_append(cblock_source->pose_lo, output_lo_lidar_pose_file); //lo pose in lidar frame
        dataio.write_lo_pose_append(cblock_source->pose_gt, output_gt_lidar_pose_file); //gt pose in lidar frame
        dataio.write_lo_pose_append(pose_lo_body_frame, output_lo_body_pose_file);      //lo pose in body frame (requried by KITTI)
        poses_lo_lidar_cs.push_back(cblock_source->pose_lo);
        poses_gt_lidar_cs.push_back(cblock_source->pose_gt);
        poses_lo_body_cs.push_back(pose_lo_body_frame);
        poses_lo_adjacent.push_back(adjacent_pose_out);

        //update initial guess
        initial_guess_tran.setIdentity();
        if (initial_guess_mode == 1 && lo_status_healthy)
            initial_guess_tran.block<3, 1>(0, 3) = adjacent_pose_out.inverse().block<3, 1>(0, 3); //uniform motion model
        else if (initial_guess_mode == 2 && lo_status_healthy)
            initial_guess_tran = adjacent_pose_out.inverse();

        //save current frame (only metadata)
        cloudblock_Ptr current_cblock_frame(new cloudblock_t(*cblock_target));
        current_cblock_frame->pose_optimized = current_cblock_frame->pose_lo;
        current_cblock_frame->information_matrix_to_next = scan2map_reg_con.information_matrix;
        cblock_frames.push_back(current_cblock_frame);
        //use this frame as the next iter's target frame
        cblock_target.swap(cblock_source);
        cblock_source->free_all();
        lo_status_healthy = true;

        //update accumulated information
        accu_tran += nav.cal_translation_from_tranmat(adjacent_pose_out);
        accu_rot_deg += nav.cal_rotation_deg_from_tranmat(adjacent_pose_out);
        accu_frame += frame_step;
        current_linear_velocity = nav.cal_velocity(poses_lo_adjacent);

        //report timing
        std::chrono::steady_clock::time_point toc_output = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used_per_frame_lo_1 = std::chrono::duration_cast<std::chrono::duration<double>>(tic_pgo - tic_without_import_pc);
        std::chrono::duration<double> time_used_per_frame_lo_2 = std::chrono::duration_cast<std::chrono::duration<double>>(toc_lo - toc_pgo);
        std::chrono::duration<double> time_used_per_frame_1 = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
        std::chrono::duration<double> time_used_per_frame_2 = std::chrono::duration_cast<std::chrono::duration<double>>(toc_output - tic_output);
        time_count += (time_used_per_frame_lo_1.count() + time_used_per_frame_lo_2.count());
        LOG(INFO) << "Consuming time of lidar odometry for current frame is [" << 1000.0 * (time_used_per_frame_lo_1.count() + time_used_per_frame_lo_2.count()) << "] ms.\n";
        LOG(INFO) << "Process frame [" << i << "] in [" << 1000.0 * (time_used_per_frame_1.count() + time_used_per_frame_2.count()) << "] ms.\n";
    }
    cloudblock_Ptr current_cblock_frame(new cloudblock_t(*cblock_target));
    current_cblock_frame->pose_optimized = current_cblock_frame->pose_lo;
    cblock_frames.push_back(current_cblock_frame);

    LOG(INFO) << "Lidar Odometry done. Average processing time per frame is ["
              << 1000.0 * time_count / frame_num << "] ms over [" << frame_num << "] frames\n";

    if (loop_closure_detection_on)
    {
        //post processing : refine pose within the submap and output final map
        //update pose of each frame in each submap using pgo
        // 1---2---3---4---5
        // |               |
        // -----------------
        std::chrono::steady_clock::time_point tic_inner_submap_refine = std::chrono::steady_clock::now();
        for (int i = 1; i < cblock_submaps.size(); i++)
        {
            cloudblock_Ptrs cblock_frames_in_submap;
            constraints inner_submap_edges;
            cblock_frames_in_submap.push_back(cblock_frames[cblock_submaps[i - 1]->unique_id]); //end frame of the last submap
            cblock_frames_in_submap[0]->id_in_strip = 0;
            cblock_frames_in_submap[0]->pose_fixed = true; //fix the first frame
            cblock_frames_in_submap[0]->pose_init = cblock_submaps[i - 1]->pose_lo;
            int node_count = 1;
            for (int j = cblock_submaps[i - 1]->unique_id; j < cblock_submaps[i]->unique_id; j++) //last submap's end frame to this submap's end frame (index j)
            {
                Eigen::Matrix4d tran_mat_12 = poses_lo_adjacent[j].inverse();
                cblock_frames[j + 1]->id_in_strip = node_count;
                if (j < cblock_submaps[i]->unique_id - 1)
                    cblock_frames[j + 1]->pose_init = cblock_frames[j]->pose_init * tran_mat_12;
                cblock_frames_in_submap.push_back(cblock_frames[j + 1]);
                node_count++;
                confinder.add_adjacent_constraint(cblock_frames_in_submap, inner_submap_edges, tran_mat_12, node_count);
            }
            cblock_frames_in_submap[node_count - 1]->pose_fixed = true; //fix the last frame
            cblock_frames_in_submap[node_count - 1]->pose_init = cblock_submaps[i]->pose_lo;
            if (!strcmp(FLAGS_pose_graph_optimization_method.c_str(), "g2o"))
                pgoptimizer.optimize_pose_graph_g2o(cblock_frames_in_submap, inner_submap_edges);
            else if (!strcmp(FLAGS_pose_graph_optimization_method.c_str(), "ceres"))
                pgoptimizer.optimize_pose_graph_ceres(cblock_frames_in_submap, inner_submap_edges, 0.1, 0.01); //set the limit better
            else if (!strcmp(FLAGS_pose_graph_optimization_method.c_str(), "gtsam"))
                pgoptimizer.optimize_pose_graph_gtsam(cblock_frames_in_submap, inner_submap_edges);
            else //default: ceres
                pgoptimizer.optimize_pose_graph_ceres(cblock_frames_in_submap, inner_submap_edges, 0.1, 0.01);

            for (int j = cblock_submaps[i - 1]->unique_id + 1; j <= cblock_submaps[i]->unique_id; j++)
                cblock_frames[j]->pose_optimized = cblock_frames_in_submap[cblock_frames[j]->id_in_strip]->pose_optimized;
            //cblock_frames[j]->pose_optimized = cblock_frames[j]->pose_init;
            constraints().swap(inner_submap_edges);
            cloudblock_Ptrs().swap(cblock_frames_in_submap);
            LOG(INFO) << "Inner-submap pose refining done for submap [" << i << "]";
        }
        std::chrono::steady_clock::time_point toc_inner_submap_refine = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used_inner_submap_refine = std::chrono::duration_cast<std::chrono::duration<double>>(toc_inner_submap_refine - tic_inner_submap_refine);
        LOG(INFO) << "Inner-submap pose refinement done in [" << 1000 * time_used_inner_submap_refine.count() << "] ms.";
    }

    if (loop_closure_detection_on)
    {
        for (int i = 0; i < cblock_submaps.size(); i++) //free submaps' point cloud
            cblock_submaps[i]->free_all();
        for (int i = 0; i < frame_num; i++) //update poses
        {
            poses_lo_lidar_cs[i] = cblock_frames[i]->pose_optimized;
            if (i == 0) //write optimized pose out (overwrite)
                dataio.write_lo_pose_overwrite(poses_lo_lidar_cs[i], output_lo_lidar_pose_file);
            else
            {
                poses_lo_body_cs[i] = poses_lo_body_cs[0] * calib_mat * poses_lo_lidar_cs[i] * calib_mat.inverse();
                dataio.write_lo_pose_append(poses_lo_lidar_cs[i], output_lo_lidar_pose_file);
            }
        }
    }
    //refresh the map
    map_viewer->removeAllShapes();
    map_viewer->removeAllPointClouds();
    mviewer.update_lo_pose(poses_lo_lidar_cs, poses_gt_lidar_cs, map_viewer, display_time_ms);
    mviewer.keep_visualize(map_viewer);

    if (write_out_map || write_out_gt_map) //export map point cloud
    {
        pcTPtr pc_map_merged(new pcT);
        pcTPtr pc_map_gt_merged(new pcT);
        //output merged map (with dist_filter, intrinsic correction and motion distortion
        for (int i = 0; i < frame_num; i++)
        {
            dataio.read_pc_cloud_block(cblock_frames[i]);

            if (vertical_ang_calib) //intrinsic angle correction
                cfilter.vertical_intrinsic_calibration(cblock_frames[i]->pc_raw, vertical_ang_correction_deg);
            if (FLAGS_apply_dist_filter)
                cfilter.dist_filter(cblock_frames[i]->pc_raw, FLAGS_min_dist_mapping, FLAGS_max_dist_mapping);
            cfilter.random_downsample(cblock_frames[i]->pc_raw, FLAGS_map_downrate_output);
            if (motion_compensation_method == 1) //calculate from time-stamp
                cfilter.get_pts_timestamp_ratio_in_frame(cblock_frames[i]->pc_raw, true);
            else if (motion_compensation_method == 2)                                            //calculate from azimuth
                cfilter.get_pts_timestamp_ratio_in_frame(cblock_frames[i]->pc_raw, false, 90.0); //HESAI Lidar: 90.0 (y+ axis, clockwise), Velodyne Lidar: 180.0
            if (write_out_map)
            {
                if (motion_compensation_method > 0 && i > 0)
                {
                    Eigen::Matrix4d adjacent_tran = cblock_frames[i]->pose_optimized.inverse() * cblock_frames[i - 1]->pose_optimized;
                    cfilter.apply_motion_compensation(cblock_frames[i]->pc_raw, adjacent_tran);
                }
                pcl::transformPointCloud(*cblock_frames[i]->pc_raw, *cblock_frames[i]->pc_raw_w, cblock_frames[i]->pose_optimized);
                if (FLAGS_write_map_each_frame)
                {
                    std::string filename_without_path = cblock_frames[i]->filename.substr(cblock_frames[i]->filename.rfind('/') + 1);
                    std::string filename_without_extension = filename_without_path.substr(0, filename_without_path.rfind('.'));
                    std::string output_pc_file = output_pc_folder + "/" + filename_without_extension + ".pcd";
                    dataio.write_cloud_file(output_pc_file, cblock_frames[i]->pc_raw_w);
                }
                pc_map_merged->points.insert(pc_map_merged->points.end(), cblock_frames[i]->pc_raw_w->points.begin(), cblock_frames[i]->pc_raw_w->points.end());
                cfilter.random_downsample(cblock_frames[i]->pc_raw_w, 10);
                mviewer.display_dense_map_realtime(cblock_frames[i], map_viewer, frame_num, display_time_ms);
            }
            if (write_out_gt_map)
            {
                if (motion_compensation_method > 0 && i > 0)
                {
                    Eigen::Matrix4d adjacent_tran = cblock_frames[i]->pose_gt.inverse() * cblock_frames[i - 1]->pose_gt;
                    cfilter.apply_motion_compensation(cblock_frames[i]->pc_raw, adjacent_tran);
                }
                pcl::transformPointCloud(*cblock_frames[i]->pc_raw, *cblock_frames[i]->pc_sketch, cblock_frames[i]->pose_gt);
                pc_map_gt_merged->points.insert(pc_map_gt_merged->points.end(), cblock_frames[i]->pc_sketch->points.begin(), cblock_frames[i]->pc_sketch->points.end());
            }
            cblock_frames[i]->free_all();
        }

        //TODO: add more map based operation
        //1.generate 2D geo-referenced image
        //2.intensity generalization
        if (FLAGS_map_filter_on)
            cfilter.sor_filter(pc_map_merged, 20, 2.0); //sor filtering before output

        std::string output_merged_map_file = output_pc_folder + "/" + "merged_map.pcd";
        if (write_out_map)
            dataio.write_pcd_file(output_merged_map_file, pc_map_merged); //write merged map point cloud
        std::string output_merged_gt_map_file = output_pc_folder + "/" + "merged_gt_map.pcd";
        if (write_out_gt_map)
            dataio.write_pcd_file(output_merged_gt_map_file, pc_map_gt_merged); //write merged map point cloud
        pcT().swap(*pc_map_merged);
        pcT().swap(*pc_map_gt_merged);
    }

    //output trajectory as point cloud (timestamp as intensity)
    if (write_out_map)
    {
        dataio.write_pose_point_cloud(lo_lidar_pose_point_cloud_file, poses_lo_lidar_cs);
        dataio.write_pose_point_cloud(gt_lidar_pose_point_cloud_file, poses_gt_lidar_cs);
    }

    //evaluation of the estimated pose
    OdomErrorCompute ec;
    std::vector<odom_errors_t> odom_errs;
    odom_errs = ec.compute_error(poses_gt_body_cs, poses_lo_body_cs);
    ec.print_error(odom_errs);

    mviewer.keep_visualize(map_viewer);

    return true;
}

//TODO LIST
//TODO: you must get ATE better than 0.45 on KITTI seq00 in order to get ATE 0.60 on KITTI testing sequences

//TODO: add sensor integration interfaces, recursive estimation interface, loop closure detection and pose graph optimization

//TODO: add ros support

//TODO: use deep learning based methods to do global registration (for loop closure)

//TODO: do semanctic segmentation based on deep learning

//TODO: train the saliency feature (like handcrafted linearity, planarity, etc.) and use them to detect the keypoints

//TODO: add map related module: 2D gray map (for lane extraction), Octo map, Voxel map, Feature point map, Dense point map, etc.

//multi-thread
//1. main: lidar odometry
//2. find loop closure and apply optimization
//3. viusalization and output

// for kitti 20 [ROI for dynamic objects removal]
// if (i<240)
// {
//     roi_min_y = -3.0;
//     roi_max_y = 9.0;
// }
// else if (i >= 240 && i < 600)
// {
//     roi_min_y = 0.0;
//     roi_max_y = 10.0;
// }
// else if (i >= 600)
// {
//     roi_min_y = 0.0;
//     roi_max_y = 0.0;
// }