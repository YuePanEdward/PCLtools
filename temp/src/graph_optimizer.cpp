#include "graph_optimizer.h"
#include "utility.hpp"
#include "dataio.hpp"

#if G2O_ON
//g2o
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/types/slam3d/types_slam3d.h"
#endif

#if CERES_ON
//ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#endif

#if GTSAM_ON
//gtsam
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#endif

#include <glog/logging.h>

using namespace std;

namespace lo
{
bool GlobalOptimize::optimize_pose_graph_g2o(cloudblock_Ptrs &all_blocks, constraints &all_cons)
{
	//1.Create the optimizer, determine the optimization method such as LM
	//2.Assign value to the nodes(initial value to unknown parameters), fixed the datum
	//3.Assign value to the edges(observation), assign the weight matrix and the robust kernel function such as Huber
	//4.Solving

	LOG(INFO) << "Start to optimize the pose graph using g2o";

	init();

	set_pgo_options_g2o();
	set_pgo_problem_g2o(all_blocks, all_cons);
	if (!solve_pgo_problem_g2o())
		return false;

	// assign the optimized value
	assign_pose_optimized_g2o(all_blocks);
	if (update_optimized_edges(all_cons))
		return true;
	else //too much wrong edges, pgo may fail
		return false;
}

//G2O has some problem, use ceres instead
bool GlobalOptimize::set_pgo_options_g2o()
{
#if G2O_ON
	//g2o::SparseOptimizer *optimizer;

	typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
	typedef g2o::LinearSolver<BlockSolverType::PoseMatrixType> LinearSolverType;

	LinearSolverType *linearSolver = 0;
	//std::unique_ptr<LinearSolverType> linearSolver(new LinearSolverType());

	if (param_.linear_solver == "dense_schur")
		linearSolver = new g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>();
	else if (param_.linear_solver == "sparse_schur")
	{
		// linearSolver = new g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>();
		// dynamic_cast<g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType> *>(linearSolver)->setBlockOrdering(true);
	}
	else
		linearSolver = new g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>();

	BlockSolverType *blockSolver;
	blockSolver = new BlockSolverType(linearSolver);
	//std::unique_ptr<BlockSolverType> blockSolver(new BlockSolverType(std::move(linearSolver))); //new version g2o

	g2o::OptimizationAlgorithmWithHessian *solver;
	//g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(solver_ptr));
	//g2o::OptimizationAlgorithmWithHessian *solver = new g2o::OptimizationAlgorithmWithHessian(std::move(blockSolver));

	if (param_.trust_region_strategy == "levenberg_marquardt")
		solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
	else if (param_.trust_region_strategy == "dogleg")
		solver = new g2o::OptimizationAlgorithmDogleg(blockSolver);
	else
		LOG(ERROR) << "Please check your trust_region_strategy, using default setting (L-M)";

	//g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

	g2o_optimizer_->setAlgorithm(solver);
	g2o_optimizer_->setVerbose(param_.verbose);

	return true;
#endif
	return false;
}

bool GlobalOptimize::set_pgo_problem_g2o(cloudblock_Ptrs &all_blocks, constraints &all_cons)
{
#if G2O_ON
	//std::vector<g2o::VertexSE3Expmap*> pose_vertices;
	//std::vector<g2o::EdgeSE3Expmap*> edges_smo;
	//std::vector<g2o::EdgeSE3Expmap*> edges_reg;

	//Set Nodes
	for (int i = 0; i < all_blocks.size(); i++)
	{
		g2o::VertexSE3Expmap *pose_v = new g2o::VertexSE3Expmap();
		pose_v->setId(all_blocks[i]->id_in_strip);

		if (all_blocks[i]->pose_fixed)
			pose_v->setFixed(true);

		Eigen::Matrix4d node_pose = all_blocks[i]->pose_init;
		pose_v->setEstimate(g2o::SE3Quat(node_pose.topLeftCorner<3, 3>(), node_pose.topRightCorner<3, 1>())); //Initial guess
		g2o_optimizer_->addVertex(pose_v);																	  //Add nodes;
																											  //pose_vertices.push_back(pose_v);
	}

	//approximate block error: tx,ty,tz: 0.05 m, roll,pitch,yaw: 0.25 deg
	Matrix6d vc_matrix;

	// set_adjacent_edge_information_matrix(vc_matrix);

	//Set Edges
	for (int j = 0; j < all_cons.size(); j++)
	{
		if (all_cons[j].con_type != NONE)
		{
			// if (all_cons[j].con_type == ADJACENT)
			// 	all_cons[j].information_matrix = 4.0 * all_cons[0].information_matrix; //the frist one should be registration edge //TODO fix

			g2o::EdgeSE3Expmap *pose_e = new g2o::EdgeSE3Expmap();
			pose_e->setId(j);

			//g2o's edge is from vertex 0 to vertex 1
			//The transformation in constraint is from block 2 (v1) to block 1 (v0) (Trans1_2)
			int v0 = all_cons[j].block1->unique_id;
			int v1 = all_cons[j].block2->unique_id;
			pose_e->setVertex(0, g2o_optimizer_->vertices()[v1]); // block 2 -> v1 -> vertex 0
			pose_e->setVertex(1, g2o_optimizer_->vertices()[v0]); // block 1 -> v0 -> vertex 1

			Eigen::Matrix4d edge_pose = all_cons[j].Trans1_2;
			pose_e->setMeasurement(g2o::SE3Quat(edge_pose.topLeftCorner<3, 3>(), edge_pose.topRightCorner<3, 1>())); //Set the transformation measurement
			//float weight = determine_weight(all_cons[j].block1->data_type, all_cons[j].block2->data_type, all_cons[j].con_type);
			Matrix6d info_mat;

			// info_mat = all_cons[j].information_matrix.cast<double>(); //Set the Information (Weight) Matrix [normalization to mm level]
			// info_mat.block<3, 3>(3, 3) = 1e-8 * info_mat.block<3, 3>(3, 3);
			// info_mat.block<3, 3>(3, 0) = 1e-4 * info_mat.block<3, 3>(3, 0);
			// info_mat.block<3, 3>(0, 3) = 1e-4 * info_mat.block<3, 3>(0, 3);

			//read the paper about quaternion's covariance calculation

			if (param_.use_equal_weight)
			{
				info_mat.setIdentity();
				//info_mat.block<3, 3>(0, 0) = 1e-8 * info_mat.block<3, 3>(0, 0);
			}

			pose_e->setInformation(info_mat);

			if (param_.robustify)
			{
				g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
				rk->setDelta(param_.robust_delta);
				pose_e->setRobustKernel(rk); //Set the robust kernel function
			}

			g2o_optimizer_->addEdge(pose_e);
			//if (all_cons[j].con_type == 1)      edges_smo.push_back(pose_e);  //Smooth Edges
			//else if (all_cons[j].con_type == 2) edges_reg.push_back(pose_e);  //Registration Edges

			// LOG(INFO) << "set con " << all_cons[j].unique_id << " : " << all_cons[j].block1->unique_id << " - " << all_cons[j].block2->unique_id << std::endl
			// 		  << all_cons[j].Trans1_2;
		}
	}

	return true;
#endif
	return false;
}

bool GlobalOptimize::solve_pgo_problem_g2o()
{
#if G2O_ON
	//Optimize
	LOG(INFO) << "=============================================  Optimization Start  =============================================";
	g2o_optimizer_->initializeOptimization();
	g2o_optimizer_->optimize(param_.num_iterations); // Number of Iterations
	LOG(INFO) << "==============================================  Optimization END  ==============================================";
	return true;
#endif
	return false;
}

bool GlobalOptimize::assign_pose_optimized_g2o(cloudblock_Ptrs &all_blocks)
{
#if G2O_ON
	for (int i = 0; i < all_blocks.size(); i++)
	{
		g2o::VertexSE3Expmap *v_o = dynamic_cast<g2o::VertexSE3Expmap *>(g2o_optimizer_->vertices()[i]);
		Eigen::Isometry3d pose_o = v_o->estimate();
		all_blocks[i]->pose_optimized = pose_o.matrix();
		all_blocks[i]->pose_lo = all_blocks[i]->pose_optimized;

		LOG(INFO) << "Optimized pose of node [" << all_blocks[i]->id_in_strip << "]:\n"
				  << all_blocks[i]->pose_optimized;
	}
	return true;
#endif
	return false;
}

//TODO:
bool GlobalOptimize::robust_verify_chi2_g2o()
{
	//To guarentee the robustness, chi2 test
	/*for (int i = 0; i < iterations.size(); ++i)
		{
		double last_active_chi2 = 0.0, epsilon = 0.000001;
		for (int j = 0; j < iterations[i]; ++j)
		{
		optimizer.initializeOptimization(0);
		optimizer.optimize(1);
		optimizer.computeActiveErrors();

		double active_chi2 = optimizer.activeRobustChi2();
		LOG(INFO) << j << "th active_chi2 is " << active_chi2;
		if (std::isnan(active_chi2))
		{
		LOG(WARNING) << "Essential Graph Optimization " << j << "th iteration chi2 is NAN!";
		bSuccess = false;
		bBreak = true;
		break;
		}
		if (std::isinf(active_chi2))
		{
		LOG(WARNING) << "Essential Graph Optimization " << j << "th iteration chi2 is INF!";
		bSuccess = false;
		bBreak = true;
		break;
		}

		double improvment = last_active_chi2 - active_chi2;
		if (j > 0 && improvment < epsilon) // negative/ not enough improvement
		{
		LOG(WARNING) << "Essential Graph Optimization negative/ not enough improvement(" << improvment << " < " << epsilon << ")";
		bBreak = true;
		break;
		}
		else if (i == 0 && active_chi2 < epsilon) // error is under epsilon
		{
		LOG(INFO) << "Essential Graph Optimization error(" << active_chi2 << ") is under epsilon(" << epsilon << ")";
		bBreak = true;
		break;
		}

		last_active_chi2 = active_chi2;
		}*/
	return true;
}

bool GlobalOptimize::optimize_pose_graph_ceres(cloudblock_Ptrs &all_blocks, constraints &all_cons,
											   double moving_threshold_tran, double moving_threshold_rot)
{
	LOG(INFO) << "Start to optimize the pose graph using ceres";

	//clear the problem, options, etc.
	init();

	set_pgo_options_ceres();

	if (!set_pgo_problem_ceres(all_blocks, all_cons, moving_threshold_tran, moving_threshold_rot))
		return false;
	if (!solve_pgo_problem_ceres())
		return false;

	// assign the optimized value
	assign_pose_optimized_ceres(all_blocks);
	if (update_optimized_edges(all_cons))
		return true;
	else //too much wrong edges, pgo may fail
		return false;
	return true;
}

bool GlobalOptimize::set_pgo_options_ceres()
{
#if CERES_ON
	ceres_options_->max_num_iterations = param_.num_iterations;
	ceres_options_->minimizer_progress_to_stdout = true;
	ceres_options_->num_threads = param_.num_threads;

	//ceres_options_->gradient_tolerance=1e-16;
	ceres_options_->function_tolerance = 1e-16;

	//robust kernel function
	ceres_robust_kernel_function_ = param_.robustify ? new ceres::HuberLoss(param_.robust_delta) : NULL;

	//example : "dense_schur"
	CHECK(ceres::StringToLinearSolverType(param_.linear_solver,
										  &ceres_options_->linear_solver_type));

	//example : "suite_sparse"
	CHECK(ceres::StringToSparseLinearAlgebraLibraryType(param_.sparse_linear_algebra_library,
														&ceres_options_->sparse_linear_algebra_library_type));

	//example : "eigen"
	CHECK(ceres::StringToDenseLinearAlgebraLibraryType(param_.dense_linear_algebra_library,
													   &ceres_options_->dense_linear_algebra_library_type));

	//example : "levenberg_marquardt"
	CHECK(ceres::StringToTrustRegionStrategyType(param_.trust_region_strategy,
												 &ceres_options_->trust_region_strategy_type));

	return true;
#endif
	return false;
}

bool GlobalOptimize::set_pgo_problem_ceres(cloudblock_Ptrs &all_blocks, constraints &all_cons,
										   double stable_threshold_tran, double stable_threshold_rot)
{
#if CERES_ON
	double fixed_threshold = 1e-14; //For reference (fixed) frame

	int edge_count = all_cons.size();
	int node_count = all_blocks.size();
	int fixed_node_count = 0;
	for (int i = 0; i < all_blocks.size(); i++)
	{
		//fix nodes or not
		if (all_blocks[i]->pose_fixed)
			fixed_node_count++;
	}

	LOG(INFO) << "PGO setup: [" << node_count << "] nodes, [" << fixed_node_count << "] fixed nodes, [" << edge_count << "] edges";

	if (node_count - fixed_node_count > edge_count)
	{
		LOG(ERROR) << "Edge number is not enough for the PGO";
		return false;
	}

	// Matrix6d vc_matrix;
	// set_adjacent_edge_information_matrix(vc_matrix);

	num_ceres_observations_ = 0;
	for (int i = 0; i < all_cons.size(); i++)
	{
		if (all_cons[i].con_type != NONE)
		{
			num_ceres_observations_++;
			if (param_.use_equal_weight)
			{
				all_cons[i].information_matrix.setIdentity();
				all_cons[i].information_matrix(3, 3) = 1000.0;
				all_cons[i].information_matrix(4, 4) = 1000.0;
				all_cons[i].information_matrix(5, 5) = 1000.0;
				// if (all_cons[i].con_type == ADJACENT)
				// 	all_cons[i].information_matrix = 4.0 * all_cons[i].information_matrix;
			}
			else //directly use information matrix
			{
				all_cons[i].information_matrix = 1e-6 * all_cons[i].information_matrix;
			}
		}
	}

	num_ceres_parameters_ = 7 * all_blocks.size(); //x,y,z,i,j,k,w

	//ceres_observations_ = new double[6 * num_ceres_observations_];
	ceres_parameters_ = new double[num_ceres_parameters_];

	//Set initial guess of parameters for estimating (original pose)
	for (int i = 0; i < all_blocks.size(); i++)
	{
		pose_qua_t node_pose;
		node_pose.SetPose(all_blocks[i]->pose_init);

		ceres_parameters_[7 * i + 0] = node_pose.trans(0); //x
		ceres_parameters_[7 * i + 1] = node_pose.trans(1); //y
		ceres_parameters_[7 * i + 2] = node_pose.trans(2); //z
		ceres_parameters_[7 * i + 3] = node_pose.quat.x(); //i
		ceres_parameters_[7 * i + 4] = node_pose.quat.y(); //j
		ceres_parameters_[7 * i + 5] = node_pose.quat.z(); //k
		ceres_parameters_[7 * i + 6] = node_pose.quat.w(); //w
	}

	//LOG(INFO) << "Begin to calculate the loss";

	for (int i = 0; i < all_cons.size(); i++)
	{
		if (all_cons[i].con_type != NONE)
		{
			pose_qua_t edge_tran;
			edge_tran.SetPose(all_cons[i].Trans1_2);

			ceres::CostFunction *cost_function =
				PoseGraph3dErrorTermQUAT::Create(edge_tran, all_cons[i].information_matrix);

			ceres_problem_->AddResidualBlock(cost_function,
											 ceres_robust_kernel_function_, // NULL /* squared loss */, /* new CauchyLoss(0.5) */
											 mutable_pose_tran(all_cons[i].block1->id_in_strip),
											 mutable_pose_quat(all_cons[i].block1->id_in_strip),
											 mutable_pose_tran(all_cons[i].block2->id_in_strip),
											 mutable_pose_quat(all_cons[i].block2->id_in_strip));
		}
	}

	int stable_index = 0;
	//set node limit //TODO: add more kinds of limit for none-fixed nodes
	for (int i = 0; i < all_blocks.size(); i++)
	{
		//fix nodes or not
		if (all_blocks[i]->pose_fixed)
		{
			fix_node_ceres(i, fixed_threshold, fixed_threshold);
			LOG(INFO) << "Node [" << i << "]: Fixed (t_limit:" << fixed_threshold << " , r_limit:" << fixed_threshold << " )";
		}
		else if (all_blocks[i]->pose_stable)
		{
			fix_node_ceres(i, stable_threshold_tran, stable_threshold_rot);
			stable_index = i;
			LOG(INFO) << "Node [" << i << "]: Stable (t_limit:" << stable_threshold_tran << " , r_limit:" << stable_threshold_rot << " )";
		}
		else
		{
			float moving_threshold_tran = (i - stable_index) * stable_threshold_tran;
			float moving_threshold_rot = (i - stable_index) * stable_threshold_rot;
			fix_node_ceres(i, moving_threshold_tran, moving_threshold_rot);
			LOG(INFO) << "Node [" << i << "]: Free (t_limit:" << moving_threshold_tran << " , r_limit:" << moving_threshold_rot << " )";
		}
	}
	//You must add the parameter block to the problem before you can set a lower bound on one of its components.

	return true;

#endif
	return false;
}

bool GlobalOptimize::fix_node_ceres(int block_id, float translate_thre, float quat_thre) // set the limit for the parameters for estimating
{
#if CERES_ON
	for (int j = 0; j < 3; j++)
	{
		ceres_problem_->SetParameterLowerBound(mutable_pose_tran(block_id), j, *(mutable_pose_tran(block_id) + j) - translate_thre);
		ceres_problem_->SetParameterUpperBound(mutable_pose_tran(block_id), j, *(mutable_pose_tran(block_id) + j) + translate_thre);
	}
	for (int j = 0; j < 4; j++)
	{
		ceres_problem_->SetParameterLowerBound(mutable_pose_quat(block_id), j, *(mutable_pose_quat(block_id) + j) - quat_thre);
		ceres_problem_->SetParameterUpperBound(mutable_pose_quat(block_id), j, *(mutable_pose_quat(block_id) + j) + quat_thre);
	}
	return true;
#endif
	return false;
}

bool GlobalOptimize::solve_pgo_problem_ceres()
{
#if CERES_ON

	ceres::Solve(*ceres_options_, ceres_problem_, ceres_summary_);

	LOG(INFO) << "=============================================  Optimization Start  =============================================";
	if (param_.verbose)
		LOG(INFO) << ceres_summary_->FullReport();
	else
		LOG(INFO) << ceres_summary_->BriefReport();
	LOG(INFO) << "==============================================  Optimization END  ==============================================";

	return ceres_summary_->IsSolutionUsable();
#endif
	return false;
}

//TO CHECK: let the unique id equal to the saving sequence of blocks
bool GlobalOptimize::assign_pose_optimized_ceres(cloudblock_Ptrs &all_blocks)
{
	//update nodes
	for (int i = 0; i < all_blocks.size(); i++)
	{
		pose_qua_t temp_pose;

		temp_pose.trans << ceres_parameters_[7 * i], ceres_parameters_[7 * i + 1], ceres_parameters_[7 * i + 2];
		temp_pose.quat.x() = ceres_parameters_[7 * i + 3];
		temp_pose.quat.y() = ceres_parameters_[7 * i + 4];
		temp_pose.quat.z() = ceres_parameters_[7 * i + 5];
		temp_pose.quat.w() = ceres_parameters_[7 * i + 6];

		temp_pose.quat.normalize();

		all_blocks[i]->pose_optimized = temp_pose.GetMatrix();

		LOG(INFO) << "transformation of node [" << all_blocks[i]->id_in_strip << "]:\n"
				  << all_blocks[i]->pose_init.inverse() * all_blocks[i]->pose_optimized;
	}
	return true;
}

bool GlobalOptimize::optimize_pose_graph_gtsam(cloudblock_Ptrs &all_blocks, constraints &all_cons)
{

	LOG(INFO) << "Start to optimize the pose graph using gtsam";

#if GTSAM_ON
	init();

	gtsam::Vector Vector6(6);
	Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
	priorNoise = noiseModel::Diagonal::Variances(Vector6);
	odometryNoise = noiseModel::Diagonal::Variances(Vector6);

	// make common variables at forward
	float x, y, z, roll, pitch, yaw;
	Eigen::Affine3f correctionCameraFrame;
	float noiseScore = 0.5; // constant is ok...
	gtsam::Vector Vector6(6);
	Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
	constraintNoise = noiseModel::Diagonal::Variances(Vector6);
	robustNoiseModel = gtsam::noiseModel::Robust::Create(
		gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure
		gtsam::noiseModel::Diagonal::Variances(Vector6)); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)

	if (isValidRSloopFactor == true)
	{
		correctionCameraFrame = icp.getFinalTransformation(); // get transformation in camera frame (because points are in camera frame)
		pcl::getTranslationAndEulerAngles(correctionCameraFrame, x, y, z, roll, pitch, yaw);
		Eigen::Affine3f correctionLidarFrame = pcl::getTransformation(z, x, y, yaw, roll, pitch);
		// transform from world origin to wrong pose
		Eigen::Affine3f tWrong = pclPointToAffine3fCameraToLidar(cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
		// transform from world origin to corrected pose
		Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
		pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
		gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
		gtsam::Pose3 poseTo = pclPointTogtsamPose3(cloudKeyPoses6D->points[RSclosestHistoryFrameID]);
		gtsam::Vector Vector6(6);

		std::lock_guard<std::mutex> lock(mtx);
		gtSAMgraph.add(BetweenFactor<Pose3>(latestFrameIDLoopCloure, RSclosestHistoryFrameID, poseFrom.between(poseTo), robustNoiseModel));
		isam->update(gtSAMgraph);
		isam->update();
		gtSAMgraph.resize(0);
	}
	//TODO:

	/**
         * update gtsam graph
         */
	if (cloudKeyPoses3D->points.empty())
	{
		gtSAMgraph.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]), Point3(transformTobeMapped[5], transformTobeMapped[3], transformTobeMapped[4])), priorNoise));
		initialEstimate.insert(0, Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]),
										Point3(transformTobeMapped[5], transformTobeMapped[3], transformTobeMapped[4])));
		for (int i = 0; i < 6; ++i)
			transformLast[i] = transformTobeMapped[i];
	}
	else
	{
		gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(transformLast[2], transformLast[0], transformLast[1]),
									  Point3(transformLast[5], transformLast[3], transformLast[4]));
		gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
									Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4]));
		gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->points.size() - 1, cloudKeyPoses3D->points.size(), poseFrom.between(poseTo), odometryNoise));
		initialEstimate.insert(cloudKeyPoses3D->points.size(), Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
																	 Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4])));
	}
	/**
         * update iSAM
         */
	isam->update(gtSAMgraph, initialEstimate);
	isam->update();

	gtSAMgraph.resize(0);
	initialEstimate.clear();

	return true;
#endif
	return false;
}

bool GlobalOptimize::update_optimized_edges(constraints &all_cons, bool fixed_reg_edge, float wrong_edge_ratio_thre,
											float wrong_edge_translation_thre, float wrong_edge_rotation_thre)
{
	int wrong_edge_count = 0;
	for (int i = 0; i < all_cons.size(); i++)
	{
		//check if the change is too much (Trans1_2_optimized vs. Trans1_2)
		Eigen::Matrix4d Trans1_2_optimized = all_cons[i].block1->pose_optimized.inverse() * all_cons[i].block2->pose_optimized;
		Eigen::Matrix4d delta_Trans1_2 = Trans1_2_optimized.inverse() * all_cons[i].Trans1_2;

		if (check_wrong_edge(delta_Trans1_2, wrong_edge_translation_thre, wrong_edge_rotation_thre))
		{
			wrong_edge_count++;
			if (all_cons[i].con_type == REGISTRATION)
				all_cons[i].con_type = NONE;
		}
	}
	if (1.0 * wrong_edge_count / all_cons.size() > wrong_edge_ratio_thre) //too many candidate wrong edges
		return false;
	else //udpate edges //TODO: we will not update edge now
	{
		for (int i = 0; i < all_cons.size(); i++)
		{
			if (fixed_reg_edge && all_cons[i].con_type == REGISTRATION)
				continue;

			all_cons[i].Trans1_2 = all_cons[i].block1->pose_optimized.inverse() * all_cons[i].block2->pose_optimized;
		}
	}
	return true;
}

bool GlobalOptimize::update_optimized_nodes(cloudblock_Ptrs &all_blocks, bool update_init, bool update_bbx)
{
	CloudUtility<Point_T> cu;

	for (int i = 0; i < all_blocks.size(); i++)
	{
		all_blocks[i]->pose_lo = all_blocks[i]->pose_optimized;
		if (update_init)
			all_blocks[i]->pose_init = all_blocks[i]->pose_lo;

		//update global bbx
		if (update_bbx)
		{
			all_blocks[i]->merge_feature_points(all_blocks[i]->pc_raw, false);
			pcl::transformPointCloud(*all_blocks[i]->pc_raw, *all_blocks[i]->pc_raw_w, all_blocks[i]->pose_lo);
			cu.get_cloud_bbx(all_blocks[i]->pc_raw_w, all_blocks[i]->bound);
			all_blocks[i]->free_raw_cloud();
		}
	}
}

bool GlobalOptimize::check_wrong_edge(Eigen::Matrix4d delta_tran_mat, float translation_thre, float rotation_thre_deg)
{
	Eigen::Vector3d ts(delta_tran_mat(0, 3), delta_tran_mat(1, 3), delta_tran_mat(2, 3));
	Eigen::AngleAxisd rs(delta_tran_mat.block<3, 3>(0, 0));

	if (ts.norm() > translation_thre || std::abs(rs.angle()) > rotation_thre_deg / 180.0 * M_PI)
		return true;
	else
		return false;
}

float GlobalOptimize::determine_weight(int node0_type, int node1_type, int edge_type)
{
	float weight_result = 1.0;
	if (node0_type == 1 && node1_type == 1 && edge_type == 2)
		weight_result = 0.1; //ALS+ALS registration
	if ((node0_type == 1 && node1_type == 3) || (node0_type == 3 && node1_type == 1))
		weight_result = 0.5; //ALS+MLS registration

	if (edge_type == 1)
		weight_result = 0.5; //ALS+ALS / MLS+MLS smooth

	return weight_result;
}

void GlobalOptimize::set_adjacent_edge_information_matrix(Matrix6d &vc_matrix)
{
	vc_matrix.setIdentity();

	vc_matrix(0, 0) = param_.tx_std * param_.tx_std;
	vc_matrix(1, 1) = param_.ty_std * param_.ty_std;
	vc_matrix(2, 2) = param_.tz_std * param_.tz_std;
	vc_matrix(3, 3) = (param_.roll_std * M_PI / 180.0) * (param_.roll_std * M_PI / 180.0);
	vc_matrix(4, 4) = (param_.pitch_std * M_PI / 180.0) * (param_.pitch_std * M_PI / 180.0);
	vc_matrix(5, 5) = (param_.yaw_std * M_PI / 180.0) * (param_.yaw_std * M_PI / 180.0);
}

bool GlobalOptimize::set_equal_weight(bool on_or_not)
{
	param_.use_equal_weight = on_or_not;
	return on_or_not;
}

#if 0
	//registration blunder elimination (By Zhao Xing)
	bool GlobalOptimize::cal_con_number(constraints &all_cons, strip &all_blocks)
	{
		LOG(INFO) << "Begin calculating reg_source_con_number!";
		//std::cout << "Begin calculating reg_source_con_number!\n";
		for (int i = 0; i < all_cons.size(); i++)
		{
			if ((all_cons[i].con_type == REGISTRATION) && (all_blocks[all_cons[i].block2->unique_id].cal_regsource_con_number == false))
			{
				int temp_uniqueID = all_cons[i].block2->unique_id;
				all_blocks[temp_uniqueID].reg_source_con_number = 0;
				for (int j = i; j < all_cons.size(); j++)
				{
					if ((all_cons[j].con_type == REGISTRATION) && (all_cons[j].block2->unique_id == temp_uniqueID))
					{
						all_blocks[temp_uniqueID].reg_source_con_number++;
						all_blocks[temp_uniqueID].as_source_con_unique_id.push_back(all_cons[j].unique_id);

						LOG(INFO) << "all_cons[j].unique_id: " << all_cons[j].unique_id << "all_blocks[temp_uniqueID].as_source_con_unique_id[reg_source_con_number-1]: " << all_blocks[temp_uniqueID].as_source_con_unique_id[all_blocks[temp_uniqueID].reg_source_con_number - 1] << endl;
					}
				}
				all_blocks[temp_uniqueID].cal_regsource_con_number = true;
			}
		}
		return 1;
	}

	bool GlobalOptimize::eliminate_blunders(constraints &all_cons, strip &all_blocks)
	{
		LOG(INFO) << "Begin eliminating errors!";
		cout << "Begin eliminating errors!\n";
		int elimination_count = 0; //count the number of the edges that would been eliminate
		//Each REGISTRATION con
		for (int i = 0; i < all_cons.size(); i++)
		{
			if (all_cons[i].con_type == REGISTRATION)
			{
				int temp_uniqueID = all_cons[i].block2->unique_id;

				if (all_blocks[temp_uniqueID].reg_source_con_number <= 1 && (all_blocks[temp_uniqueID].elimination_flag == false))
				{
					/*all_cons[i].con_type = NONE;
					all_cons[i].Trans1_2.setIdentity();
					all_cons[i].information_matrix.setIdentity();*/
					all_blocks[temp_uniqueID].elimination_flag = true;
					//elimination_count++;
					LOG(WARNING) << "The registration edge " << all_cons[i].unique_id << " has been deleted because the source block just has one registration!";
					//cout << "The registration edge " << all_cons[i].unique_id << " has been deleted because the source block just has one registration!" << endl;
				}
				if (all_blocks[temp_uniqueID].reg_source_con_number == 2 && (all_blocks[temp_uniqueID].elimination_flag == false))
				{
					int con_number = all_blocks[temp_uniqueID].reg_source_con_number;
					//construct adjacent matrix of registration
					std::vector<std::vector<float>> RegMatrix(con_number, std::vector<float>(con_number, 0.0));
					//sum each line of RegMatrix
					//std::vector<float> temp(con_number, 0.0);
					//find con_unique_id
					//vector<constraint_t, Eigen::aligned_allocator<constraint_t>>  tempcon;
					std::vector<int> index;
					for (int j = 0; j < con_number; j++)
					{
						for (int k = 0; k < all_cons.size(); k++)
						{
							if (all_cons[k].unique_id == all_blocks[temp_uniqueID].as_source_con_unique_id[j])
							{
								index.push_back(k);
								//tempcon.push_back(all_cons[k]);
								break;
							}
						}
					}
					LOG(INFO) << "The source block " << all_blocks[temp_uniqueID].unique_id << "'s temp:\n";
					for (int j = 0; j < con_number; j++)
					{
						for (int k = 0; k < con_number; k++)
						{
							LOG(INFO) << "RegMatrix[" << j << "][" << k << "]'sFnorm^2:"
									  << "( con" << all_cons[index[j]].unique_id << " with con" << all_cons[index[k]].unique_id << " )\n";
							RegMatrix[j][k] = cal_f_norm(all_cons[index[j]].Trans1_2, all_cons[index[k]].Trans1_2);
							LOG(INFO) << "RegMatrix[" << k << "][" << j << "]'sFnorm^2:"
									  << "( con" << all_cons[index[k]].unique_id << " with con" << all_cons[index[j]].unique_id << " )\n";
							RegMatrix[k][j] = cal_f_norm(all_cons[index[k]].Trans1_2, all_cons[index[j]].Trans1_2);
						}
					}

					cout << "all_blocks[temp_uniqueID].unique_id: " << all_blocks[temp_uniqueID].unique_id << "     temp_uniqueID: " << temp_uniqueID << endl;
					cout << "The source block " << all_blocks[temp_uniqueID].unique_id << "'s Fnorm:\n";

					for (int j = 0; j < con_number; j++)
					{
						for (int k = 0; k < con_number - 1; k++)
							cout << RegMatrix[j][k] << "\t";
						cout << RegMatrix[j][con_number - 1] << "\n";
					}

					if (RegMatrix[0][1] > 1.0) //1.0 is the threshold
					{
						for (int j = 0; j < con_number; j++)
						{
							all_cons[index[j]].con_type = NONE;
							all_cons[index[j]].Trans1_2.setIdentity();
							all_cons[index[j]].information_matrix.setIdentity();
							elimination_count++;
							LOG(INFO) << "all_cons[index[j]].unique_id: " << all_cons[index[j]].unique_id << "     all_blocks[temp_uniqueID].as_source_con_unique_id[j]: " << all_blocks[temp_uniqueID].as_source_con_unique_id[j] << endl;
							LOG(WARNING) << "The registration edge " << all_cons[index[j]].unique_id << " has been deleted because the transformation matrix of source block is inconsistent with another one!";
							LOG(INFO) << "The registration edge " << all_cons[index[j]].unique_id << " has been deleted because the transformation matrix of source block is inconsistent with another one!" << endl;
						}
					}

					all_blocks[temp_uniqueID].elimination_flag = true;
				}
				if ((all_blocks[temp_uniqueID].reg_source_con_number > 2) && (all_blocks[temp_uniqueID].elimination_flag == false))
				{
					int con_number = all_blocks[temp_uniqueID].reg_source_con_number;

					//construct adjacent matrix of registration
					std::vector<std::vector<float>> RegMatrix(con_number, std::vector<float>(con_number, 0.0));
					//sum each line of RegMatrix
					std::vector<float> temp(con_number, 0.0);

					//find con_unique_id
					//vector<constraint_t, Eigen::aligned_allocator<constraint_t>>  tempcon;
					std::vector<int> index;
					for (int j = 0; j < con_number; j++)
					{
						for (int k = 0; k < all_cons.size(); k++)
						{
							if (all_cons[k].unique_id == all_blocks[temp_uniqueID].as_source_con_unique_id[j])
							{
								index.push_back(k);
								//tempcon.push_back(all_cons[k]);
								break;
							}
						}
					}

					float sum = 0.0;
					float average = 0.0;
					float sigama = 0.0;

					/*	for (int j = 0; j < RegMatrix.size() - 1; j++)
					{
						for (int k = j + 1; k < RegMatrix.size(); k++)
						{
							RegMatrix[j][k] = calFnorm(all_cons[all_blocks[all_cons[i].block2->unique_id].as_source_con_unique_id[j]].Trans1_2, all_cons[all_blocks[all_cons[i].block2->unique_id].as_source_con_unique_id[k]].Trans1_2);
							RegMatrix[k][j] = RegMatrix[j][k];
							temp[j] += RegMatrix[j][k];
							temp[k] += RegMatrix[k][j];
							sum += 2*RegMatrix[j][k];
						}
						
					}*/

					LOG(INFO) << "The source block " << all_blocks[temp_uniqueID].unique_id << "'s temp:\n";

					for (int j = 0; j < con_number; j++)
					{
						for (int k = 0; k < con_number; k++)
						{
							LOG(INFO) << "RegMatrix[" << j << "][" << k << "]'sFnorm^2:"
									  << "( con" << all_cons[index[j]].unique_id << " with con" << all_cons[index[k]].unique_id << " )\n";
							RegMatrix[j][k] = cal_f_norm(all_cons[index[j]].Trans1_2, all_cons[index[k]].Trans1_2);
							LOG(INFO) << "RegMatrix[" << k << "][" << j << "]'sFnorm^2:"
									  << "( con" << all_cons[index[k]].unique_id << " with con" << all_cons[index[j]].unique_id << " )\n";
							RegMatrix[k][j] = cal_f_norm(all_cons[index[k]].Trans1_2, all_cons[index[j]].Trans1_2);
							temp[j] += RegMatrix[j][k];
							sum += RegMatrix[j][k];
						}
					}

					LOG(INFO) << "all_blocks[temp_uniqueID].unique_id: " << all_blocks[temp_uniqueID].unique_id << "     temp_uniqueID: " << temp_uniqueID << endl;
					LOG(INFO) << "The source block " << all_blocks[temp_uniqueID].unique_id << "'s Fnorm:\n";

					for (int j = 0; j < con_number; j++)
					{
						for (int k = 0; k < con_number - 1; k++)
							cout << RegMatrix[j][k] << "\t";
						cout << RegMatrix[j][con_number - 1] << "\n";
					}

					//find out error and eliminate
					average = 1.0 * sum / con_number;
					for (int j = 0; j < con_number; j++)
					{
						sigama += 1.0 * (temp[j] - average) * (temp[j] - average) / con_number;
					}
					sigama = sqrt(sigama);

					for (int j = 0; j < con_number; j++)
					{
						if (temp[j] - average > sigama) //is error
						{
							all_cons[index[j]].con_type = NONE;
							all_cons[index[j]].Trans1_2.setIdentity();
							all_cons[index[j]].information_matrix.setIdentity();
							elimination_count++;
							cout << "all_cons[index[j]].unique_id: " << all_cons[index[j]].unique_id << "     all_blocks[temp_uniqueID].as_source_con_unique_id[j]: " << all_blocks[temp_uniqueID].as_source_con_unique_id[j] << endl;
							LOG(WARNING) << "The registration edge " << all_cons[index[j]].unique_id << " has been deleted because the transformation matrix of source block is inconsistent with others!";
							cout << "The registration edge " << all_cons[index[j]].unique_id << " has been deleted because the transformation matrix of source block is inconsistent with others!" << endl;
						}
					}
					all_blocks[temp_uniqueID].elimination_flag = true;
				}
			}
		}

		LOG(INFO) << "!----------------------------------------------------------------------------!";
		LOG(INFO) << "Eliminate blunders done ";
		LOG(INFO) << "The number of eliminated registration constraints : " << elimination_count;
		LOG(INFO) << "!----------------------------------------------------------------------------!";

		return 1;
	}

	//Calculate Fnorm between Transformation martix1 and Transformtion martix2
	float GlobalOptimize::cal_f_norm(Eigen::Matrix4d &Trans1, Eigen::Matrix4d &Trans2)
	{
		float temp = 0.0;
		float Fnorm = 0.0;
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				if ((Trans1(i, j) - Trans2(i, j) < 0.001) && (Trans2(i, j) - Trans1(i, j) < 0.001))
				{
					temp += 0.0;
				}
				else
				{
					temp += (Trans1(i, j) - Trans2(i, j)) * (Trans1(i, j) - Trans2(i, j));
				}
			}
		}
		cout << "before:" << temp << "\t";
		if (temp < 0.000001)
		{
			temp = 0.0;
		}
		cout << "after:" << temp << endl;
		Fnorm = sqrt(temp);
		return Fnorm;
	}
#endif
} // namespace lo
