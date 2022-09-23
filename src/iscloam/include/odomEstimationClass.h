// Author of ISCLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#ifndef _ODOM_ESTIMATION_CLASS_H_
#define _ODOM_ESTIMATION_CLASS_H_

//std lib
#include <string>
#include <math.h>
#include <vector>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

//ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

//LOCAL LIB
#include "lidar.h"
#include "lidarOptimization.h"
#include <ros/ros.h>

class OdomEstimationClass 
{

    public:
    	OdomEstimationClass();
    	
		void init(lidar::Lidar lidar_param, double map_resolution);	
		void initMapWithPoints(const pcl::PointCloud<pcl::PointXYZL>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZL>::Ptr& surf_in);
		void updatePointsToMap(const pcl::PointCloud<pcl::PointXYZL>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZL>::Ptr& surf_in);
		void getMap(pcl::PointCloud<pcl::PointXYZL>::Ptr& laserCloudMap);

		Eigen::Isometry3d odom;
		pcl::PointCloud<pcl::PointXYZL>::Ptr laserCloudCornerMap;
		pcl::PointCloud<pcl::PointXYZL>::Ptr laserCloudSurfMap;
	private:
		//optimization variable
		double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
		Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
		Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);

		Eigen::Isometry3d last_odom;

		//kd-tree
		pcl::KdTreeFLANN<pcl::PointXYZL>::Ptr kdtreeEdgeMap;
		pcl::KdTreeFLANN<pcl::PointXYZL>::Ptr kdtreeSurfMap;

		//points downsampling before add to map
		pcl::VoxelGrid<pcl::PointXYZL> downSizeFilterEdge;
		pcl::VoxelGrid<pcl::PointXYZL> downSizeFilterSurf;

		//local map
		pcl::CropBox<pcl::PointXYZL> cropBoxFilter;

		//optimization count 
		int optimization_count;

		//function
		void addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZL>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZL>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);
		void addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZL>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZL>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);
		void addPointsToMap(const pcl::PointCloud<pcl::PointXYZL>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZL>::Ptr& downsampledSurfCloud);
		void pointAssociateToMap(pcl::PointXYZL const *const pi, pcl::PointXYZL *const po);
		void downSamplingToMap(const pcl::PointCloud<pcl::PointXYZL>::Ptr& edge_pc_in, pcl::PointCloud<pcl::PointXYZL>::Ptr& edge_pc_out, const pcl::PointCloud<pcl::PointXYZL>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZL>::Ptr& surf_pc_out);
};

#endif // _ODOM_ESTIMATION_CLASS_H_

