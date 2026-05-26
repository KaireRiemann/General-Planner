#pragma once
#include <iostream>

#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/warp_point_rigid_3d.h>

// #include <fast_gicp/gicp/fast_gicp.hpp>
// #include <fast_gicp/gicp/fast_gicp_st.hpp>
// #include <fast_gicp/gicp/fast_vgicp.hpp>

#include <Eigen/src/Core/Matrix.h>

using namespace pcl;
using namespace std;

using ICPFunPtr = Eigen::Matrix4f (*)(PointCloud<PointXYZI>::Ptr, PointCloud<PointXYZI>::Ptr, Eigen::Matrix4f&, \
                                      std::vector<double>&, pcl::Correspondences&, std::vector<double>);

namespace ICPAlgorithm {

// bool plane_fit(Eigen::Matrix<float, 3, Eigen::Dynamic> A, Eigen::Vector4f &plane_coeff);

void addNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
               pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals,
               int num);

// Eigen::Matrix4f GDPointToPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
//                                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
//                                Eigen::Matrix4f& guess,
//                                std::vector<double> &ret,
//                                pcl::Correspondences& cor,
//                                std::vector<double> params);

Eigen::Matrix4f PointToPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
                             Eigen::Matrix4f& guess,
                             std::vector<double> &ret,
                             pcl::Correspondences& cor,
                             std::vector<double> params);

Eigen::Matrix4f PointToPlaneWithNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
                             pcl::PointCloud<pcl::PointNormal>::Ptr cloud2,
                             Eigen::Matrix4f guess,
                             std::vector<double> &ret,
                             pcl::Correspondences& cor,
                             std::vector<double> params);

Eigen::Matrix4f PointToPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
                               Eigen::Matrix4f& guess,
                               std::vector<double> &ret,
                               pcl::Correspondences& cor,
                               std::vector<double> params);

Eigen::Matrix4f icpPlaneToPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
                               Eigen::Matrix4f& guess,
                               std::vector<double> &ret,
                               pcl::Correspondences& cor,
                               std::vector<double> params);


Eigen::Matrix4f icpPointToPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
                               Eigen::Matrix4f& guess,
                               std::vector<double> &ret,
                               pcl::Correspondences& cor,
                               std::vector<double> params);

Eigen::Matrix4f pclNDT(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
                       Eigen::Matrix4f& guess,
                       std::vector<double> &ret,
                       pcl::Correspondences& cor,
                       std::vector<double> params);

// Eigen::Matrix4f fastGICPST(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
//                        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
//                        Eigen::Matrix4f& guess,
//                        std::vector<double> &ret,
//                        pcl::Correspondences& cor,
//                        std::vector<double> params);

// Eigen::Matrix4f fastGICPMT(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
//                        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
//                        Eigen::Matrix4f& guess,
//                        std::vector<double> &ret,
//                        pcl::Correspondences& cor,
//                        std::vector<double> params);


std::map<std::string, ICPFunPtr> icp_map{{"icpPlaneToPlane", &icpPlaneToPlane},
                                         {"icpPointToPlane", &icpPointToPlane},
                                         {"PointToPlane", &PointToPlane},
                                         {"PointToPoint", &PointToPoint},
                                         {"pclNDT", &pclNDT}};

} // namespace ICPAlgorithm