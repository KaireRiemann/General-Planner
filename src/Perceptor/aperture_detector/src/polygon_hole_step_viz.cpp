/*
Developer: OpenAI Codex

Step-by-step visualization node for detecting a convex polygonal hole on a
frame plane using multi-frame LiDAR accumulation from rosbag.
*/

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <vector>
#include <deque>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <aperture_detector/ApertureObservation.h>
#include <std_msgs/String.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <XmlRpcValue.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include "data_preprocess.hpp"
#include <Eigen/Geometry>

struct StepConfig
{
  std::string frame_id = "map";
  double voxel_leaf = 0.01;
  double plane_distance_threshold = 0.015;
  double hole_depth_gap_threshold = 0.15;
  double cluster_tolerance = 0.06;
  int min_hole_cluster_size = 12;
  int max_hole_cluster_size = 200000;
  double pass_area_ratio = 1.0 / 3.0;
  double line_width = 0.02;
  double center_marker_size = 0.08;
  double roi_line_width = 0.015;
  double publish_rate = 1.0;
  bool dynamic_roi_enable = false;
  std::string dynamic_roi_pose_topic = "/roi_pose";
  std::string dynamic_roi_source = "pose";
  std::string dynamic_roi_odom_topic = "/laserMapping/odometry_body";
  double dynamic_roi_timeout = 0.5;
  bool dynamic_roi_use_pose_orientation = false;
  double odom_history_duration = 10.0;
  double odom_sync_max_dt = 0.3;
  bool visual_roi_enable = false;
  std::string visual_roi_topic = "/windowtec/window_box";
  double visual_roi_timeout = 5.0;
  double visual_roi_margin_px = 40.0;
  bool visual_roi_required = false;
  std::string visual_camera_info_topic = "/camera0/color/info";
  Eigen::Matrix3d visual_Rcw = Eigen::Matrix3d::Identity();  // camera <- world
  Eigen::Vector3d visual_Pcw = Eigen::Vector3d::Zero();      // camera <- world
  Eigen::Matrix3d visual_Rcb = Eigen::Matrix3d::Identity();  // camera <- body
  Eigen::Vector3d visual_t_bc = Eigen::Vector3d::Zero();     // camera origin in body
  double hole_grid_resolution = 0.05;
  int hole_occupancy_dilate_cells = 1;
  int hole_min_cluster_cells = 30;
  bool hole_reject_border_clusters = true;
  double hole_min_empty_area_ratio = 0.6;
  double hole_boundary_support_radius = 0.04;
  int hole_boundary_min_support = 3;
  double hole_boundary_sample_step = 0.02;
  double hole_boundary_max_unsupported_ratio = 0.25;
  bool publish_overlay_image = true;
  bool has_extrinsics = false;
  Eigen::Matrix3d Rcl = Eigen::Matrix3d::Identity();
  Eigen::Vector3d Pcl = Eigen::Vector3d::Zero();
  std::string image_topic = "/usb_cam/image_raw";
  bool point_cloud_in_imu_frame = true;
  bool has_lidar_imu_extrinsics = false;
  Eigen::Matrix3d Ril = Eigen::Matrix3d::Identity();  // IMU <- LiDAR
  Eigen::Vector3d Til = Eigen::Vector3d::Zero();      // IMU <- LiDAR
  Eigen::Matrix3d Rli = Eigen::Matrix3d::Identity();  // LiDAR <- IMU
  Eigen::Vector3d Tli = Eigen::Vector3d::Zero();      // LiDAR <- IMU
  bool realtime_mode = true;
  bool detection_initial_enabled = true;
  std::string detection_enable_topic = "detection_enable";
  int accumulate_frames = 90;
  int min_frames_to_process = 60;
  bool save_overlay_image = false;
  bool transform_input_to_target_frame = true;
  double tf_lookup_timeout = 0.05;
  bool projection_use_tf = true;
  std::string projection_camera_frame = "usb_cam";
  bool enable_plane_orientation_filter = false;
  Eigen::Vector3d plane_filter_up_axis = Eigen::Vector3d(0.0, 0.0, 1.0);
  double plane_filter_max_up_dot = 0.75;
  int plane_candidate_max_trials = 4;
  bool plane_polygon_use_concave_hull = false;
  double plane_polygon_concave_alpha = 0.08;
  bool plane_polygon_use_largest_cluster = true;
  double plane_polygon_cluster_tolerance = 0.06;
  int plane_polygon_cluster_min_size = 80;
  bool plane_polygon_tight_fit_enable = true;
  bool plane_polygon_auto_alpha_enable = true;
  double plane_polygon_alpha_min = 0.03;
  double plane_polygon_alpha_max = 0.14;
  int plane_polygon_alpha_steps = 6;
  double plane_polygon_min_coverage_ratio = 0.96;
  double plane_boundary_support_radius = 0.04;
  int plane_boundary_min_support = 4;
  double plane_boundary_sample_step = 0.02;
  double plane_boundary_max_unsupported_ratio = 0.20;
  bool plane_boundary_auto_shrink_enable = false;
  double plane_boundary_shrink_step = 0.03;
  int plane_boundary_shrink_max_iters = 6;
  double plane_boundary_shrink_min_area_ratio = 0.45;
  bool save_hole_polygon_txt = false;
  std::string hole_polygon_txt_path = "";
  bool publish_planning_snapshot_json = true;
  std::string planning_snapshot_json_topic = "planning_snapshot_json";
  int planning_snapshot_min_vertices = 4;
};

struct Polygon2D
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr hull{new pcl::PointCloud<pcl::PointXYZ>};
  pcl::PointXYZ centroid{};
  double area = 0.0;
  bool valid = false;
};

struct StepResult
{
  std::string summary = "not_started";
  bool pass = false;
  bool has_plane = false;
  bool has_hole = false;
  double plane_area = 0.0;
  double hole_area = 0.0;
  double area_ratio = 0.0;
  size_t lidar_frames = 0;
  size_t lidar_points_raw = 0;
  size_t lidar_points_accumulated = 0;
  size_t roi_points = 0;
  size_t plane_points = 0;
  size_t non_plane_points = 0;
  size_t far_candidate_points = 0;
  size_t hole_cluster_points = 0;
  bool visual_roi_used = false;
  size_t visual_roi_points = 0;
};

struct OdomState
{
  bool has_odom = false;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  geometry_msgs::Pose pose;
  ros::Time msg_stamp = ros::Time(0);
  ros::Time rcv_stamp = ros::Time(0);
};

struct DynamicRoiState
{
  bool has_pose = false;
  geometry_msgs::PoseStamped pose;
  ros::Time rcv_stamp = ros::Time(0);
};

struct VisualRoiState
{
  bool has_polygon = false;
  geometry_msgs::PolygonStamped polygon;
  ros::Time rcv_stamp = ros::Time(0);
};

struct CameraInfoState
{
  bool has_info = false;
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
};

struct ActiveRoi
{
  Params params;
  bool oriented = false;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
};

Params makeRoiParamsRelativeToCenter(const Params &base_params, const Eigen::Vector3d &center)
{
  Params roi_params = base_params;
  roi_params.x_min = center.x() + base_params.x_min;
  roi_params.x_max = center.x() + base_params.x_max;
  roi_params.y_min = center.y() + base_params.y_min;
  roi_params.y_max = center.y() + base_params.y_max;
  roi_params.z_min = center.z() + base_params.z_min;
  roi_params.z_max = center.z() + base_params.z_max;
  return roi_params;
}

ActiveRoi makeStaticRoi(const Params &params)
{
  ActiveRoi roi;
  roi.params = params;
  roi.center = Eigen::Vector3d(
    0.5 * (params.x_min + params.x_max),
    0.5 * (params.y_min + params.y_max),
    0.5 * (params.z_min + params.z_max));
  return roi;
}

ActiveRoi makeAxisAlignedDynamicRoi(const Params &base_params, const Eigen::Vector3d &center)
{
  ActiveRoi roi;
  roi.params = makeRoiParamsRelativeToCenter(base_params, center);
  roi.center = Eigen::Vector3d(
    0.5 * (roi.params.x_min + roi.params.x_max),
    0.5 * (roi.params.y_min + roi.params.y_max),
    0.5 * (roi.params.z_min + roi.params.z_max));
  return roi;
}

ActiveRoi makeOrientedDynamicRoi(const Params &base_params, const geometry_msgs::Pose &pose)
{
  ActiveRoi roi;
  roi.params = base_params;
  roi.oriented = true;
  roi.center = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);

  Eigen::Quaterniond q(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z);
  if (q.norm() < 1e-6)
  {
    q = Eigen::Quaterniond::Identity();
  }
  else
  {
    q.normalize();
  }
  roi.rotation = q.toRotationMatrix();
  return roi;
}

Eigen::Vector3d roiLocalBoxCenter(const Params &params)
{
  return Eigen::Vector3d(
    0.5 * (params.x_min + params.x_max),
    0.5 * (params.y_min + params.y_max),
    0.5 * (params.z_min + params.z_max));
}

pcl::PointXYZ roiTextAnchor(const ActiveRoi &roi)
{
  Eigen::Vector3d p;
  if (roi.oriented)
  {
    Eigen::Vector3d local = roiLocalBoxCenter(roi.params);
    local.z() = roi.params.z_max + 0.2;
    p = roi.center + roi.rotation * local;
  }
  else
  {
    p = Eigen::Vector3d(
      0.5 * (roi.params.x_min + roi.params.x_max),
      0.5 * (roi.params.y_min + roi.params.y_max),
      roi.params.z_max + 0.2);
  }
  return pcl::PointXYZ(p.x(), p.y(), p.z());
}

bool findNearestOdomByStamp(
  const std::deque<OdomState> &history,
  const ros::Time &target_stamp,
  double max_dt,
  OdomState &matched_odom,
  double &matched_dt)
{
  if (history.empty() || target_stamp.isZero()) return false;
  bool found = false;
  double best_dt = std::numeric_limits<double>::max();
  OdomState best;
  for (const auto &odom : history)
  {
    if (!odom.has_odom || odom.msg_stamp.isZero()) continue;
    const double dt = std::abs((odom.msg_stamp - target_stamp).toSec());
    if (dt < best_dt)
    {
      best_dt = dt;
      best = odom;
      found = true;
    }
  }
  if (!found) return false;
  if (max_dt > 0.0 && best_dt > max_dt) return false;
  matched_odom = best;
  matched_dt = best_dt;
  return true;
}

bool loadVectorParam(ros::NodeHandle &nh, const std::string &name, std::vector<double> &values, int expected_size)
{
  XmlRpc::XmlRpcValue xml_value;
  if (!nh.getParam(name, xml_value)) return false;
  if (xml_value.getType() != XmlRpc::XmlRpcValue::TypeArray) return false;
  if (xml_value.size() != expected_size) return false;

  values.resize(expected_size);
  for (int i = 0; i < expected_size; ++i)
  {
    if (xml_value[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
        xml_value[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      return false;
    }
    values[i] = xml_value[i].getType() == XmlRpc::XmlRpcValue::TypeInt
      ? static_cast<int>(xml_value[i])
      : static_cast<double>(xml_value[i]);
  }
  return true;
}

bool loadVectorParamAny(
  ros::NodeHandle &nh,
  const std::vector<std::string> &names,
  std::vector<double> &values,
  int expected_size)
{
  for (const auto &name : names)
  {
    if (loadVectorParam(nh, name, values, expected_size))
    {
      return true;
    }
  }
  return false;
}

StepConfig loadStepConfig(ros::NodeHandle &nh)
{
  StepConfig cfg;
  nh.param("frame_id", cfg.frame_id, std::string("map"));
  nh.param("voxel_leaf", cfg.voxel_leaf, 0.01);
  nh.param("plane_distance_threshold", cfg.plane_distance_threshold, 0.015);
  nh.param("hole_depth_gap_threshold", cfg.hole_depth_gap_threshold, 0.15);
  nh.param("cluster_tolerance", cfg.cluster_tolerance, 0.06);
  nh.param("min_hole_cluster_size", cfg.min_hole_cluster_size, 12);
  nh.param("max_hole_cluster_size", cfg.max_hole_cluster_size, 200000);
  nh.param("pass_area_ratio", cfg.pass_area_ratio, 1.0 / 3.0);
  nh.param("line_width", cfg.line_width, 0.02);
  nh.param("center_marker_size", cfg.center_marker_size, 0.08);
  nh.param("roi_line_width", cfg.roi_line_width, 0.015);
  nh.param("publish_rate", cfg.publish_rate, 1.0);
  nh.param("dynamic_roi_enable", cfg.dynamic_roi_enable, false);
  nh.param("dynamic_roi_pose_topic", cfg.dynamic_roi_pose_topic, std::string("/roi_pose"));
  nh.param("dynamic_roi_source", cfg.dynamic_roi_source, std::string("pose"));
  nh.param("dynamic_roi_odom_topic", cfg.dynamic_roi_odom_topic, std::string("/laserMapping/odometry_body"));
  nh.param("dynamic_roi_timeout", cfg.dynamic_roi_timeout, 0.5);
  nh.param("dynamic_roi_use_pose_orientation", cfg.dynamic_roi_use_pose_orientation, false);
  nh.param("odom_history_duration", cfg.odom_history_duration, 10.0);
  nh.param("odom_sync_max_dt", cfg.odom_sync_max_dt, 0.3);
  nh.param("visual_roi_enable", cfg.visual_roi_enable, false);
  nh.param("visual_roi_topic", cfg.visual_roi_topic, std::string("/windowtec/window_box"));
  nh.param("visual_roi_timeout", cfg.visual_roi_timeout, 5.0);
  nh.param("visual_roi_margin_px", cfg.visual_roi_margin_px, 40.0);
  nh.param("visual_roi_required", cfg.visual_roi_required, false);
  nh.param("visual_camera_info_topic", cfg.visual_camera_info_topic, std::string("/camera0/color/info"));
  nh.param("hole_grid_resolution", cfg.hole_grid_resolution, 0.05);
  nh.param("hole_occupancy_dilate_cells", cfg.hole_occupancy_dilate_cells, 1);
  nh.param("hole_min_cluster_cells", cfg.hole_min_cluster_cells, 30);
  nh.param("hole_reject_border_clusters", cfg.hole_reject_border_clusters, true);
  nh.param("hole_min_empty_area_ratio", cfg.hole_min_empty_area_ratio, 0.6);
  nh.param("hole_boundary_support_radius", cfg.hole_boundary_support_radius, 0.04);
  nh.param("hole_boundary_min_support", cfg.hole_boundary_min_support, 3);
  nh.param("hole_boundary_sample_step", cfg.hole_boundary_sample_step, 0.02);
  nh.param("hole_boundary_max_unsupported_ratio", cfg.hole_boundary_max_unsupported_ratio, 0.25);
  nh.param("publish_overlay_image", cfg.publish_overlay_image, true);
  nh.param("image_topic", cfg.image_topic, std::string("/usb_cam/image_raw"));
  nh.param("point_cloud_in_imu_frame", cfg.point_cloud_in_imu_frame, true);
  nh.param("realtime_mode", cfg.realtime_mode, true);
  nh.param("detection_initial_enabled", cfg.detection_initial_enabled, true);
  nh.param("detection_enable_topic", cfg.detection_enable_topic, std::string("detection_enable"));
  nh.param("accumulate_frames", cfg.accumulate_frames, 90);
  nh.param("min_frames_to_process", cfg.min_frames_to_process, 60);
  nh.param("save_overlay_image", cfg.save_overlay_image, false);
  nh.param("transform_input_to_target_frame", cfg.transform_input_to_target_frame, true);
  nh.param("tf_lookup_timeout", cfg.tf_lookup_timeout, 0.05);
  nh.param("projection_use_tf", cfg.projection_use_tf, true);
  nh.param("projection_camera_frame", cfg.projection_camera_frame, std::string("usb_cam"));
  nh.param("enable_plane_orientation_filter", cfg.enable_plane_orientation_filter, false);
  nh.param("plane_filter_max_up_dot", cfg.plane_filter_max_up_dot, 0.75);
  nh.param("plane_candidate_max_trials", cfg.plane_candidate_max_trials, 4);
  nh.param("plane_polygon_use_concave_hull", cfg.plane_polygon_use_concave_hull, false);
  nh.param("plane_polygon_concave_alpha", cfg.plane_polygon_concave_alpha, 0.08);
  nh.param("plane_polygon_use_largest_cluster", cfg.plane_polygon_use_largest_cluster, true);
  nh.param("plane_polygon_cluster_tolerance", cfg.plane_polygon_cluster_tolerance, 0.06);
  nh.param("plane_polygon_cluster_min_size", cfg.plane_polygon_cluster_min_size, 80);
  nh.param("plane_polygon_tight_fit_enable", cfg.plane_polygon_tight_fit_enable, true);
  nh.param("plane_polygon_auto_alpha_enable", cfg.plane_polygon_auto_alpha_enable, true);
  nh.param("plane_polygon_alpha_min", cfg.plane_polygon_alpha_min, 0.03);
  nh.param("plane_polygon_alpha_max", cfg.plane_polygon_alpha_max, 0.14);
  nh.param("plane_polygon_alpha_steps", cfg.plane_polygon_alpha_steps, 6);
  nh.param("plane_polygon_min_coverage_ratio", cfg.plane_polygon_min_coverage_ratio, 0.96);
  nh.param("plane_boundary_support_radius", cfg.plane_boundary_support_radius, 0.04);
  nh.param("plane_boundary_min_support", cfg.plane_boundary_min_support, 4);
  nh.param("plane_boundary_sample_step", cfg.plane_boundary_sample_step, 0.02);
  nh.param("plane_boundary_max_unsupported_ratio", cfg.plane_boundary_max_unsupported_ratio, 0.20);
  nh.param("plane_boundary_auto_shrink_enable", cfg.plane_boundary_auto_shrink_enable, true);
  nh.param("plane_boundary_shrink_step", cfg.plane_boundary_shrink_step, 0.03);
  nh.param("plane_boundary_shrink_max_iters", cfg.plane_boundary_shrink_max_iters, 6);
  nh.param("plane_boundary_shrink_min_area_ratio", cfg.plane_boundary_shrink_min_area_ratio, 0.45);
  nh.param("save_hole_polygon_txt", cfg.save_hole_polygon_txt, false);
  nh.param("hole_polygon_txt_path", cfg.hole_polygon_txt_path, std::string(""));
  nh.param("publish_planning_snapshot_json", cfg.publish_planning_snapshot_json, true);
  nh.param("planning_snapshot_json_topic", cfg.planning_snapshot_json_topic, std::string("planning_snapshot_json"));
  nh.param("planning_snapshot_min_vertices", cfg.planning_snapshot_min_vertices, 4);
  cfg.planning_snapshot_min_vertices = std::max(3, cfg.planning_snapshot_min_vertices);

  std::vector<double> up_axis_values;
  if (loadVectorParamAny(nh, {"plane_filter_up_axis"}, up_axis_values, 3))
  {
    const Eigen::Vector3d axis(up_axis_values[0], up_axis_values[1], up_axis_values[2]);
    if (axis.norm() > 1e-6)
    {
      cfg.plane_filter_up_axis = axis.normalized();
    }
  }

  std::vector<double> rcl_values;
  std::vector<double> pcl_values;
  if (loadVectorParamAny(nh, {"Rcl"}, rcl_values, 9) && loadVectorParamAny(nh, {"Pcl"}, pcl_values, 3))
  {
    for (int r = 0; r < 3; ++r)
    {
      for (int c = 0; c < 3; ++c)
      {
        cfg.Rcl(r, c) = rcl_values[r * 3 + c];
      }
    }
    cfg.Pcl = Eigen::Vector3d(pcl_values[0], pcl_values[1], pcl_values[2]);
    cfg.has_extrinsics = true;
  }

  std::vector<double> visual_r_values;
  if (loadVectorParamAny(nh, {"visual_Rcw", "visual_camera_Rcw"}, visual_r_values, 9))
  {
    for (int r = 0; r < 3; ++r)
    {
      for (int c = 0; c < 3; ++c)
      {
        cfg.visual_Rcw(r, c) = visual_r_values[r * 3 + c];
      }
    }
  }
  std::vector<double> visual_p_values;
  if (loadVectorParamAny(nh, {"visual_Pcw", "visual_camera_Pcw"}, visual_p_values, 3))
  {
    cfg.visual_Pcw = Eigen::Vector3d(visual_p_values[0], visual_p_values[1], visual_p_values[2]);
  }
  std::vector<double> visual_rcb_values;
  if (loadVectorParamAny(nh, {"visual_Rcb", "visual_camera_Rcb"}, visual_rcb_values, 9))
  {
    for (int r = 0; r < 3; ++r)
    {
      for (int c = 0; c < 3; ++c)
      {
        cfg.visual_Rcb(r, c) = visual_rcb_values[r * 3 + c];
      }
    }
  }
  std::vector<double> visual_tbc_values;
  if (loadVectorParamAny(nh, {"visual_t_bc", "visual_camera_t_bc"}, visual_tbc_values, 3))
  {
    cfg.visual_t_bc = Eigen::Vector3d(visual_tbc_values[0], visual_tbc_values[1], visual_tbc_values[2]);
  }

  std::vector<double> ril_values;
  std::vector<double> til_values;
  if (loadVectorParamAny(nh, {"extrinsic_R", "mapping/extrinsic_R"}, ril_values, 9))
  {
    for (int r = 0; r < 3; ++r)
    {
      for (int c = 0; c < 3; ++c)
      {
        cfg.Ril(r, c) = ril_values[r * 3 + c];
      }
    }
    cfg.has_lidar_imu_extrinsics = true;
  }

  if (loadVectorParamAny(nh, {"extrinsic_T", "mapping/extrinsic_T"}, til_values, 3))
  {
    cfg.Til = Eigen::Vector3d(til_values[0], til_values[1], til_values[2]);
    cfg.has_lidar_imu_extrinsics = true;
  }
  else
  {
    double imu_tx = 0.0;
    double imu_ty = 0.0;
    double imu_tz = 0.0;
    const bool has_imu_xyz =
      nh.getParam("imu_tx", imu_tx) ||
      nh.getParam("imu_ty", imu_ty) ||
      nh.getParam("imu_tz", imu_tz);
    if (has_imu_xyz)
    {
      cfg.Til = Eigen::Vector3d(imu_tx, imu_ty, imu_tz);
      cfg.has_lidar_imu_extrinsics = true;
    }
  }

  if (cfg.has_lidar_imu_extrinsics)
  {
    cfg.Rli = cfg.Ril.transpose();
    cfg.Tli = -cfg.Rli * cfg.Til;
  }
  if (cfg.hole_polygon_txt_path.empty())
  {
    cfg.hole_polygon_txt_path = "/tmp/hole_polygon_latest.txt";
  }
  return cfg;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr toXYZCloud(const pcl::PointCloud<Common::Point>::Ptr &input)
{
  auto output = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  output->reserve(input->size());
  for (const auto &pt : *input)
  {
    output->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
  }
  return output;
}

double polygonArea2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr &polygon)
{
  if (!polygon || polygon->size() < 3) return 0.0;
  double area = 0.0;
  for (size_t i = 0; i < polygon->size(); ++i)
  {
    const auto &a = polygon->points[i];
    const auto &b = polygon->points[(i + 1) % polygon->size()];
    area += static_cast<double>(a.x) * b.y - static_cast<double>(b.x) * a.y;
  }
  return std::abs(area) * 0.5;
}

pcl::PointXYZ polygonCentroid2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr &polygon)
{
  pcl::PointXYZ center;
  center.x = center.y = center.z = 0.0f;
  if (!polygon || polygon->empty()) return center;

  const double area = polygonArea2D(polygon);
  if (area < 1e-8)
  {
    for (const auto &pt : polygon->points)
    {
      center.x += pt.x;
      center.y += pt.y;
    }
    center.x /= static_cast<float>(polygon->size());
    center.y /= static_cast<float>(polygon->size());
    return center;
  }

  double cx = 0.0;
  double cy = 0.0;
  double cross_sum = 0.0;
  for (size_t i = 0; i < polygon->size(); ++i)
  {
    const auto &a = polygon->points[i];
    const auto &b = polygon->points[(i + 1) % polygon->size()];
    const double cross = static_cast<double>(a.x) * b.y - static_cast<double>(b.x) * a.y;
    cross_sum += cross;
    cx += (a.x + b.x) * cross;
    cy += (a.y + b.y) * cross;
  }

  center.x = static_cast<float>(cx / (3.0 * cross_sum));
  center.y = static_cast<float>(cy / (3.0 * cross_sum));
  center.z = 0.0f;
  return center;
}

Polygon2D makeConvexPolygon2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_2d)
{
  Polygon2D poly;
  if (!cloud_2d || cloud_2d->size() < 3) return poly;

  pcl::ConvexHull<pcl::PointXYZ> hull;
  hull.setInputCloud(cloud_2d);
  hull.setDimension(2);
  hull.reconstruct(*poly.hull);

  if (poly.hull->size() < 3) return poly;
  poly.area = polygonArea2D(poly.hull);
  if (poly.area <= 1e-6) return poly;
  poly.centroid = polygonCentroid2D(poly.hull);
  poly.valid = true;
  return poly;
}

Polygon2D makeConcavePolygon2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_2d, double alpha)
{
  Polygon2D poly;
  if (!cloud_2d || cloud_2d->size() < 3) return poly;

  pcl::ConcaveHull<pcl::PointXYZ> hull;
  hull.setInputCloud(cloud_2d);
  hull.setDimension(2);
  hull.setAlpha(std::max(1e-3, alpha));
  hull.reconstruct(*poly.hull);

  if (poly.hull->size() < 3) return poly;
  poly.area = polygonArea2D(poly.hull);
  if (poly.area <= 1e-6) return poly;
  poly.centroid = polygonCentroid2D(poly.hull);
  poly.valid = true;
  return poly;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr keepLargestPlaneCluster2D(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_2d,
  const StepConfig &cfg,
  std::string &reason)
{
  auto output = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  if (!cloud_2d || cloud_2d->empty())
  {
    reason = "empty";
    return output;
  }
  if (!cfg.plane_polygon_use_largest_cluster)
  {
    *output = *cloud_2d;
    reason = "disabled";
    return output;
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_2d);

  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(std::max(1e-3, cfg.plane_polygon_cluster_tolerance));
  ec.setMinClusterSize(std::max(10, cfg.plane_polygon_cluster_min_size));
  ec.setMaxClusterSize(std::max(100, static_cast<int>(cloud_2d->size())));
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_2d);
  ec.extract(clusters);

  if (clusters.empty())
  {
    *output = *cloud_2d;
    reason = "no clusters, fallback all";
    return output;
  }

  const auto best_it = std::max_element(
    clusters.begin(), clusters.end(),
    [](const pcl::PointIndices &a, const pcl::PointIndices &b)
    {
      return a.indices.size() < b.indices.size();
    });
  output->reserve(best_it->indices.size());
  for (int idx : best_it->indices)
  {
    output->push_back(cloud_2d->points[static_cast<size_t>(idx)]);
  }

  std::ostringstream oss;
  oss << "largest cluster kept " << output->size() << "/" << cloud_2d->size();
  reason = oss.str();
  return output;
}

double boundaryUnsupportedRatio(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &polygon,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &support_cloud,
  double sample_step,
  double radius,
  int min_support)
{
  if (!polygon || polygon->size() < 3 || !support_cloud || support_cloud->empty())
  {
    return 1.0;
  }

  pcl::search::KdTree<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(support_cloud);

  const double step = std::max(1e-3, sample_step);
  const double r = std::max(1e-3, radius);
  const int k_min = std::max(1, min_support);
  int unsupported = 0;
  int total = 0;
  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;

  for (size_t i = 0; i < polygon->size(); ++i)
  {
    const auto &a = polygon->points[i];
    const auto &b = polygon->points[(i + 1) % polygon->size()];
    const Eigen::Vector2d av(a.x, a.y);
    const Eigen::Vector2d bv(b.x, b.y);
    const double len = (bv - av).norm();
    const int sample_n = std::max(1, static_cast<int>(std::ceil(len / step)));
    for (int s = 0; s <= sample_n; ++s)
    {
      const double t = static_cast<double>(s) / static_cast<double>(sample_n);
      const Eigen::Vector2d p = av + t * (bv - av);
      pcl::PointXYZ q;
      q.x = static_cast<float>(p.x());
      q.y = static_cast<float>(p.y());
      q.z = 0.0f;
      const int k = kdtree.radiusSearch(q, r, k_indices, k_sqr_distances);
      ++total;
      if (k < k_min) ++unsupported;
    }
  }

  if (total <= 0) return 1.0;
  return static_cast<double>(unsupported) / static_cast<double>(total);
}

Polygon2D shrinkPolygonTowardCentroid(
  const Polygon2D &input,
  double shrink_step)
{
  Polygon2D output;
  if (!input.valid || !input.hull || input.hull->size() < 3)
  {
    return output;
  }

  const Eigen::Vector2d center(input.centroid.x, input.centroid.y);
  double mean_radius = 0.0;
  for (const auto &pt : input.hull->points)
  {
    mean_radius += (Eigen::Vector2d(pt.x, pt.y) - center).norm();
  }
  mean_radius /= std::max<size_t>(1, input.hull->size());
  if (mean_radius <= 1e-6)
  {
    return output;
  }

  const double step = std::max(1e-4, shrink_step);
  if (mean_radius <= step)
  {
    return output;
  }

  const double scale = (mean_radius - step) / mean_radius;
  output.hull->reserve(input.hull->size());
  for (const auto &pt : input.hull->points)
  {
    const Eigen::Vector2d v(pt.x, pt.y);
    const Eigen::Vector2d vs = center + scale * (v - center);
    output.hull->push_back(pcl::PointXYZ(vs.x(), vs.y(), 0.0f));
  }

  if (output.hull->size() < 3) return output;
  output.area = polygonArea2D(output.hull);
  if (output.area <= 1e-6) return output;
  output.centroid = polygonCentroid2D(output.hull);
  output.valid = true;
  return output;
}

Polygon2D makePlanePolygon2D(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &aligned_plane,
  const StepConfig &cfg,
  std::string &reason)
{
  Polygon2D poly;
  if (!aligned_plane || aligned_plane->size() < 3)
  {
    reason = "plane cloud too small";
    return poly;
  }

  std::string cluster_reason;
  auto plane_for_poly = keepLargestPlaneCluster2D(aligned_plane, cfg, cluster_reason);
  if (!plane_for_poly || plane_for_poly->size() < 3)
  {
    reason = "plane cluster too small";
    return poly;
  }

  // 回退到原始凸包边界：在过滤离散点后的平面点上构建凸包。
  poly = makeConvexPolygon2D(plane_for_poly);
  if (!poly.valid)
  {
    reason = "plane convex hull invalid";
    return poly;
  }

  const double unsupported_ratio = boundaryUnsupportedRatio(
    poly.hull,
    plane_for_poly,
    cfg.plane_boundary_sample_step,
    cfg.plane_boundary_support_radius,
    cfg.plane_boundary_min_support);
  if (unsupported_ratio <= cfg.plane_boundary_max_unsupported_ratio)
  {
    std::ostringstream oss;
    oss << "ok: convex hull | " << cluster_reason
        << " unsupported=" << unsupported_ratio;
    reason = oss.str();
    return poly;
  }

  if (!cfg.plane_boundary_auto_shrink_enable)
  {
    std::ostringstream oss;
    oss << "plane boundary sparse unsupported_ratio=" << unsupported_ratio
        << " max=" << cfg.plane_boundary_max_unsupported_ratio
        << " | " << cluster_reason;
    reason = oss.str();
    poly.valid = false;
    return poly;
  }

  const double original_area = std::max(1e-9, poly.area);
  const double min_area = original_area * std::max(0.05, cfg.plane_boundary_shrink_min_area_ratio);
  const int max_iters = std::max(1, cfg.plane_boundary_shrink_max_iters);
  Polygon2D shrunk = poly;
  double last_ratio = unsupported_ratio;

  for (int iter = 1; iter <= max_iters; ++iter)
  {
    shrunk = shrinkPolygonTowardCentroid(shrunk, cfg.plane_boundary_shrink_step);
    if (!shrunk.valid || shrunk.area < min_area)
    {
      break;
    }

    last_ratio = boundaryUnsupportedRatio(
      shrunk.hull,
      plane_for_poly,
      cfg.plane_boundary_sample_step,
      cfg.plane_boundary_support_radius,
      cfg.plane_boundary_min_support);
    if (last_ratio <= cfg.plane_boundary_max_unsupported_ratio)
    {
      std::ostringstream oss;
      oss << "ok: convex hull shrink accepted iter=" << iter
          << " unsupported_ratio=" << last_ratio
          << " max=" << cfg.plane_boundary_max_unsupported_ratio
          << " area_ratio=" << (shrunk.area / original_area)
          << " | " << cluster_reason;
      reason = oss.str();
      return shrunk;
    }
  }

  std::ostringstream oss;
  oss << "plane boundary sparse after shrink unsupported_ratio=" << last_ratio
      << " max=" << cfg.plane_boundary_max_unsupported_ratio
      << " | " << cluster_reason;
  reason = oss.str();
  poly.valid = false;
  return poly;
}

bool isInsidePolygon2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr &polygon, const pcl::PointXYZ &pt)
{
  if (!polygon || polygon->size() < 3) return false;
  bool inside = false;
  const double x = pt.x;
  const double y = pt.y;
  for (size_t i = 0, j = polygon->size() - 1; i < polygon->size(); j = i++)
  {
    const auto &pi = polygon->points[i];
    const auto &pj = polygon->points[j];
    const bool intersect = ((pi.y > y) != (pj.y > y)) &&
      (x < (static_cast<double>(pj.x - pi.x) * (y - pi.y) / (pj.y - pi.y + 1e-12) + pi.x));
    if (intersect) inside = !inside;
  }
  return inside;
}

Eigen::Matrix3d computeAlignmentRotation(const Eigen::Vector3d &normal)
{
  Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
  Eigen::Vector3d n = normal.normalized();
  double dot = std::max(-1.0, std::min(1.0, n.dot(z_axis)));
  if (std::abs(dot - 1.0) < 1e-8) return Eigen::Matrix3d::Identity();
  if (std::abs(dot + 1.0) < 1e-8)
  {
    return Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  Eigen::Vector3d axis = n.cross(z_axis).normalized();
  double angle = std::acos(dot);
  return Eigen::AngleAxisd(angle, axis).toRotationMatrix();
}

pcl::PointXYZ toOriginalPoint(const pcl::PointXYZ &aligned_pt, const Eigen::Matrix3d &R_inv, double average_z)
{
  const Eigen::Vector3d lifted(aligned_pt.x, aligned_pt.y, aligned_pt.z + average_z);
  const Eigen::Vector3d original = R_inv * lifted;
  return pcl::PointXYZ(original.x(), original.y(), original.z());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr toOriginalCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &aligned_cloud,
  const Eigen::Matrix3d &R_inv,
  double average_z,
  bool use_input_z)
{
  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->reserve(aligned_cloud->size());
  for (const auto &p : aligned_cloud->points)
  {
    pcl::PointXYZ pt = p;
    if (!use_input_z) pt.z = 0.0f;
    cloud->push_back(toOriginalPoint(pt, R_inv, average_z));
  }
  return cloud;
}

struct EmptyClusterCandidate
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cells{new pcl::PointCloud<pcl::PointXYZ>};
  bool touches_border = false;
  double empty_area = 0.0;
};

bool buildEmptyCandidatesOnPlane(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_poly,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &aligned_plane,
  const StepConfig &cfg,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &all_empty_cells,
  std::vector<EmptyClusterCandidate> &clusters,
  std::string &reason)
{
  if (!plane_poly || plane_poly->size() < 3 || !aligned_plane || aligned_plane->empty())
  {
    reason = "invalid aligned plane data";
    return false;
  }

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = -std::numeric_limits<double>::max();
  double max_y = -std::numeric_limits<double>::max();
  for (const auto &p : plane_poly->points)
  {
    min_x = std::min(min_x, static_cast<double>(p.x));
    min_y = std::min(min_y, static_cast<double>(p.y));
    max_x = std::max(max_x, static_cast<double>(p.x));
    max_y = std::max(max_y, static_cast<double>(p.y));
  }

  const double res = std::max(0.01, cfg.hole_grid_resolution);
  const int nx = std::max(1, static_cast<int>(std::ceil((max_x - min_x) / res)));
  const int ny = std::max(1, static_cast<int>(std::ceil((max_y - min_y) / res)));
  const size_t grid_size = static_cast<size_t>(nx) * static_cast<size_t>(ny);
  if (grid_size > 5000000)
  {
    reason = "grid too large, increase hole_grid_resolution";
    return false;
  }

  auto indexOf = [nx](int ix, int iy) -> size_t {
    return static_cast<size_t>(iy) * static_cast<size_t>(nx) + static_cast<size_t>(ix);
  };

  std::vector<uint8_t> inside(grid_size, 0);
  std::vector<uint8_t> occupied(grid_size, 0);
  std::vector<uint8_t> occupied_dilate(grid_size, 0);
  std::vector<uint8_t> empty_mask(grid_size, 0);
  std::vector<uint8_t> visited(grid_size, 0);

  for (int iy = 0; iy < ny; ++iy)
  {
    for (int ix = 0; ix < nx; ++ix)
    {
      const double cx = min_x + (static_cast<double>(ix) + 0.5) * res;
      const double cy = min_y + (static_cast<double>(iy) + 0.5) * res;
      if (isInsidePolygon2D(plane_poly, pcl::PointXYZ(cx, cy, 0.0f)))
      {
        inside[indexOf(ix, iy)] = 1;
      }
    }
  }

  for (const auto &p : aligned_plane->points)
  {
    const int ix = static_cast<int>(std::floor((p.x - min_x) / res));
    const int iy = static_cast<int>(std::floor((p.y - min_y) / res));
    if (ix < 0 || ix >= nx || iy < 0 || iy >= ny) continue;
    const size_t idx = indexOf(ix, iy);
    if (!inside[idx]) continue;
    occupied[idx] = 1;
  }

  const int r = std::max(0, cfg.hole_occupancy_dilate_cells);
  occupied_dilate = occupied;
  if (r > 0)
  {
    for (int iy = 0; iy < ny; ++iy)
    {
      for (int ix = 0; ix < nx; ++ix)
      {
        if (!occupied[indexOf(ix, iy)]) continue;
        for (int dy = -r; dy <= r; ++dy)
        {
          const int yy = iy + dy;
          if (yy < 0 || yy >= ny) continue;
          for (int dx = -r; dx <= r; ++dx)
          {
            const int xx = ix + dx;
            if (xx < 0 || xx >= nx) continue;
            occupied_dilate[indexOf(xx, yy)] = 1;
          }
        }
      }
    }
  }

  all_empty_cells->clear();
  for (int iy = 0; iy < ny; ++iy)
  {
    for (int ix = 0; ix < nx; ++ix)
    {
      const size_t idx = indexOf(ix, iy);
      if (!inside[idx] || occupied_dilate[idx]) continue;
      empty_mask[idx] = 1;
      const double cx = min_x + (static_cast<double>(ix) + 0.5) * res;
      const double cy = min_y + (static_cast<double>(iy) + 0.5) * res;
      all_empty_cells->push_back(pcl::PointXYZ(cx, cy, 0.0f));
    }
  }

  const int min_cells = std::max(4, cfg.hole_min_cluster_cells);
  for (int iy = 0; iy < ny; ++iy)
  {
    for (int ix = 0; ix < nx; ++ix)
    {
      const size_t seed = indexOf(ix, iy);
      if (!empty_mask[seed] || visited[seed]) continue;

      std::queue<std::pair<int, int>> q;
      q.push({ix, iy});
      visited[seed] = 1;
      EmptyClusterCandidate candidate;

      while (!q.empty())
      {
        const int cx = q.front().first;
        const int cy = q.front().second;
        q.pop();
        if (cx == 0 || cy == 0 || cx == nx - 1 || cy == ny - 1)
        {
          candidate.touches_border = true;
        }
        const double px = min_x + (static_cast<double>(cx) + 0.5) * res;
        const double py = min_y + (static_cast<double>(cy) + 0.5) * res;
        candidate.cells->push_back(pcl::PointXYZ(px, py, 0.0f));

        for (int dy = -1; dy <= 1; ++dy)
        {
          for (int dx = -1; dx <= 1; ++dx)
          {
            if (dx == 0 && dy == 0) continue;
            const int nx_i = cx + dx;
            const int ny_i = cy + dy;
            if (nx_i < 0 || nx_i >= nx || ny_i < 0 || ny_i >= ny) continue;
            const size_t nidx = indexOf(nx_i, ny_i);
            if (!empty_mask[nidx] || visited[nidx]) continue;
            visited[nidx] = 1;
            q.push({nx_i, ny_i});
          }
        }
      }

      candidate.empty_area =
        static_cast<double>(candidate.cells->size()) * res * res;
      if (static_cast<int>(candidate.cells->size()) < min_cells) continue;
      if (cfg.hole_reject_border_clusters && candidate.touches_border) continue;
      clusters.push_back(candidate);
    }
  }

  reason = "ok";
  return true;
}

cv::Mat makeCameraMatrix(const Params &params)
{
  return (cv::Mat_<double>(3, 3) <<
    params.fx, 0.0, params.cx,
    0.0, params.fy, params.cy,
    0.0, 0.0, 1.0);
}

cv::Mat makeDistCoeffs(const Params &params)
{
  return (cv::Mat_<double>(1, 5) << params.k1, params.k2, params.p1, params.p2, 0.0);
}

cv::Mat ensureBgrImage(const cv::Mat &input)
{
  if (input.empty()) return input;
  if (input.channels() == 3) return input.clone();

  cv::Mat output;
  if (input.channels() == 1)
  {
    cv::cvtColor(input, output, cv::COLOR_GRAY2BGR);
  }
  else if (input.channels() == 4)
  {
    cv::cvtColor(input, output, cv::COLOR_BGRA2BGR);
  }
  else
  {
    output = input.clone();
  }
  return output;
}

bool loadFirstImageFromBag(const std::string &bag_path, const std::string &image_topic, cv::Mat &image)
{
  std::fstream file_check(bag_path, std::ios::in);
  if (!file_check) return false;

  rosbag::Bag bag;
  try
  {
    bag.open(bag_path, rosbag::bagmode::Read);
  }
  catch (const rosbag::BagException &)
  {
    return false;
  }

  rosbag::View view(bag, rosbag::TopicQuery(std::vector<std::string>{image_topic}));
  for (const rosbag::MessageInstance &m : view)
  {
    auto img_msg = m.instantiate<sensor_msgs::Image>();
    if (!img_msg) continue;
    try
    {
      image = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
      bag.close();
      return !image.empty();
    }
    catch (const cv_bridge::Exception &)
    {
      continue;
    }
  }

  bag.close();
  return false;
}

bool projectLidarPointToImage(
  const pcl::PointXYZ &point_lidar,
  const StepConfig &cfg,
  const cv::Mat &camera_matrix,
  const cv::Mat &dist_coeffs,
  cv::Point2f &uv)
{
  if (!cfg.has_extrinsics) return false;

  Eigen::Vector3d point_l(point_lidar.x, point_lidar.y, point_lidar.z);
  if (cfg.point_cloud_in_imu_frame && cfg.has_lidar_imu_extrinsics)
  {
    const Eigen::Vector3d point_i(point_lidar.x, point_lidar.y, point_lidar.z);
    point_l = cfg.Rli * point_i + cfg.Tli;
  }

  const Eigen::Vector3d point_c = cfg.Rcl * point_l + cfg.Pcl;
  if (point_c.z() <= 0.0) return false;

  std::vector<cv::Point3f> object_points;
  object_points.emplace_back(point_c.x(), point_c.y(), point_c.z());
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(
    object_points,
    cv::Vec3d(0.0, 0.0, 0.0),
    cv::Vec3d(0.0, 0.0, 0.0),
    camera_matrix,
    dist_coeffs,
    image_points);
  if (image_points.empty()) return false;
  uv = image_points.front();
  return true;
}

bool projectCameraPointToImage(
  const pcl::PointXYZ &point_camera,
  const cv::Mat &camera_matrix,
  const cv::Mat &dist_coeffs,
  cv::Point2f &uv)
{
  if (point_camera.z <= 0.0f) return false;
  std::vector<cv::Point3f> object_points;
  object_points.emplace_back(point_camera.x, point_camera.y, point_camera.z);
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(
    object_points,
    cv::Vec3d(0.0, 0.0, 0.0),
    cv::Vec3d(0.0, 0.0, 0.0),
    camera_matrix,
    dist_coeffs,
    image_points);
  if (image_points.empty()) return false;
  uv = image_points.front();
  return true;
}

bool transformPointByTf(
  const pcl::PointXYZ &pt_in,
  const geometry_msgs::TransformStamped &tf_msg,
  pcl::PointXYZ &pt_out)
{
  const auto &tr = tf_msg.transform.translation;
  const auto &qr = tf_msg.transform.rotation;
  Eigen::Quaterniond q(qr.w, qr.x, qr.y, qr.z);
  if (q.norm() < 1e-12) return false;
  q.normalize();
  const Eigen::Matrix3d R = q.toRotationMatrix();
  const Eigen::Vector3d t(tr.x, tr.y, tr.z);
  const Eigen::Vector3d v(pt_in.x, pt_in.y, pt_in.z);
  const Eigen::Vector3d vt = R * v + t;
  pt_out.x = static_cast<float>(vt.x());
  pt_out.y = static_cast<float>(vt.y());
  pt_out.z = static_cast<float>(vt.z());
  return true;
}

void drawProjectedPolygon(
  cv::Mat &image,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &polygon,
  const StepConfig &cfg,
  const cv::Mat &camera_matrix,
  const cv::Mat &dist_coeffs,
  const cv::Scalar &color,
  int thickness)
{
  if (!polygon || polygon->size() < 2) return;

  std::vector<cv::Point> valid_points;
  valid_points.reserve(polygon->size());
  for (const auto &pt : polygon->points)
  {
    cv::Point2f uv;
    if (!projectLidarPointToImage(pt, cfg, camera_matrix, dist_coeffs, uv)) continue;
    valid_points.emplace_back(cvRound(uv.x), cvRound(uv.y));
  }
  if (valid_points.size() < 2) return;

  for (size_t i = 0; i < valid_points.size(); ++i)
  {
    const cv::Point &p0 = valid_points[i];
    const cv::Point &p1 = valid_points[(i + 1) % valid_points.size()];
    cv::line(image, p0, p1, color, thickness, cv::LINE_AA);
    cv::circle(image, p0, std::max(2, thickness + 1), color, -1, cv::LINE_AA);
  }
}

void drawProjectedPolygonWithTf(
  cv::Mat &image,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &polygon,
  const geometry_msgs::TransformStamped &src_to_cam_tf,
  const cv::Mat &camera_matrix,
  const cv::Mat &dist_coeffs,
  const cv::Scalar &color,
  int thickness)
{
  if (!polygon || polygon->size() < 2) return;
  std::vector<cv::Point> valid_points;
  valid_points.reserve(polygon->size());
  for (const auto &pt : polygon->points)
  {
    pcl::PointXYZ pt_cam;
    if (!transformPointByTf(pt, src_to_cam_tf, pt_cam)) continue;
    cv::Point2f uv;
    if (!projectCameraPointToImage(pt_cam, camera_matrix, dist_coeffs, uv)) continue;
    valid_points.emplace_back(cvRound(uv.x), cvRound(uv.y));
  }
  if (valid_points.size() < 2) return;
  for (size_t i = 0; i < valid_points.size(); ++i)
  {
    const cv::Point &p0 = valid_points[i];
    const cv::Point &p1 = valid_points[(i + 1) % valid_points.size()];
    cv::line(image, p0, p1, color, thickness, cv::LINE_AA);
    cv::circle(image, p0, std::max(2, thickness + 1), color, -1, cv::LINE_AA);
  }
}

bool saveOverlayImage(const Params &params, const cv::Mat &image)
{
  if (image.empty()) return false;
  std::string output_dir = params.output_path;
  if (!output_dir.empty() && output_dir.back() != '/') output_dir += '/';
  const std::string output_file = output_dir + "hole_step_projection.png";
  const bool ok = cv::imwrite(output_file, image);
  if (ok)
  {
    ROS_INFO_STREAM("[polygon_hole_step_viz] Saved projected overlay image to " << output_file);
  }
  else
  {
    ROS_WARN_STREAM("[polygon_hole_step_viz] Failed to save projected overlay image to " << output_file);
  }
  return ok;
}

visualization_msgs::Marker makeLineStripMarker(
  const std::string &frame_id,
  const std::string &ns,
  int id,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &polygon,
  float r, float g, float b,
  double width)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = width;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  if (polygon)
  {
    for (const auto &pt : polygon->points)
    {
      geometry_msgs::Point p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      marker.points.push_back(p);
    }
    if (!polygon->empty())
    {
      geometry_msgs::Point p;
      p.x = polygon->points.front().x;
      p.y = polygon->points.front().y;
      p.z = polygon->points.front().z;
      marker.points.push_back(p);
    }
  }
  return marker;
}

visualization_msgs::Marker makeSphereMarker(
  const std::string &frame_id,
  const std::string &ns,
  int id,
  const pcl::PointXYZ &center,
  float r, float g, float b,
  double size)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = center.x;
  marker.pose.position.y = center.y;
  marker.pose.position.z = center.z;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  return marker;
}

visualization_msgs::Marker makeTextMarker(
  const std::string &frame_id,
  const std::string &ns,
  int id,
  const pcl::PointXYZ &position,
  const std::string &text,
  double size,
  float r, float g, float b)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = position.x;
  marker.pose.position.y = position.y;
  marker.pose.position.z = position.z;
  marker.scale.z = size;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.text = text;
  return marker;
}

visualization_msgs::Marker makeDeleteMarker(
  const std::string &frame_id,
  const std::string &ns,
  int id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = id;
  marker.action = visualization_msgs::Marker::DELETE;
  return marker;
}

geometry_msgs::Point makeRoiCorner(const ActiveRoi &roi, double x, double y, double z)
{
  Eigen::Vector3d p(x, y, z);
  if (roi.oriented)
  {
    p = roi.center + roi.rotation * p;
  }

  geometry_msgs::Point out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  return out;
}

visualization_msgs::Marker makeRoiBoxMarker(const ActiveRoi &roi, const std::string &frame_id, int id, double width)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "step_roi_box";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = width;
  marker.color.r = 0.3f;
  marker.color.g = 0.9f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;

  std::vector<geometry_msgs::Point> c(8);
  c[0] = makeRoiCorner(roi, roi.params.x_min, roi.params.y_min, roi.params.z_min);
  c[1] = makeRoiCorner(roi, roi.params.x_max, roi.params.y_min, roi.params.z_min);
  c[2] = makeRoiCorner(roi, roi.params.x_max, roi.params.y_max, roi.params.z_min);
  c[3] = makeRoiCorner(roi, roi.params.x_min, roi.params.y_max, roi.params.z_min);
  c[4] = makeRoiCorner(roi, roi.params.x_min, roi.params.y_min, roi.params.z_max);
  c[5] = makeRoiCorner(roi, roi.params.x_max, roi.params.y_min, roi.params.z_max);
  c[6] = makeRoiCorner(roi, roi.params.x_max, roi.params.y_max, roi.params.z_max);
  c[7] = makeRoiCorner(roi, roi.params.x_min, roi.params.y_max, roi.params.z_max);

  const int e[12][2] = {
    {0,1},{1,2},{2,3},{3,0},
    {4,5},{5,6},{6,7},{7,4},
    {0,4},{1,5},{2,6},{3,7}
  };
  for (const auto &edge : e)
  {
    marker.points.push_back(c[edge[0]]);
    marker.points.push_back(c[edge[1]]);
  }
  return marker;
}

void publishCloud(ros::Publisher &pub, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &frame_id)
{
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;
  pub.publish(msg);
}

void cropCloudByRoi(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
  const ActiveRoi &roi,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
{
  output->clear();
  if (!input) return;

  if (!roi.oriented)
  {
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(input);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(roi.params.x_min, roi.params.x_max);
    pass_x.filter(*output);

    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(output);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(roi.params.y_min, roi.params.y_max);
    pass_y.filter(*output);

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(output);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(roi.params.z_min, roi.params.z_max);
    pass_z.filter(*output);
    return;
  }

  const Eigen::Matrix3d R_inv = roi.rotation.transpose();
  output->reserve(input->size());
  for (const auto &pt : input->points)
  {
    const Eigen::Vector3d p_world(pt.x, pt.y, pt.z);
    const Eigen::Vector3d p_local = R_inv * (p_world - roi.center);
    if (p_local.x() < roi.params.x_min || p_local.x() > roi.params.x_max) continue;
    if (p_local.y() < roi.params.y_min || p_local.y() > roi.params.y_max) continue;
    if (p_local.z() < roi.params.z_min || p_local.z() > roi.params.z_max) continue;
    output->push_back(pt);
  }
}

std::vector<cv::Point2f> expandedVisualPolygon(
  const geometry_msgs::PolygonStamped &polygon,
  double margin_px)
{
  std::vector<cv::Point2f> points;
  points.reserve(polygon.polygon.points.size());
  for (const auto &pt : polygon.polygon.points)
  {
    points.emplace_back(pt.x, pt.y);
  }
  if (points.size() < 3 || margin_px <= 0.0) return points;

  cv::Point2f center(0.0f, 0.0f);
  for (const auto &pt : points)
  {
    center += pt;
  }
  center.x /= static_cast<float>(points.size());
  center.y /= static_cast<float>(points.size());

  for (auto &pt : points)
  {
    const cv::Point2f delta = pt - center;
    const double len = std::hypot(delta.x, delta.y);
    if (len < 1e-6) continue;
    const float scale = static_cast<float>((len + margin_px) / len);
    pt = center + delta * scale;
  }
  return points;
}

size_t filterCloudByVisualRoi(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
  const geometry_msgs::PolygonStamped &visual_polygon,
  const OdomState *visual_odom,
  const StepConfig &cfg,
  const cv::Mat &camera_matrix,
  const cv::Mat &dist_coeffs,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
{
  output->clear();
  if (!input || input->empty()) return 0;

  const auto polygon = expandedVisualPolygon(visual_polygon, cfg.visual_roi_margin_px);
  if (polygon.size() < 3) return 0;

  double poly_min_u = std::numeric_limits<double>::max();
  double poly_min_v = std::numeric_limits<double>::max();
  double poly_max_u = -std::numeric_limits<double>::max();
  double poly_max_v = -std::numeric_limits<double>::max();
  for (const auto &pt : polygon)
  {
    poly_min_u = std::min(poly_min_u, static_cast<double>(pt.x));
    poly_min_v = std::min(poly_min_v, static_cast<double>(pt.y));
    poly_max_u = std::max(poly_max_u, static_cast<double>(pt.x));
    poly_max_v = std::max(poly_max_v, static_cast<double>(pt.y));
  }

  size_t points_in_front = 0;
  size_t project_ok = 0;
  double proj_min_u = std::numeric_limits<double>::max();
  double proj_min_v = std::numeric_limits<double>::max();
  double proj_max_u = -std::numeric_limits<double>::max();
  double proj_max_v = -std::numeric_limits<double>::max();

  output->reserve(input->size());
  Eigen::Matrix3d R_bw = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_wb = Eigen::Vector3d::Zero();
  const bool use_body_camera_extrinsic = visual_odom && visual_odom->has_odom;
  if (use_body_camera_extrinsic)
  {
    const auto &q_msg = visual_odom->pose.orientation;
    Eigen::Quaterniond q_wb(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    if (q_wb.norm() > 1e-6)
    {
      q_wb.normalize();
      R_bw = q_wb.toRotationMatrix().transpose();
      t_wb = Eigen::Vector3d(
        visual_odom->pose.position.x,
        visual_odom->pose.position.y,
        visual_odom->pose.position.z);
    }
  }

  for (const auto &pt : input->points)
  {
    const Eigen::Vector3d point_w(pt.x, pt.y, pt.z);
    Eigen::Vector3d point_c;
    if (use_body_camera_extrinsic)
    {
      const Eigen::Vector3d point_b = R_bw * (point_w - t_wb);
      point_c = cfg.visual_Rcb * (point_b - cfg.visual_t_bc);
    }
    else
    {
      point_c = cfg.visual_Rcw * point_w + cfg.visual_Pcw;
    }
    if (point_c.z() <= 0.0) continue;
    ++points_in_front;

    cv::Point2f uv;
    if (!projectCameraPointToImage(
          pcl::PointXYZ(point_c.x(), point_c.y(), point_c.z()),
          camera_matrix,
          dist_coeffs,
          uv)) continue;
    ++project_ok;
    proj_min_u = std::min(proj_min_u, static_cast<double>(uv.x));
    proj_min_v = std::min(proj_min_v, static_cast<double>(uv.y));
    proj_max_u = std::max(proj_max_u, static_cast<double>(uv.x));
    proj_max_v = std::max(proj_max_v, static_cast<double>(uv.y));
    if (cv::pointPolygonTest(polygon, uv, false) < 0.0) continue;
    output->push_back(pt);
  }
  if (output->empty())
  {
    ROS_WARN_THROTTLE(1.0,
                      "[polygon_hole_step_viz] visual ROI debug: input=%zu front=%zu project_ok=%zu "
                      "poly_uv=[%.1f %.1f]-[%.1f %.1f] proj_uv=[%.1f %.1f]-[%.1f %.1f]",
                      input->size(), points_in_front, project_ok,
                      poly_min_u, poly_min_v, poly_max_u, poly_max_v,
                      project_ok > 0 ? proj_min_u : 0.0,
                      project_ok > 0 ? proj_min_v : 0.0,
                      project_ok > 0 ? proj_max_u : 0.0,
                      project_ok > 0 ? proj_max_v : 0.0);
  }
  return output->size();
}

bool saveHolePolygonToTxt(
  const std::string &file_path,
  const StepResult &result,
  const std::string &frame_id,
  const OdomState &odom_state,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &hole_polygon_cloud,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &hole_center_cloud)
{
  std::ofstream ofs(file_path, std::ios::out | std::ios::trunc);
  if (!ofs.is_open())
  {
    ROS_WARN_THROTTLE(2.0, "[polygon_hole_step_viz] Cannot write txt: %s", file_path.c_str());
    return false;
  }

  ofs.setf(std::ios::fixed);
  ofs.precision(6);
  ofs << "# planning snapshot (auto-updated)\n";
  ofs << "stamp: " << ros::Time::now().toSec() << "\n";
  ofs << "frame_id: " << frame_id << "\n";
  ofs << "saved_by: hole_polygon\n";

  Eigen::Vector3d start = Eigen::Vector3d::Zero();
  bool has_start = false;
  if (odom_state.has_odom)
  {
    start = odom_state.position;
    has_start = true;
    ofs << "odom: " << start.x() << " " << start.y() << " " << start.z() << "\n";
    ofs << "start: " << start.x() << " " << start.y() << " " << start.z() << "\n";
  }

  if (!hole_center_cloud->empty())
  {
    const auto &c = hole_center_cloud->points.front();
    ofs << "center: " << c.x << " " << c.y << " " << c.z << "\n";
    if (has_start)
    {
      const double goal_x = 2.0 * static_cast<double>(c.x) - start.x();
      const double goal_y = 2.0 * static_cast<double>(c.y) - start.y();
      const double goal_z = start.z();
      ofs << "goal: " << goal_x << " " << goal_y << " " << goal_z << "\n";
    }
  }
  else
  {
    ofs << "center: nan nan nan\n";
  }

  const size_t n = hole_polygon_cloud ? hole_polygon_cloud->size() : 0;
  ofs << "vertex_count: " << n << "\n";
  ofs << "vertices_xyz:\n";
  if (hole_polygon_cloud)
  {
    for (size_t i = 0; i < hole_polygon_cloud->size(); ++i)
    {
      const auto &p = hole_polygon_cloud->points[i];
      ofs << i << " " << p.x << " " << p.y << " " << p.z << "\n";
    }
  }
  ofs.close();
  return true;
}

bool buildPlanningSnapshotJson(
  const StepResult &result,
  const StepConfig &cfg,
  const OdomState &odom_state,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &hole_polygon_cloud,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &hole_center_cloud,
  std::string &json,
  std::string &reason)
{
  json.clear();
  if (!result.pass) { reason = "detection did not pass"; return false; }
  reason.clear();
  if (!hole_polygon_cloud || hole_polygon_cloud->size() < static_cast<size_t>(cfg.planning_snapshot_min_vertices))
  {
    reason = "vertices too few";
    return false;
  }
  if (!odom_state.has_odom || !odom_state.position.allFinite())
  {
    reason = "missing finite odom/start";
    return false;
  }
  if (!hole_center_cloud || hole_center_cloud->empty())
  {
    reason = "missing hole center";
    return false;
  }

  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(hole_polygon_cloud->size());
  for (const auto &p : hole_polygon_cloud->points)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
    {
      reason = "non-finite vertex";
      return false;
    }
    vertices.emplace_back(p.x, p.y, p.z);
  }
  if (vertices.size() >= 2 && (vertices.front() - vertices.back()).norm() < 1.0e-4)
  {
    vertices.pop_back();
  }
  if (vertices.size() < static_cast<size_t>(cfg.planning_snapshot_min_vertices))
  {
    reason = "vertices too few after dedup";
    return false;
  }

  const auto &center_pt = hole_center_cloud->points.front();
  if (!std::isfinite(center_pt.x) || !std::isfinite(center_pt.y) || !std::isfinite(center_pt.z))
  {
    reason = "non-finite center";
    return false;
  }
  const Eigen::Vector3d center(center_pt.x, center_pt.y, center_pt.z);
  const Eigen::Vector3d start = odom_state.position;
  const Eigen::Vector3d goal(2.0 * center.x() - start.x(),
                             2.0 * center.y() - start.y(),
                             start.z());

  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < vertices.size(); ++i)
  {
    const Eigen::Vector3d a = vertices[i] - center;
    const Eigen::Vector3d b = vertices[(i + 1) % vertices.size()] - center;
    normal += a.cross(b);
  }
  if (!normal.allFinite() || normal.norm() < 1.0e-6)
  {
    reason = "invalid polygon normal";
    return false;
  }
  normal.normalize();

  const double min_side_sep = 1.0e-3;
  const double start_side = normal.dot(start - center);
  const double goal_side = normal.dot(goal - center);
  if (std::abs(start_side) < min_side_sep ||
      std::abs(goal_side) < min_side_sep ||
      start_side * goal_side >= 0.0)
  {
    reason = "start/goal not on opposite gate sides";
    return false;
  }

  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(6);
  oss << "{";
  oss << "\"stamp\":" << ros::Time::now().toSec() << ",";
  oss << "\"frame_id\":\"" << cfg.frame_id << "\",";
  oss << "\"saved_by\":\"hole_polygon\",";
  oss << "\"odom\":[" << start.x() << "," << start.y() << "," << start.z() << "],";
  oss << "\"start\":[" << start.x() << "," << start.y() << "," << start.z() << "],";
  oss << "\"center\":[" << center.x() << "," << center.y() << "," << center.z() << "],";
  oss << "\"goal\":[" << goal.x() << "," << goal.y() << "," << goal.z() << "],";
  oss << "\"vertex_count\":" << vertices.size() << ",";
  oss << "\"vertices_xyz\":[";
  for (size_t i = 0; i < vertices.size(); ++i)
  {
    if (i > 0) oss << ",";
    oss << "[" << vertices[i].x() << "," << vertices[i].y() << "," << vertices[i].z() << "]";
  }
  oss << "]}";
  json = oss.str();
  return true;
}

void transformCloudByTf(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
  const geometry_msgs::TransformStamped &tf_msg,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
{
  const auto &tr = tf_msg.transform.translation;
  const auto &qr = tf_msg.transform.rotation;
  Eigen::Quaterniond q(qr.w, qr.x, qr.y, qr.z);
  q.normalize();
  const Eigen::Matrix3d R = q.toRotationMatrix();
  const Eigen::Vector3d t(tr.x, tr.y, tr.z);

  output->clear();
  output->reserve(input->size());
  for (const auto &p : input->points)
  {
    const Eigen::Vector3d v(p.x, p.y, p.z);
    const Eigen::Vector3d vt = R * v + t;
    output->push_back(pcl::PointXYZ(vt.x(), vt.y(), vt.z()));
  }
}

void runDetectionFromRawCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_raw_cloud,
  const ActiveRoi &roi,
  const VisualRoiState *visual_roi,
  const OdomState *visual_projection_odom,
  const cv::Mat &visual_camera_matrix,
  const cv::Mat &visual_dist_coeffs,
  const StepConfig &cfg,
  StepResult &result,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &raw_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &roi_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &non_plane_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &empty_candidate_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &hole_cluster_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_polygon_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &hole_polygon_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &hole_center_cloud)
{
  result.summary = "not_started";
  result.pass = false;
  result.has_plane = false;
  result.has_hole = false;
  result.plane_area = 0.0;
  result.hole_area = 0.0;
  result.area_ratio = 0.0;
  result.roi_points = 0;
  result.plane_points = 0;
  result.non_plane_points = 0;
  result.far_candidate_points = 0;
  result.hole_cluster_points = 0;
  result.visual_roi_used = false;
  result.visual_roi_points = 0;

  raw_cloud->clear();
  roi_cloud->clear();
  plane_cloud->clear();
  non_plane_cloud->clear();
  empty_candidate_cloud->clear();
  hole_cluster_cloud->clear();
  plane_polygon_cloud->clear();
  hole_polygon_cloud->clear();
  hole_center_cloud->clear();

  if (!input_raw_cloud || input_raw_cloud->empty())
  {
    result.summary = "waiting: no lidar frames yet";
    return;
  }

  *raw_cloud = *input_raw_cloud;

  cropCloudByRoi(raw_cloud, roi, roi_cloud);
  if (visual_roi && visual_roi->has_polygon)
  {
    auto visual_filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    const size_t visual_points = filterCloudByVisualRoi(
      roi_cloud,
      visual_roi->polygon,
      visual_projection_odom,
      cfg,
      visual_camera_matrix,
      visual_dist_coeffs,
      visual_filtered);
    result.visual_roi_points = visual_points;
    if (visual_points > 0)
    {
      *roi_cloud = *visual_filtered;
      result.visual_roi_used = true;
    }
    else if (cfg.visual_roi_required)
    {
      result.summary = "failed: visual roi cloud too small";
      return;
    }
    else
    {
      ROS_WARN_THROTTLE(1.0, "[polygon_hole_step_viz] visual ROI produced no projected points. Continue with 3D ROI only.");
    }
  }

  result.roi_points = roi_cloud->size();

  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(roi_cloud);
  vg.setLeafSize(cfg.voxel_leaf, cfg.voxel_leaf, cfg.voxel_leaf);
  vg.filter(*roi_cloud);

  if (roi_cloud->size() < 50)
  {
    result.summary = "failed: roi cloud too small";
    return;
  }

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(cfg.plane_distance_threshold);

  auto working_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*roi_cloud));
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  bool accepted_plane = false;
  std::string reject_reason = "plane not found";
  const int max_trials = std::max(1, cfg.plane_candidate_max_trials);

  for (int trial = 0; trial < max_trials; ++trial)
  {
    if (working_cloud->size() < 50)
    {
      reject_reason = "remaining cloud too small";
      break;
    }

    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setInputCloud(working_cloud);
    seg.segment(*inliers, *coeff);
    if (inliers->indices.empty())
    {
      reject_reason = "plane not found";
      break;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(working_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    auto candidate_plane = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*candidate_plane);
    extract.setNegative(true);
    auto candidate_non_plane = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*candidate_non_plane);

    if (candidate_plane->size() < 20)
    {
      reject_reason = "plane points too small";
      working_cloud = candidate_non_plane;
      continue;
    }

    const Eigen::Vector3d candidate_normal(coeff->values[0], coeff->values[1], coeff->values[2]);
    const Eigen::Vector3d n = candidate_normal.normalized();
    const double up_dot = std::abs(n.dot(cfg.plane_filter_up_axis));
    if (cfg.enable_plane_orientation_filter && up_dot > cfg.plane_filter_max_up_dot)
    {
      std::ostringstream oss;
      oss << "reject ground-like plane trial=" << trial
          << " up_dot=" << up_dot
          << " max_up_dot=" << cfg.plane_filter_max_up_dot;
      reject_reason = oss.str();
      working_cloud = candidate_non_plane;
      continue;
    }

    *plane_cloud = *candidate_plane;
    *non_plane_cloud = *candidate_non_plane;
    normal = n;
    accepted_plane = true;
    break;
  }

  if (!accepted_plane)
  {
    result.summary = "failed: " + reject_reason;
    return;
  }

  result.plane_points = plane_cloud->size();
  result.non_plane_points = non_plane_cloud->size();
  result.has_plane = plane_cloud->size() >= 20;
  if (!result.has_plane)
  {
    result.summary = "failed: plane points too small";
    return;
  }

  const Eigen::Matrix3d R_align = computeAlignmentRotation(normal);
  const Eigen::Matrix3d R_inv = R_align.transpose();

  auto aligned_plane = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  aligned_plane->reserve(plane_cloud->size());
  double z_mean = 0.0;
  for (const auto &pt : plane_cloud->points)
  {
    Eigen::Vector3d p(pt.x, pt.y, pt.z);
    Eigen::Vector3d a = R_align * p;
    aligned_plane->push_back(pcl::PointXYZ(a.x(), a.y(), 0.0f));
    z_mean += a.z();
  }
  z_mean /= std::max<size_t>(1, plane_cloud->size());

  std::string plane_poly_reason;
  Polygon2D plane_poly = makePlanePolygon2D(aligned_plane, cfg, plane_poly_reason);
  if (!plane_poly.valid)
  {
    result.summary = "failed: " + plane_poly_reason;
    return;
  }

  result.plane_area = plane_poly.area;
  auto empty_cells = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<EmptyClusterCandidate> empty_clusters;
  std::string grid_reason;
  const bool ok = buildEmptyCandidatesOnPlane(
    plane_poly.hull, aligned_plane, cfg, empty_cells, empty_clusters, grid_reason);

  result.far_candidate_points = empty_cells->size();
  *empty_candidate_cloud = *toOriginalCloud(empty_cells, R_inv, z_mean, false);
  *plane_polygon_cloud = *toOriginalCloud(plane_poly.hull, R_inv, z_mean, false);

  if (!ok)
  {
    result.summary = "failed: empty-cell extraction failed (" + grid_reason + ")";
    return;
  }
  if (empty_clusters.empty())
  {
    result.summary = "failed: no enclosed empty region on plane";
    return;
  }

  double best_score = -std::numeric_limits<double>::infinity();
  Polygon2D best_hole_poly;
  pcl::PointCloud<pcl::PointXYZ>::Ptr best_cluster(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto &cluster : empty_clusters)
  {
    Polygon2D hole_poly = makeConvexPolygon2D(cluster.cells);
    if (!hole_poly.valid) continue;
    if (hole_poly.area >= plane_poly.area * 0.95) continue;
    const double empty_ratio =
      cluster.empty_area / std::max(1e-9, hole_poly.area);
    if (empty_ratio < cfg.hole_min_empty_area_ratio) continue;
    const double hole_boundary_unsupported_ratio = boundaryUnsupportedRatio(
      hole_poly.hull,
      aligned_plane,
      cfg.hole_boundary_sample_step,
      cfg.hole_boundary_support_radius,
      cfg.hole_boundary_min_support);
    if (hole_boundary_unsupported_ratio > cfg.hole_boundary_max_unsupported_ratio) continue;

    const double dx = hole_poly.centroid.x - plane_poly.centroid.x;
    const double dy = hole_poly.centroid.y - plane_poly.centroid.y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    const double score = hole_poly.area * empty_ratio / (1.0 + 2.0 * dist);
    if (score > best_score)
    {
      best_score = score;
      best_hole_poly = hole_poly;
      *best_cluster = *cluster.cells;
    }
  }

  if (!(best_score > -1e8 && best_hole_poly.valid))
  {
    result.summary = "failed: no valid convex hole polygon";
    return;
  }

  result.has_hole = true;
  result.hole_area = best_hole_poly.area;
  result.area_ratio = result.hole_area / std::max(1e-9, result.plane_area);
  result.pass = result.area_ratio > cfg.pass_area_ratio;
  {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(4);
    oss << (result.pass ? "pass" : "fail")
        << ": hole area ratio=" << result.area_ratio
        << " threshold=" << cfg.pass_area_ratio;
    result.summary = oss.str();
  }

  *hole_polygon_cloud = *toOriginalCloud(best_hole_poly.hull, R_inv, z_mean, false);
  *hole_cluster_cloud = *toOriginalCloud(best_cluster, R_inv, z_mean, false);
  hole_center_cloud->push_back(toOriginalPoint(best_hole_poly.centroid, R_inv, z_mean));
  result.hole_cluster_points = hole_cluster_cloud->size();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "polygon_hole_step_viz");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  Params params = loadParameters(pnh);
  StepConfig cfg = loadStepConfig(pnh);
  if (!cfg.realtime_mode)
  {
    ROS_WARN("[polygon_hole_step_viz] realtime_mode is false, but this node currently runs in realtime subscriber mode.");
  }

  ros::Publisher observation_pub = pnh.advertise<aperture_detector::ApertureObservation>("observation", 1, true);
  ros::Publisher raw_pub = pnh.advertise<sensor_msgs::PointCloud2>("raw_multi_cloud", 1, true);
  ros::Publisher roi_pub = pnh.advertise<sensor_msgs::PointCloud2>("roi_cloud", 1, true);
  ros::Publisher plane_pub = pnh.advertise<sensor_msgs::PointCloud2>("plane_cloud", 1, true);
  ros::Publisher non_plane_pub = pnh.advertise<sensor_msgs::PointCloud2>("non_plane_cloud", 1, true);
  ros::Publisher far_pub = pnh.advertise<sensor_msgs::PointCloud2>("far_candidate_cloud", 1, true);
  ros::Publisher empty_pub = pnh.advertise<sensor_msgs::PointCloud2>("empty_candidate_cloud", 1, true);
  ros::Publisher hole_cluster_pub = pnh.advertise<sensor_msgs::PointCloud2>("hole_cluster_cloud", 1, true);
  ros::Publisher plane_poly_pub = pnh.advertise<sensor_msgs::PointCloud2>("plane_polygon_cloud", 1, true);
  ros::Publisher hole_poly_pub = pnh.advertise<sensor_msgs::PointCloud2>("hole_polygon_cloud", 1, true);
  ros::Publisher hole_center_pub = pnh.advertise<sensor_msgs::PointCloud2>("hole_center_cloud", 1, true);
  ros::Publisher marker_pub = pnh.advertise<visualization_msgs::MarkerArray>("step_markers", 1, true);
  ros::Publisher overlay_image_pub = pnh.advertise<sensor_msgs::Image>("overlay_image", 1, true);
  ros::Publisher planning_snapshot_json_pub;
  if (cfg.publish_planning_snapshot_json)
  {
    planning_snapshot_json_pub = pnh.advertise<std_msgs::String>(cfg.planning_snapshot_json_topic, 1, true);
  }

  std::mutex data_mutex;
  std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_window;
  ros::Time latest_cloud_stamp;
  ros::WallTime latest_cloud_received;
  double cloud_timeout = 1.5;
  pnh.param("cloud_timeout", cloud_timeout, 1.5);
  cv::Mat latest_image;
  OdomState latest_odom;
  std::deque<OdomState> odom_history;
  DynamicRoiState latest_dynamic_roi;
  VisualRoiState latest_visual_roi;
  CameraInfoState latest_camera_info;
  bool detection_enabled = cfg.detection_initial_enabled;
  size_t total_frames_received = 0;
  size_t total_points_received = 0;
  size_t total_frames_transformed = 0;
  size_t total_frames_dropped_tf = 0;

  tf2_ros::Buffer tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  if (cfg.transform_input_to_target_frame || (cfg.publish_overlay_image && cfg.projection_use_tf))
  {
    tf_listener.reset(new tf2_ros::TransformListener(tf_buffer));
  }

  const int max_window_frames = std::max(1, cfg.accumulate_frames);
  const int min_process_frames = std::max(1, cfg.min_frames_to_process);
  ROS_INFO_STREAM("[polygon_hole_step_viz] Realtime mode enabled. lidar_topic=" << params.lidar_topic
                  << " image_topic=" << cfg.image_topic
                  << " detection_enable_topic=" << cfg.detection_enable_topic
                  << " detection_initial_enabled=" << (cfg.detection_initial_enabled ? "true" : "false")
                  << " dynamic_roi_enable="
                  << (cfg.dynamic_roi_enable ? "true" : "false")
                  << " dynamic_roi_source=" << cfg.dynamic_roi_source
                  << " dynamic_roi_pose_topic=" << cfg.dynamic_roi_pose_topic
                  << " dynamic_roi_odom_topic=" << cfg.dynamic_roi_odom_topic
                  << " dynamic_roi_use_pose_orientation="
                  << (cfg.dynamic_roi_use_pose_orientation ? "true" : "false")
                  << " visual_roi_enable=" << (cfg.visual_roi_enable ? "true" : "false")
                  << " visual_roi_topic=" << cfg.visual_roi_topic
                  << " visual_roi_margin_px=" << cfg.visual_roi_margin_px
                  << " visual_camera_info_topic=" << cfg.visual_camera_info_topic
                  << " accumulate_frames=" << max_window_frames
                  << " min_frames_to_process=" << min_process_frames
                  << " transform_input_to_target_frame="
                  << (cfg.transform_input_to_target_frame ? "true" : "false")
                  << " plane_orientation_filter="
                  << (cfg.enable_plane_orientation_filter ? "true" : "false")
                  << " plane_filter_max_up_dot=" << cfg.plane_filter_max_up_dot
                  << " plane_polygon_use_concave_hull="
                  << (cfg.plane_polygon_use_concave_hull ? "true" : "false")
                  << " concave_alpha=" << cfg.plane_polygon_concave_alpha
                  << " hole_min_empty_area_ratio=" << cfg.hole_min_empty_area_ratio
                  << " hole_boundary_max_unsupported_ratio=" << cfg.hole_boundary_max_unsupported_ratio
                  << " (建议先用80~90帧)");

  auto lidar_cb = [&](const sensor_msgs::PointCloud2ConstPtr &msg) {
    {
      std::lock_guard<std::mutex> lock(data_mutex);
      if (!detection_enabled)
      {
        return;
      }
    }
    if (msg->header.frame_id.empty() ||
        (!cfg.transform_input_to_target_frame && msg->header.frame_id != cfg.frame_id)) {
      ROS_WARN_THROTTLE(1.0, "[aperture_detector] Cloud frame is not the configured target frame; enable TF conversion.");
      return;
    }
    auto frame_cloud_src = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *frame_cloud_src);
    auto frame_cloud = frame_cloud_src;

    if (cfg.transform_input_to_target_frame &&
        !msg->header.frame_id.empty() &&
        msg->header.frame_id != cfg.frame_id)
    {
      geometry_msgs::TransformStamped tf_msg;
      bool tf_ok = false;
      try
      {
        tf_msg = tf_buffer.lookupTransform(
          cfg.frame_id,
          msg->header.frame_id,
          msg->header.stamp,
          ros::Duration(std::max(0.0, cfg.tf_lookup_timeout)));
        tf_ok = true;
      }
      catch (const tf2::TransformException &ex) {
        ++total_frames_dropped_tf;
        ROS_WARN_THROTTLE(1.0, "[aperture_detector] No cloud-time TF %s -> %s: %s",
                          msg->header.frame_id.c_str(), cfg.frame_id.c_str(), ex.what());
        return;
      }

      if (tf_ok)
      {
        auto transformed = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        transformCloudByTf(frame_cloud_src, tf_msg, transformed);
        frame_cloud = transformed;
        ++total_frames_transformed;
      }
    }

    std::lock_guard<std::mutex> lock(data_mutex);
    latest_cloud_stamp = msg->header.stamp;
    latest_cloud_received = ros::WallTime::now();
    cloud_window.push_back(frame_cloud);
    ++total_frames_received;
    total_points_received += frame_cloud->size();
    while (static_cast<int>(cloud_window.size()) > max_window_frames)
    {
      cloud_window.pop_front();
    }
  };
  ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(params.lidar_topic, 50, lidar_cb);

  ros::Subscriber detection_enable_sub = pnh.subscribe<std_msgs::Bool>(
    cfg.detection_enable_topic, 1,
    [&](const std_msgs::BoolConstPtr &msg) {
      std::lock_guard<std::mutex> lock(data_mutex);
      const bool new_state = msg->data;
      if (detection_enabled == new_state)
      {
        return;
      }
      detection_enabled = new_state;
      if (!detection_enabled)
      {
        cloud_window.clear();
        latest_image.release();
      }
      ROS_INFO_STREAM("[polygon_hole_step_viz] detection "
                      << (detection_enabled ? "enabled" : "disabled")
                      << " via " << cfg.detection_enable_topic);
    },
    ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

  ros::Subscriber dynamic_roi_sub;
  if (cfg.dynamic_roi_enable && cfg.dynamic_roi_source != "odom")
  {
    auto dynamic_roi_cb = [&](const geometry_msgs::PoseStampedConstPtr &msg) {
      std::lock_guard<std::mutex> lock(data_mutex);
      latest_dynamic_roi.has_pose = true;
      latest_dynamic_roi.pose = *msg;
      latest_dynamic_roi.rcv_stamp = ros::Time::now();
    };
    dynamic_roi_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      cfg.dynamic_roi_pose_topic, 10, dynamic_roi_cb,
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
  }

  ros::Subscriber visual_roi_sub;
  if (cfg.visual_roi_enable)
  {
    auto visual_roi_cb = [&](const geometry_msgs::PolygonStampedConstPtr &msg) {
      std::lock_guard<std::mutex> lock(data_mutex);
      latest_visual_roi.has_polygon = msg->polygon.points.size() >= 3;
      latest_visual_roi.polygon = *msg;
      latest_visual_roi.rcv_stamp = ros::Time::now();
    };
    visual_roi_sub = nh.subscribe<geometry_msgs::PolygonStamped>(
      cfg.visual_roi_topic, 5, visual_roi_cb,
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
  }

  ros::Subscriber camera_info_sub;
  if (cfg.visual_roi_enable)
  {
    auto camera_info_cb = [&](const sensor_msgs::CameraInfoConstPtr &msg) {
      CameraInfoState state;
      state.has_info = true;
      state.camera_matrix = (cv::Mat_<double>(3, 3) <<
        msg->K[0], msg->K[1], msg->K[2],
        msg->K[3], msg->K[4], msg->K[5],
        msg->K[6], msg->K[7], msg->K[8]);
      state.dist_coeffs = cv::Mat::zeros(1, std::max<size_t>(5, msg->D.size()), CV_64F);
      for (size_t i = 0; i < msg->D.size(); ++i)
      {
        state.dist_coeffs.at<double>(0, static_cast<int>(i)) = msg->D[i];
      }
      std::lock_guard<std::mutex> lock(data_mutex);
      latest_camera_info = state;
    };
    camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo>(
      cfg.visual_camera_info_topic, 5, camera_info_cb,
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
  }

  auto odom_cb = [&](const nav_msgs::OdometryConstPtr &msg) {
    if (msg->header.frame_id != cfg.frame_id) {
      ROS_WARN_THROTTLE(1.0, "[aperture_detector] Odom frame does not match target frame.");
      return;
    }
    std::lock_guard<std::mutex> lock(data_mutex);
    latest_odom.has_odom = true;
    latest_odom.pose = msg->pose.pose;
    latest_odom.msg_stamp = msg->header.stamp;
    latest_odom.rcv_stamp = ros::Time::now();
    latest_odom.position.x() = msg->pose.pose.position.x;
    latest_odom.position.y() = msg->pose.pose.position.y;
    latest_odom.position.z() = msg->pose.pose.position.z;
    odom_history.push_back(latest_odom);

    const ros::Time newest_stamp = latest_odom.msg_stamp.isZero() ? latest_odom.rcv_stamp : latest_odom.msg_stamp;
    while (!odom_history.empty())
    {
      const ros::Time oldest_stamp = odom_history.front().msg_stamp.isZero()
        ? odom_history.front().rcv_stamp
        : odom_history.front().msg_stamp;
      if ((newest_stamp - oldest_stamp).toSec() <= std::max(0.1, cfg.odom_history_duration)) break;
      odom_history.pop_front();
    }
  };
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(cfg.dynamic_roi_odom_topic, 10, odom_cb);

  ros::Subscriber image_sub;
  if (cfg.publish_overlay_image)
  {
    auto image_cb = [&](const sensor_msgs::ImageConstPtr &msg) {
      try
      {
        cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        std::lock_guard<std::mutex> lock(data_mutex);
        latest_image = img;
      }
      catch (const cv_bridge::Exception &e)
      {
        ROS_WARN_THROTTLE(2.0, "[polygon_hole_step_viz] image decode failed: %s", e.what());
      }
    };
    image_sub = nh.subscribe<sensor_msgs::Image>(cfg.image_topic, 10, image_cb);
  }

  // Use wall time to avoid blocking when /use_sim_time=true but /clock is absent.
  ros::WallRate rate(std::max(0.5, cfg.publish_rate));
  while (ros::ok())
  {
    ros::spinOnce();

    std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> local_window;
    cv::Mat local_image;
    OdomState local_odom;
    std::deque<OdomState> local_odom_history;
    DynamicRoiState local_dynamic_roi;
    VisualRoiState local_visual_roi;
    CameraInfoState local_camera_info;
    bool local_detection_enabled = true;
    ros::Time observation_stamp;
    ros::WallTime observation_received;
    size_t total_frames_snapshot = 0;
    size_t total_points_snapshot = 0;
    {
      std::lock_guard<std::mutex> lock(data_mutex);
      local_detection_enabled = detection_enabled;
      local_window = cloud_window;
      observation_stamp = latest_cloud_stamp;
      observation_received = latest_cloud_received;
      if (local_detection_enabled)
      {
        local_image = latest_image.clone();
      }
      local_odom = latest_odom;
      local_odom_history = odom_history;
      local_dynamic_roi = latest_dynamic_roi;
      local_visual_roi = latest_visual_roi;
      local_camera_info = latest_camera_info;
      total_frames_snapshot = total_frames_received;
      total_points_snapshot = total_points_received;
    }

    const bool cloud_stale = observation_received.isZero() ||
      (cloud_timeout > 0.0 && (ros::WallTime::now() - observation_received).toSec() > cloud_timeout);
    if (!local_detection_enabled || cloud_stale)
    {
      ActiveRoi active_roi = makeStaticRoi(params);
      auto empty_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
      StepResult result;
      result.lidar_frames = 0;
      result.lidar_points_raw = 0;
      result.lidar_points_accumulated = 0;
      result.summary = cloud_stale ? "waiting: fresh point cloud" : "disabled: detection_enable=false";
      aperture_detector::ApertureObservation invalid;
      invalid.header.stamp = observation_stamp;
      invalid.header.frame_id = cfg.frame_id;
      invalid.reason = result.summary;
      observation_pub.publish(invalid);
      if (cloud_stale) {
        std::lock_guard<std::mutex> lock(data_mutex);
        cloud_window.clear();
      }

      publishCloud(raw_pub, empty_cloud, cfg.frame_id);
      publishCloud(roi_pub, empty_cloud, cfg.frame_id);
      publishCloud(plane_pub, empty_cloud, cfg.frame_id);
      publishCloud(non_plane_pub, empty_cloud, cfg.frame_id);
      publishCloud(far_pub, empty_cloud, cfg.frame_id);
      publishCloud(empty_pub, empty_cloud, cfg.frame_id);
      publishCloud(hole_cluster_pub, empty_cloud, cfg.frame_id);
      publishCloud(plane_poly_pub, empty_cloud, cfg.frame_id);
      publishCloud(hole_poly_pub, empty_cloud, cfg.frame_id);
      publishCloud(hole_center_pub, empty_cloud, cfg.frame_id);

      visualization_msgs::MarkerArray markers;
      markers.markers.push_back(makeRoiBoxMarker(active_roi, cfg.frame_id, 0, cfg.roi_line_width));
      markers.markers.push_back(makeDeleteMarker(cfg.frame_id, "step_plane_poly", 1));
      markers.markers.push_back(makeDeleteMarker(cfg.frame_id, "step_hole_poly", 2));
      markers.markers.push_back(makeDeleteMarker(cfg.frame_id, "step_hole_center", 3));
      markers.markers.push_back(makeTextMarker(cfg.frame_id, "step_status", 4, roiTextAnchor(active_roi), "detection disabled", 0.12, 1.0f, 0.2f, 0.2f));
      marker_pub.publish(markers);

      ROS_INFO_STREAM_THROTTLE(1.0, "[polygon_hole_step_viz] " << result.summary
                            << " | window_frames=" << local_window.size()
                            << " recv_frames_total=" << total_frames_snapshot
                            << " recv_points_total=" << total_points_snapshot
                            << " threshold=" << cfg.pass_area_ratio);
      rate.sleep();
      continue;
    }

    VisualRoiState active_visual_roi;
    bool visual_roi_active = false;
    if (cfg.visual_roi_enable && local_visual_roi.has_polygon)
    {
      const double visual_age = (ros::Time::now() - local_visual_roi.rcv_stamp).toSec();
      if (cfg.visual_roi_timeout <= 0.0 || visual_age <= cfg.visual_roi_timeout)
      {
        active_visual_roi = local_visual_roi;
        if (local_camera_info.has_info)
        {
          visual_roi_active = true;
        }
        else
        {
          ROS_WARN_THROTTLE(1.0, "[polygon_hole_step_viz] waiting for camera info on %s. Continue with 3D ROI only.",
                            cfg.visual_camera_info_topic.c_str());
        }
      }
      else
      {
        ROS_WARN_THROTTLE(1.0, "[polygon_hole_step_viz] visual ROI timeout: age=%.3fs timeout=%.3fs. Continue with 3D ROI only.",
                          visual_age, cfg.visual_roi_timeout);
      }
    }
    else if (cfg.visual_roi_enable)
    {
      ROS_WARN_THROTTLE(1.0, "[polygon_hole_step_viz] waiting for visual ROI polygon on %s. Continue with 3D ROI only.",
                        cfg.visual_roi_topic.c_str());
    }

    ActiveRoi active_roi = makeStaticRoi(params);
    bool dynamic_roi_active = false;
    const bool dynamic_roi_use_odom = (cfg.dynamic_roi_source == "odom");
    OdomState roi_odom = local_odom;
    if (cfg.dynamic_roi_enable && dynamic_roi_use_odom && visual_roi_active)
    {
      OdomState synced_odom;
      double sync_dt = 0.0;
      if (findNearestOdomByStamp(local_odom_history, active_visual_roi.polygon.header.stamp, cfg.odom_sync_max_dt, synced_odom, sync_dt))
      {
        roi_odom = synced_odom;
        ROS_INFO_STREAM_THROTTLE(1.0, "[polygon_hole_step_viz] using visual-stamp synced odom for ROI: dt=" << sync_dt
                                << "s stamp=" << active_visual_roi.polygon.header.stamp);
      }
      else
      {
        ROS_WARN_THROTTLE(1.0, "[polygon_hole_step_viz] no odom near visual stamp %.6f within %.3fs. Use latest odom for ROI.",
                          active_visual_roi.polygon.header.stamp.toSec(), cfg.odom_sync_max_dt);
      }
    }

    if (cfg.dynamic_roi_enable && dynamic_roi_use_odom && roi_odom.has_odom)
    {
      const double odom_age = (ros::Time::now() - roi_odom.rcv_stamp).toSec();
      if (cfg.dynamic_roi_timeout <= 0.0 || odom_age <= cfg.dynamic_roi_timeout)
      {
        if (cfg.dynamic_roi_use_pose_orientation)
        {
          active_roi = makeOrientedDynamicRoi(params, roi_odom.pose);
        }
        else
        {
          active_roi = makeAxisAlignedDynamicRoi(params, roi_odom.position);
        }
        dynamic_roi_active = true;
      }
      else
      {
        ROS_WARN_THROTTLE(1.0, "[polygon_hole_step_viz] dynamic ROI odom timeout: age=%.3fs timeout=%.3fs. Fallback to static ROI.",
                          odom_age, cfg.dynamic_roi_timeout);
      }
    }
    else if (cfg.dynamic_roi_enable && dynamic_roi_use_odom)
    {
      ROS_WARN_THROTTLE(1.0, "[polygon_hole_step_viz] waiting for dynamic ROI odom on %s. Fallback to static ROI.",
                        cfg.dynamic_roi_odom_topic.c_str());
    }
    else if (cfg.dynamic_roi_enable && local_dynamic_roi.has_pose)
    {
      const double pose_age = (ros::Time::now() - local_dynamic_roi.rcv_stamp).toSec();
      if (cfg.dynamic_roi_timeout <= 0.0 || pose_age <= cfg.dynamic_roi_timeout)
      {
        if (cfg.dynamic_roi_use_pose_orientation)
        {
          active_roi = makeOrientedDynamicRoi(params, local_dynamic_roi.pose.pose);
        }
        else
        {
          const Eigen::Vector3d roi_center(local_dynamic_roi.pose.pose.position.x,
                                           local_dynamic_roi.pose.pose.position.y,
                                           local_dynamic_roi.pose.pose.position.z);
          active_roi = makeAxisAlignedDynamicRoi(params, roi_center);
        }
        dynamic_roi_active = true;
      }
      else
      {
        ROS_WARN_THROTTLE(1.0, "[polygon_hole_step_viz] dynamic ROI pose timeout: age=%.3fs timeout=%.3fs. Fallback to static ROI.",
                          pose_age, cfg.dynamic_roi_timeout);
      }
    }
    else if (cfg.dynamic_roi_enable)
    {
      ROS_WARN_THROTTLE(1.0, "[polygon_hole_step_viz] waiting for dynamic ROI pose on %s. Fallback to static ROI.",
                        cfg.dynamic_roi_pose_topic.c_str());
    }

    auto accumulated_raw_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    size_t points_in_window = 0;
    for (const auto &frame : local_window)
    {
      if (!frame) continue;
      points_in_window += frame->size();
      *accumulated_raw_cloud += *frame;
    }

    auto raw_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    auto roi_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    auto plane_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    auto non_plane_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    auto empty_candidate_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    auto hole_cluster_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    auto plane_polygon_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    auto hole_polygon_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    auto hole_center_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    StepResult result;
    result.lidar_frames = local_window.size();
    result.lidar_points_raw = points_in_window;
    result.lidar_points_accumulated = accumulated_raw_cloud->size();

    if (static_cast<int>(local_window.size()) < min_process_frames)
    {
      std::ostringstream wait_msg;
      wait_msg << "waiting: frames " << local_window.size() << "/" << min_process_frames;
      result.summary = wait_msg.str();
    }
    else
    {
      runDetectionFromRawCloud(
        accumulated_raw_cloud, active_roi,
        visual_roi_active ? &active_visual_roi : nullptr,
        (visual_roi_active && dynamic_roi_use_odom && roi_odom.has_odom) ? &roi_odom : nullptr,
        local_camera_info.camera_matrix,
        local_camera_info.dist_coeffs,
        cfg, result,
        raw_cloud, roi_cloud, plane_cloud, non_plane_cloud,
        empty_candidate_cloud, hole_cluster_cloud, plane_polygon_cloud,
        hole_polygon_cloud, hole_center_cloud);
    }

    cv::Mat overlay_image;
    if (cfg.publish_overlay_image && cfg.has_extrinsics && !local_image.empty())
    {
      overlay_image = ensureBgrImage(local_image);
      const cv::Mat camera_matrix = makeCameraMatrix(params);
      const cv::Mat dist_coeffs = makeDistCoeffs(params);
      bool projected_with_tf = false;
      geometry_msgs::TransformStamped src_to_cam_tf;
      if (cfg.projection_use_tf)
      {
        try
        {
          src_to_cam_tf = tf_buffer.lookupTransform(
            cfg.projection_camera_frame,
            cfg.frame_id,
            ros::Time(0),
            ros::Duration(std::max(0.0, cfg.tf_lookup_timeout)));
          drawProjectedPolygonWithTf(
            overlay_image,
            hole_polygon_cloud,
            src_to_cam_tf,
            camera_matrix,
            dist_coeffs,
            cv::Scalar(0, 140, 255),
            3);
          projected_with_tf = true;
        }
        catch (const tf2::TransformException &ex)
        {
          ROS_WARN_THROTTLE(1.0, "[polygon_hole_step_viz] projection TF failed %s -> %s: %s",
                            cfg.frame_id.c_str(), cfg.projection_camera_frame.c_str(), ex.what());
        }
      }
      if (!projected_with_tf)
      {
        drawProjectedPolygon(
          overlay_image,
          hole_polygon_cloud,
          cfg,
          camera_matrix,
          dist_coeffs,
          cv::Scalar(0, 140, 255),
          3);
      }
      if (!hole_center_cloud->empty())
      {
        cv::Point2f uv;
        bool center_ok = false;
        if (projected_with_tf)
        {
          pcl::PointXYZ center_cam;
          if (transformPointByTf(hole_center_cloud->points.front(), src_to_cam_tf, center_cam))
          {
            center_ok = projectCameraPointToImage(center_cam, camera_matrix, dist_coeffs, uv);
          }
        }
        else
        {
          center_ok = projectLidarPointToImage(hole_center_cloud->points.front(), cfg, camera_matrix, dist_coeffs, uv);
        }

        if (center_ok)
        {
            const cv::Point center_px(cvRound(uv.x), cvRound(uv.y));
            cv::circle(overlay_image, center_px, 6, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
            cv::putText(
              overlay_image,
              "hole center",
              center_px + cv::Point(8, -8),
              cv::FONT_HERSHEY_SIMPLEX,
              0.55,
              cv::Scalar(0, 0, 255),
              2,
              cv::LINE_AA);
        }
      }
      if (cfg.save_overlay_image)
      {
        saveOverlayImage(params, overlay_image);
      }
    }

    ROS_INFO_STREAM_THROTTLE(1.0, "[polygon_hole_step_viz] " << result.summary
                          << " | window_frames=" << result.lidar_frames
                          << " need_frames=" << min_process_frames
                          << " window_points=" << result.lidar_points_raw
                          << " recv_frames_total=" << total_frames_snapshot
                          << " recv_points_total=" << total_points_snapshot
                          << " tf_transformed=" << total_frames_transformed
                          << " tf_dropped=" << total_frames_dropped_tf
                          << " dynamic_roi=" << (dynamic_roi_active ? "active" : "static")
                          << " visual_roi=" << (result.visual_roi_used ? "used" : (visual_roi_active ? "fallback" : "inactive"))
                          << " visual_points=" << result.visual_roi_points
                          << " roi_points=" << result.roi_points
                          << " plane_area=" << result.plane_area
                          << " hole_area=" << result.hole_area
                          << " ratio=" << result.area_ratio
                          << " threshold=" << cfg.pass_area_ratio);

    publishCloud(raw_pub, raw_cloud, cfg.frame_id);
    publishCloud(roi_pub, roi_cloud, cfg.frame_id);
    publishCloud(plane_pub, plane_cloud, cfg.frame_id);
    publishCloud(non_plane_pub, non_plane_cloud, cfg.frame_id);
    publishCloud(far_pub, empty_candidate_cloud, cfg.frame_id);
    publishCloud(empty_pub, empty_candidate_cloud, cfg.frame_id);
    publishCloud(hole_cluster_pub, hole_cluster_cloud, cfg.frame_id);
    publishCloud(plane_poly_pub, plane_polygon_cloud, cfg.frame_id);
    publishCloud(hole_poly_pub, hole_polygon_cloud, cfg.frame_id);
    publishCloud(hole_center_pub, hole_center_cloud, cfg.frame_id);

    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(makeRoiBoxMarker(active_roi, cfg.frame_id, 0, cfg.roi_line_width));
    if (plane_polygon_cloud && !plane_polygon_cloud->empty())
    {
      markers.markers.push_back(makeLineStripMarker(cfg.frame_id, "step_plane_poly", 1, plane_polygon_cloud, 0.0f, 0.9f, 1.0f, cfg.line_width));
    }
    else
    {
      markers.markers.push_back(makeDeleteMarker(cfg.frame_id, "step_plane_poly", 1));
    }
    if (hole_polygon_cloud && !hole_polygon_cloud->empty())
    {
      markers.markers.push_back(makeLineStripMarker(cfg.frame_id, "step_hole_poly", 2, hole_polygon_cloud, 1.0f, 0.5f, 0.0f, cfg.line_width));
    }
    else
    {
      markers.markers.push_back(makeDeleteMarker(cfg.frame_id, "step_hole_poly", 2));
    }

    pcl::PointXYZ center_text_anchor = roiTextAnchor(active_roi);

    if (!hole_center_cloud->empty())
    {
      markers.markers.push_back(makeSphereMarker(cfg.frame_id, "step_hole_center", 3, hole_center_cloud->points.front(), 1.0f, 0.0f, 0.0f, cfg.center_marker_size));
    }
    else
    {
      markers.markers.push_back(makeDeleteMarker(cfg.frame_id, "step_hole_center", 3));
    }

    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(3);
    oss << "frames=" << result.lidar_frames << "/" << max_window_frames
        << " need>=" << min_process_frames
        << " roi=" << result.roi_points
        << " visual=" << (result.visual_roi_used ? "used" : (visual_roi_active ? "fallback" : "off"))
        << " vpts=" << result.visual_roi_points
        << " planeArea=" << result.plane_area
        << " holeArea=" << result.hole_area
        << " ratio=" << result.area_ratio
        << " th=" << cfg.pass_area_ratio
        << " status=" << (result.pass ? "PASS" : "FAIL")
        << " (" << result.summary << ")";
    const float tr = result.pass ? 0.2f : 1.0f;
    const float tg = result.pass ? 1.0f : 0.2f;
    const float tb = 0.2f;
    markers.markers.push_back(makeTextMarker(cfg.frame_id, "step_status", 4, center_text_anchor, oss.str(), 0.12, tr, tg, tb));

    aperture_detector::ApertureObservation observation;
    observation.header.stamp = observation_stamp;
    observation.header.frame_id = cfg.frame_id;
    observation.geometry_valid = result.pass && hole_polygon_cloud->size() >= 4;
    observation.reason = result.summary;
    observation.plane_area = result.plane_area;
    observation.hole_area = result.hole_area;
    observation.area_ratio = result.area_ratio;
    observation.visual_roi_used = result.visual_roi_used;
    // This detector estimates a plane aperture, not wall thickness or free space behind it.
    observation.depth_known = false;
    if (observation.geometry_valid && !hole_center_cloud->empty()) {
      const auto &c = hole_center_cloud->front();
      observation.center.x = c.x; observation.center.y = c.y; observation.center.z = c.z;
      Eigen::Vector3d normal = Eigen::Vector3d::Zero();
      for (size_t i = 0; i < hole_polygon_cloud->size(); ++i) {
        const auto &a = hole_polygon_cloud->points[i];
        const auto &b = hole_polygon_cloud->points[(i + 1) % hole_polygon_cloud->size()];
        geometry_msgs::Point32 point;
        point.x = a.x; point.y = a.y; point.z = a.z;
        observation.boundary.points.push_back(point);
        normal += Eigen::Vector3d(a.x-c.x, a.y-c.y, a.z-c.z).cross(
                  Eigen::Vector3d(b.x-c.x, b.y-c.y, b.z-c.z));
      }
      if (normal.norm() > 1e-6 && normal.allFinite()) {
        normal.normalize();
        observation.normal.x = normal.x(); observation.normal.y = normal.y(); observation.normal.z = normal.z();
      } else {
        observation.geometry_valid = false;
        observation.reason = "invalid polygon normal";
      }
    }
    observation_pub.publish(observation);
    marker_pub.publish(markers);
    if (!overlay_image.empty())
    {
      sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", overlay_image).toImageMsg();
      image_msg->header.stamp = ros::Time::now();
      overlay_image_pub.publish(image_msg);
    }
    if (cfg.save_hole_polygon_txt)
    {
      saveHolePolygonToTxt(
        cfg.hole_polygon_txt_path,
        result,
        cfg.frame_id,
        local_odom,
        hole_polygon_cloud,
        hole_center_cloud);
    }
    if (cfg.publish_planning_snapshot_json)
    {
      std::string snapshot_json;
      std::string reject_reason;
      if (buildPlanningSnapshotJson(
            result,
            cfg,
            local_odom,
            hole_polygon_cloud,
            hole_center_cloud,
            snapshot_json,
            reject_reason))
      {
        std_msgs::String msg;
        msg.data = snapshot_json;
        planning_snapshot_json_pub.publish(msg);
        ROS_INFO_STREAM("[polygon_hole_step_viz] published planning JSON: " << snapshot_json);
      }
      else
      {
        ROS_WARN_STREAM("[polygon_hole_step_viz] planning JSON not published: "
                        << reject_reason << " | detection=" << result.summary);
      }
    }
    rate.sleep();
  }

  return 0;
}
