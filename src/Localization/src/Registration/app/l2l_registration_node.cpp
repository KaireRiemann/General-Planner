#include <iostream>
#include <fstream>
#include <sstream>
#include <random>
#include <chrono>
#include <string>

#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/Eigenvalues>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>

#include "utils/registration.h"

using namespace pcl;
using namespace std;
using namespace ICPAlgorithm;

typedef pcl::PointXYZI Point_T;

bool g_is_other_map_received = false;
bool g_is_my_map_received = false;
bool g_is_use_pcd = true;
pcl::PointCloud<Point_T>::Ptr g_other_map_ptr, g_my_map_ptr;
string g_icp_pattern, g_other_map_fn;
std::vector<float> g_init_guess;
std::vector<float> g_map_range;
Eigen::Matrix4f g_HT_init_guess = Eigen::Matrix4f::Identity();
Eigen::Matrix4f g_HT = Eigen::Matrix4f::Identity();
Eigen::Matrix4f g_HT_body_wrt_w0 = Eigen::Matrix4f::Identity();

int g_icp_iter, g_ransac_iter, g_neibor_k;
float g_max_cor_dis;
float g_trigger_time;
float g_resolution;
nav_msgs::Odometry modify_odom, local_odom, old_odom;
sensor_msgs::PointCloud2 modify_pc, old_pc;
sensor_msgs::PointCloud2 other_map_msg;

ros::Subscriber other_map_sub, my_map_sub, lidar_odom_sub, lidar_map_sub;
ros::Publisher other_map_pub, modify_odom_pub, modify_my_map_pub,\
              local_odom_pub,old_odom_pub,old_my_map_pub, reg_trigger_pub;

Eigen::Quaterniond euler2quaternion(Eigen::Vector3d euler)
{
  double cr = cos(euler(0)/2);
  double sr = sin(euler(0)/2);
  double cp = cos(euler(1)/2);
  double sp = sin(euler(1)/2);
  double cy = cos(euler(2)/2);
  double sy = sin(euler(2)/2);
  Eigen::Quaterniond q;
  q.w() = cr*cp*cy + sr*sp*sy;
  q.x() = sr*cp*cy - cr*sp*sy;
  q.y() = cr*sp*cy + sr*cp*sy;
  q.z() = cr*cp*sy - sr*sp*cy;
  return q; 
}

Eigen::Matrix4d odom2Matrix4d(const nav_msgs::Odometry &odom) {
  Eigen::Matrix4d HT = Eigen::Matrix4d::Identity();
  Eigen::Vector3d p(odom.pose.pose.position.x,\
                    odom.pose.pose.position.y,\
                    odom.pose.pose.position.z);

  Eigen::Quaterniond q(odom.pose.pose.orientation.w,\
                       odom.pose.pose.orientation.x,\
                       odom.pose.pose.orientation.y,\
                       odom.pose.pose.orientation.z);
  HT.block<3, 1>(0, 3) = p;
  HT.block<3, 3>(0, 0) = q.toRotationMatrix();
  return HT;
}

nav_msgs::Odometry Matrix4d2odom(const Eigen::Matrix4d &HT) {
  nav_msgs::Odometry odom;
  Eigen::Vector3d p = HT.block<3, 1>(0, 3);
  Eigen::Quaterniond q(HT.block<3, 3>(0, 0));
  odom.pose.pose.position.x = p[0];
  odom.pose.pose.position.y = p[1];
  odom.pose.pose.position.z = p[2];
  odom.pose.pose.orientation.w = q.w();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  
  return odom;
}

Eigen::Matrix4f
run(PointCloud<PointXYZI>::Ptr src, PointCloud<PointXYZI>::Ptr tar, Eigen::Matrix4f guess, ICPFunPtr icp, std::vector<double> &ret)
{
  std::vector<double> params;
  pcl::Correspondences cor;
  params.push_back(g_neibor_k);
  params.push_back(g_max_cor_dis);
  params.push_back(g_icp_iter);
  params.push_back(g_ransac_iter);   

  return icp(src, tar, guess, ret, cor, params);  
}

void other_map_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
  ROS_WARN("[lidar2lidar_registration_node] other_map_cb");
  msg->header.frame_id = "world";
  pcl::fromROSMsg(*msg, *g_other_map_ptr);
  other_map_msg = *msg;

  if (g_my_map_ptr->size() > 0 && g_other_map_ptr->size() > 0) {
    std::vector<double> ret;        
    g_HT = run(g_my_map_ptr, g_other_map_ptr, g_HT_init_guess, icp_map[g_icp_pattern], ret);
    local_odom = Matrix4d2odom(g_HT.cast<double>());
    local_odom.header.frame_id = "world";    
    std::cout << "rel_p = " << g_HT.block<3,1>(0,3).transpose() << std::endl;
    std::cout << "rel_p = " << local_odom.pose.pose.position.x << "\t" \
                            << local_odom.pose.pose.position.y << "\t" \
                            << local_odom.pose.pose.position.z << std::endl;
    ROS_WARN("[l2l_registration_node] send local_ref to fastlio!!!");
    local_odom_pub.publish(local_odom);
  }
}


void my_map_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
  ROS_WARN("[l2l_registration_node] receive my_map !!!");
  msg->header.frame_id = "world";

  pcl::fromROSMsg(*msg, *g_my_map_ptr);

  if (g_my_map_ptr->size() > 0 && g_other_map_ptr->size() > 0) {
    std::vector<double> ret;            
    g_HT = run(g_my_map_ptr, g_other_map_ptr, g_HT_init_guess, icp_map[g_icp_pattern], ret);
    local_odom = Matrix4d2odom(g_HT.cast<double>());
    local_odom.header.frame_id = "world";
    local_odom_pub.publish(local_odom);
    ROS_WARN("[l2l_registration_node] send local_ref to fastlio!!!");
    std::cout << "rel_p = " << g_HT.block<3,1>(0,3).transpose() << std::endl;
    std::cout << "rel_p = " << local_odom.pose.pose.position.x << "\t" \
                            << local_odom.pose.pose.position.y << "\t" \
                            << local_odom.pose.pose.position.z << std::endl;    
  }
}

void lidar_odom_cb(const nav_msgs::OdometryPtr &msg) {
  old_odom = *msg;
  old_odom.header.frame_id = "world";

  Eigen::Matrix4d HT_body_wrt_local = odom2Matrix4d(*msg);
  Eigen::Matrix4d HT_body_wrt_global = g_HT.cast<double>() * HT_body_wrt_local;

  modify_odom = Matrix4d2odom(HT_body_wrt_global);
  modify_odom.header.frame_id = "world";

  modify_odom_pub.publish(modify_odom);

}

void lidar_pc_cb(const sensor_msgs::PointCloud2Ptr &msg) {
  old_pc = *msg;
  old_pc.header.frame_id = "world";

  pcl::PointCloud<Point_T>::Ptr lidar_pc_local(new pcl::PointCloud<Point_T>);
  pcl::PointCloud<Point_T>::Ptr lidar_pc_global(new pcl::PointCloud<Point_T>);  
  pcl::fromROSMsg(*msg, *lidar_pc_local);
  pcl::transformPointCloud(*lidar_pc_local, *lidar_pc_global, g_HT);
  pcl::toROSMsg(*lidar_pc_global, modify_pc);
  modify_pc.header.frame_id = "world";

  modify_my_map_pub.publish(modify_pc);
}


void pcl_downSampling(pcl::PointCloud<Point_T>::Ptr &cloud_ds) 
{
  pcl::PointCloud<Point_T>::Ptr cloud_tmp(new pcl::PointCloud<Point_T>);

  std::cout << "before downsampling: " << cloud_ds->size() << std::endl;

  // 创建VoxelGrid滤波器对象
  pcl::VoxelGrid<Point_T> sor;
  sor.setInputCloud(cloud_ds);  // 设置输入点云
  sor.setLeafSize(g_resolution, g_resolution, g_resolution);  // 设置体素的大小（降采样的分辨率）

  // 执行降采样滤波
  sor.filter(*cloud_tmp);

  *cloud_ds = *cloud_tmp;
  //
  std::cout << "after downsampling: " << cloud_ds->size() << std::endl;
}

void cut_mapxyz(pcl::PointCloud<Point_T>::Ptr &cloud, std::vector<float> size) 
{
  pcl::ConditionAnd<Point_T>::Ptr range_cond(new pcl::ConditionAnd<Point_T>);//实例化条件指针

  range_cond->addComparison(pcl::FieldComparison<Point_T>::ConstPtr (new pcl::FieldComparison<Point_T>("x", pcl::ComparisonOps::GT,size[0])));
  range_cond->addComparison(pcl::FieldComparison<Point_T>::ConstPtr (new pcl::FieldComparison<Point_T>("x", pcl::ComparisonOps::LT,size[1])));

  range_cond->addComparison(pcl::FieldComparison<Point_T>::ConstPtr (new pcl::FieldComparison<Point_T>("y", pcl::ComparisonOps::GT,size[2])));
  range_cond->addComparison(pcl::FieldComparison<Point_T>::ConstPtr (new pcl::FieldComparison<Point_T>("y", pcl::ComparisonOps::LT,size[3])));

  range_cond->addComparison(pcl::FieldComparison<Point_T>::ConstPtr (new pcl::FieldComparison<Point_T>("z", pcl::ComparisonOps::GT,size[4])));
  range_cond->addComparison(pcl::FieldComparison<Point_T>::ConstPtr (new pcl::FieldComparison<Point_T>("z", pcl::ComparisonOps::LT,size[5])));

  //build the filter
  pcl::ConditionalRemoval<Point_T> condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(cloud);
  condrem.setKeepOrganized(false);//保存原有点云结结构就是点的数目没有减少，采用nan代替了
  condrem.filter(*cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar2lidar_registration_node");
    ros::NodeHandle nh("~");
    g_other_map_ptr.reset(new pcl::PointCloud<Point_T>);
    g_my_map_ptr.reset(new pcl::PointCloud<Point_T>);

    // step 1: load parameters 
    nh.getParam("/regsitraion/init_p", g_init_guess);
    nh.getParam("/regsitraion/map_range", g_map_range);    
    nh.getParam("icp_pattern", g_icp_pattern);
    nh.getParam("g_neibor_k", g_neibor_k);
    nh.getParam("icp_iter", g_icp_iter);
    nh.getParam("ransac_iter", g_ransac_iter);
    nh.getParam("max_cor_dis", g_max_cor_dis);
    nh.getParam("is_use_pcd", g_is_use_pcd);
    nh.getParam("other_map_fn", g_other_map_fn);
    nh.getParam("trigger_time", g_trigger_time);
    nh.getParam("resolution", g_resolution);

    g_init_guess[3] = g_init_guess[3] * M_PI / 180;
    g_init_guess[4] = g_init_guess[4] * M_PI / 180;
    g_init_guess[5] = g_init_guess[5] * M_PI / 180;

    std::cout << "is_use_pcd = " << g_is_use_pcd << std::endl;

    std::cout << "init_x = " << g_init_guess[0] << std::endl;
    std::cout << "init_y = " << g_init_guess[1] << std::endl;
    std::cout << "init_z = " << g_init_guess[2] << std::endl;
    std::cout << "init_roll = " << g_init_guess[3] << std::endl;
    std::cout << "init_pitch = " << g_init_guess[4] << std::endl;
    std::cout << "init_yaw = " << g_init_guess[5] << std::endl;

    Eigen::Vector3d eular_init_guess;
    eular_init_guess = Eigen::Vector3d(g_init_guess[3], g_init_guess[4], g_init_guess[5]);
    Eigen::Quaternionf q_init_guess = euler2quaternion(eular_init_guess).cast<float>();

    g_HT_init_guess.block<3,1>(0,3) = Eigen::Vector3f(g_init_guess[0], g_init_guess[1], g_init_guess[2]);
    g_HT_init_guess.block<3,3>(0,0) = q_init_guess.toRotationMatrix();

    g_HT = g_HT_init_guess;

    // step 2: register subscriber and publisher

    reg_trigger_pub = nh.advertise<std_msgs::Empty>("trigger", 1);

    // other_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/other_map_test", 1);
    local_odom_pub = nh.advertise<nav_msgs::Odometry>("/local_ref",1);

    // modify_odom_pub = nh.advertise<nav_msgs::Odometry>("Odometry_new",1);
    // modify_my_map_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_new",1);
    
    if (!g_is_use_pcd) 
    {
      other_map_sub = nh.subscribe("ref_map", 1, other_map_cb, ros::TransportHints().tcpNoDelay());     
    } 
    else 
    {
      if (pcl::io::loadPCDFile<Point_T>(g_other_map_fn, *g_other_map_ptr) == -1)
      {
          ROS_ERROR("other map pcd load fail!!!");
          return 0;
      }
      pcl_downSampling(g_other_map_ptr);

      cut_mapxyz(g_other_map_ptr, g_map_range);
            
      pcl::toROSMsg(*g_other_map_ptr, other_map_msg);
      other_map_msg.header.frame_id = "world";
      g_is_other_map_received = true;
    }

    my_map_sub = nh.subscribe("my_map", 1, my_map_cb, ros::TransportHints().tcpNoDelay());
    // lidar_odom_sub = nh.subscribe("Odometry_old", 10, lidar_odom_cb, ros::TransportHints().tcpNoDelay());
    // lidar_map_sub = nh.subscribe("cloud_old", 10, lidar_pc_cb, ros::TransportHints().tcpNoDelay());

    ros::Rate rate(10);

    ros::Time start_time = ros::Time::now();
    bool trigger_flag = false;
    
    while(ros::ok()) 
    {
      if ((ros::Time::now() - start_time).toSec() > g_trigger_time && !trigger_flag) 
      {
        std_msgs::Empty msg;
        reg_trigger_pub.publish(msg);
        ROS_WARN("[lidar2lidar_registration_node]trigger!!!");
        trigger_flag = true;
      }
      // other_map_pub.publish(other_map_msg);
      rate.sleep();
      ros::spinOnce();
    }
    return 0;
}