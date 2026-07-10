#ifndef _PERFECT_DRONE_SIM_HPP_
#define _PERFECT_DRONE_SIM_HPP_

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "nav_msgs/Odometry.h"
#include "tf2_ros/transform_broadcaster.h"
#include "string"
#include "Eigen/Dense"
#include "nav_msgs/Path.h"
#include <sensor_msgs/PointCloud2.h>
#include "marsim_render/marsim_render.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "perfect_drone_sim/config.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <sstream>

typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 3, 3> Mat33;

typedef Eigen::Matrix<double, 3, 3> StatePVA;
typedef Eigen::Matrix<double, 3, 4> StatePVAJ;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DynamicMat;
typedef Eigen::MatrixX4d MatX4;
typedef std::pair<double, Vec3> TimePosPair;

namespace perfect_drone {
    class PerfectDrone {
        marsim::MarsimRender::Ptr render_ptr_;
        Config cfg_;

    public:
        PerfectDrone(const ros::NodeHandle &n) : nh_(n) {
#define CONFIG_FILE_DIR(name) (string(string(ROOT_DIR) + "config/"+name))
            std::string dft_cfg_path = CONFIG_FILE_DIR("click.yaml");
            std::string cfg_path, cfg_name;
            if (nh_.param("config_path", cfg_path, dft_cfg_path)) {
                cout << " -- [Fsm-Test] Load config from: " << cfg_path << endl;
            } else if (nh_.param("config_name", cfg_name, dft_cfg_path)) {
                cfg_path = CONFIG_FILE_DIR(cfg_name);
                cout << " -- [Fsm-Test] Load config by file name: " << cfg_name << endl;
            }
            cfg_ = Config(cfg_path);
            applyInitialPoseOverrides();
            nh_.param("lidar_pitch", cfg_.lidar_pitch, cfg_.lidar_pitch);
            nh_.param("use_command_attitude", use_command_attitude_, false);
            nh_.param("dynamic_cloud_en", cfg_.dynamic_cloud_en, cfg_.dynamic_cloud_en);
            nh_.param("dynamic_cloud_topic", cfg_.dynamic_cloud_topic, cfg_.dynamic_cloud_topic);
            render_ptr_ = std::make_shared<marsim::MarsimRender>(cfg_path);
            cmd_sub_ = nh_.subscribe(cfg_.cmd_topic, 100, &PerfectDrone::cmdCallback, this);
            if (cfg_.dynamic_cloud_en) {
                dynamic_cloud_sub_ = nh_.subscribe(cfg_.dynamic_cloud_topic,
                                                   2,
                                                   &PerfectDrone::dynamicCloudCallback,
                                                   this,
                                                   ros::TransportHints().tcpNoDelay());
                ROS_INFO("PerfectDrone dynamic cloud enabled on topic: %s",
                         cfg_.dynamic_cloud_topic.c_str());
            }
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>(cfg_.odom_topic, 20);
            pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(cfg_.pose_topic, 10);
            robot_pub_ = nh_.advertise<visualization_msgs::Marker>("robot", 1);
            path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1);
            // A planner must consume the newest scan.  Keeping a deep publisher
            // queue here only turns transient overload into seconds of stale
            // world-frame clouds and unbounded memory pressure.
            local_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cfg_.local_pc_topic, 1);
            global_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cfg_.global_pc_topic, 1, true);
            vel_pub_ = nh_.advertise<visualization_msgs::Marker>("vel_text", 1);

            position_ = cfg_.init_pos;
            velocity_.setZero();
            yaw_ = cfg_.init_yaw;
            mesh_resource_ = cfg_.mesh_resource;
            q_ = quatFromRpy(cfg_.init_roll, cfg_.init_pitch, cfg_.init_yaw);
            odom_.header.frame_id = "world";
            odom_.child_frame_id = cfg_.robot_frame_id;
            odom_pub_timer_ = nh_.createTimer(ros::Duration(0.01), &PerfectDrone::publishOdom, this);

            global_pc_pub_timer_ = nh_.createTimer(ros::Duration(0.001), &PerfectDrone::publishGlobalPC, this);
            path_.poses.clear();
            path_.header.frame_id = "world";
            path_.header.stamp = ros::Time::now();
            pattern_epoch_ = path_.header.stamp;
            ROS_INFO("PerfectDrone LiDAR mount: body pitch %.3f deg; cloud and odometry use an identical pose timestamp",
                     cfg_.lidar_pitch);
        }

        double getSensingRate() {
            return cfg_.sensing_rate;
        }

        void visualizeText(const ros::Publisher &pub,
                           const std::string &ns,
                           const std::string &text,
                           const Vec3 &position,
                           const double &size,
                           const int &id
        ) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.ns = ns.c_str();
            if (id >= 0) {
                marker.id = id;
            } else {
                static int id = 0;
                marker.id = id++;
            }
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.scale.z = size;
            marker.color.a = 1.0;
            marker.color.r = 0.1;
            marker.color.g = 0.1;
            marker.color.b = 0.1;
            marker.text = text;
            marker.pose.position.x = position.x();
            marker.pose.position.y = position.y();
            marker.pose.position.z = position.z()+5.0;
            marker.pose.orientation.w = 1.0;
            pub.publish(marker);
        }


        void publishPC() {
            nav_msgs::Odometry odom_snapshot;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                odom_snapshot = odom_;
            }
            if (odom_snapshot.header.stamp.isZero()) {
                ROS_WARN_THROTTLE(1.0, "PerfectDrone is waiting for the first odometry snapshot before rendering");
                return;
            }

            const Vec3 body_position(odom_snapshot.pose.pose.position.x,
                                     odom_snapshot.pose.pose.position.y,
                                     odom_snapshot.pose.pose.position.z);
            Eigen::Quaterniond body_q(odom_snapshot.pose.pose.orientation.w,
                                      odom_snapshot.pose.pose.orientation.x,
                                      odom_snapshot.pose.pose.orientation.y,
                                      odom_snapshot.pose.pose.orientation.z);
            body_q.normalize();
            const Eigen::Quaterniond lidar_mount_q(
                Eigen::AngleAxisd(cfg_.lidar_pitch * M_PI / 180.0, Vec3::UnitY()));
            Eigen::Quaterniond lidar_q = body_q * lidar_mount_q;
            lidar_q.normalize();

            pcl::PointCloud<marsim::PointType>::Ptr local_map(new pcl::PointCloud<marsim::PointType>);
            // Pattern generators use float arithmetic internally.  Absolute ROS
            // epoch seconds lose sub-second precision in a float, which makes
            // the Mid360 scan loop empty.  Match the original simulator and
            // drive the pattern with time elapsed since simulator start.
            const double pattern_time = std::max(
                0.0, (odom_snapshot.header.stamp - pattern_epoch_).toSec());
            {
                std::lock_guard<std::mutex> lock(render_mutex_);
                render_ptr_->renderOnceInWorld(body_position.cast<float>(), lidar_q.cast<float>(),
                                               pattern_time,
                                               local_map);
            }
            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(*local_map, pc_msg);
            pc_msg.header.frame_id = "world";
            // The renderer used exactly this body pose.  Sharing its stamp lets
            // Exact/ApproximateTime pair the cloud with the correct odometry
            // instead of a newer pose produced while the GPU was rendering.
            pc_msg.header.stamp = odom_snapshot.header.stamp;
            ROS_INFO_STREAM_THROTTLE(1.0, "Publish local map size: " << local_map->size());
            local_pc_pub_.publish(pc_msg);
        }

        ~PerfectDrone() {}

    private:
        nav_msgs::Path path_;
        ros::Subscriber cmd_sub_, dynamic_cloud_sub_;
        ros::Publisher odom_pub_, robot_pub_, pose_pub_, path_pub_, global_pc_pub_, local_pc_pub_, vel_pub_;
        ros::Timer odom_pub_timer_;
        ros::Timer pc_pub_timer_;
        ros::Timer global_pc_pub_timer_;
        ros::NodeHandle nh_;
        Vec3 position_, velocity_;
        double yaw_;
        Eigen::Quaterniond q_;
        nav_msgs::Odometry odom_;
        ros::Time pattern_epoch_;
        std::string mesh_resource_;
        bool use_command_attitude_{false};
        std::mutex state_mutex_;
        std::mutex render_mutex_;

        static Eigen::Quaterniond quatFromRpy(const double roll,
                                              const double pitch,
                                              const double yaw) {
            Eigen::Quaterniond q =
                Eigen::AngleAxisd(yaw, Vec3::UnitZ()) *
                Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
                Eigen::AngleAxisd(roll, Vec3::UnitX());
            q.normalize();
            return q;
        }

        void applyInitialPoseOverrides() {
            nh_.param("init_roll", cfg_.init_roll, cfg_.init_roll);
            nh_.param("init_pitch", cfg_.init_pitch, cfg_.init_pitch);
            nh_.param("init_yaw", cfg_.init_yaw, cfg_.init_yaw);

            bool init_from_surface = false;
            nh_.param("init_from_surface", init_from_surface, false);
            if (!init_from_surface) {
                return;
            }

            double surface_x = cfg_.init_pos.x();
            double surface_y = cfg_.init_pos.y();
            double surface_z = cfg_.init_pos.z();
            double surface_roll = cfg_.init_roll;
            double surface_pitch = cfg_.init_pitch;
            double surface_yaw = cfg_.init_yaw;
            double robot_l = 0.28;

            nh_.param("surface_x", surface_x, surface_x);
            nh_.param("surface_y", surface_y, surface_y);
            nh_.param("surface_z", surface_z, surface_z);
            nh_.param("surface_roll", surface_roll, surface_roll);
            nh_.param("surface_pitch", surface_pitch, surface_pitch);
            nh_.param("surface_yaw", surface_yaw, surface_yaw);
            nh_.param("robot_l", robot_l, robot_l);
            nh_.param("takeoff_contact_robot_l", robot_l, robot_l);

            const Eigen::Quaterniond surface_q = quatFromRpy(surface_roll, surface_pitch, surface_yaw);
            const Vec3 surface_p(surface_x, surface_y, surface_z);
            cfg_.init_pos = surface_p + std::max(0.0, robot_l) * surface_q.toRotationMatrix().col(2);
            cfg_.init_roll = surface_roll;
            cfg_.init_pitch = surface_pitch;
            cfg_.init_yaw = surface_yaw;

            ROS_INFO("PerfectDrone initial pose follows surface contact: p=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f), robot_l=%.3f",
                     cfg_.init_pos.x(), cfg_.init_pos.y(), cfg_.init_pos.z(),
                     cfg_.init_roll, cfg_.init_pitch, cfg_.init_yaw, robot_l);
        }

        void cmdCallback(const quadrotor_msgs::PositionCommandConstPtr &msg) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            Vec3 pos(msg->position.x, msg->position.y, msg->position.z);
            Vec3 vel(msg->velocity.x, msg->velocity.y, msg->velocity.z);
            Vec3 acc(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
            double yaw = msg->yaw;
            updateFlatness(pos, vel, acc, yaw);
            if (use_command_attitude_ &&
                std::isfinite(msg->attitude.x) &&
                std::isfinite(msg->attitude.y) &&
                std::isfinite(msg->attitude.z)) {
                q_ = quatFromRpy(msg->attitude.x, msg->attitude.y, msg->attitude.z);
                yaw_ = msg->attitude.z;
            }
        }


        void dynamicCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
            pcl::PointCloud<pcl::PointXYZI> dynamic_cloud;
            pcl::fromROSMsg(*msg, dynamic_cloud);
            std::lock_guard<std::mutex> lock(render_mutex_);
            render_ptr_->input_dyn_clouds(std::move(dynamic_cloud));
        }


        void publishGlobalPC(const ros::TimerEvent &e) {
            static int last_sub_num = 0;
            // update sub num
            int sub_num = global_pc_pub_.getNumSubscribers();
            if (sub_num > 0 && last_sub_num != sub_num) {
                ros::Duration(0.5).sleep();
                pcl::PointCloud<marsim::PointType>::Ptr global_map(new pcl::PointCloud<marsim::PointType>);
                render_ptr_->getGlobalMap(global_map);
                sensor_msgs::PointCloud2 pc_msg;
                pcl::toROSMsg(*global_map, pc_msg);
                pc_msg.header.frame_id = "world";
                pc_msg.header.stamp = ros::Time::now();
                global_pc_pub_.publish(pc_msg);
                std::cout << "Publish global map" << std::endl;
            }
            last_sub_num = sub_num;
        }

        void publishOdom(const ros::TimerEvent &e) {
            nav_msgs::Odometry odom_snapshot;
            Vec3 position_snapshot;
            double speed = 0.0;
            {
                // cmdCallback is serviced by an AsyncSpinner.  Atomically cache
                // one complete body state so both this odometry message and the
                // next rendered cloud refer to a physically possible pose.
                std::lock_guard<std::mutex> lock(state_mutex_);
                odom_.pose.pose.position.x = position_.x();
                odom_.pose.pose.position.y = position_.y();
                odom_.pose.pose.position.z = position_.z();

                odom_.pose.pose.orientation.x = q_.x();
                odom_.pose.pose.orientation.y = q_.y();
                odom_.pose.pose.orientation.z = q_.z();
                odom_.pose.pose.orientation.w = q_.w();

                odom_.twist.twist.linear.x = velocity_.x();
                odom_.twist.twist.linear.y = velocity_.y();
                odom_.twist.twist.linear.z = velocity_.z();
                odom_.header.stamp = ros::Time::now();

                odom_snapshot = odom_;
                position_snapshot = position_;
                speed = velocity_.norm();
            }

            std::ostringstream oss;
            oss << std::fixed << std::setprecision(3) << speed;
            visualizeText(vel_pub_, "vel",
                          "Speed: " + oss.str() + " m/s",
                          position_snapshot, 3.0, 0);

            odom_pub_.publish(odom_snapshot);

            geometry_msgs::PoseStamped pose;
            pose.pose = odom_snapshot.pose.pose;
            pose.header = odom_snapshot.header;
            pose_pub_.publish(pose);

            static tf2_ros::TransformBroadcaster br_map_ego;
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = odom_snapshot.header.stamp;
            transformStamped.header.frame_id = "world";
            transformStamped.child_frame_id = cfg_.robot_frame_id;
            transformStamped.transform.translation.x = odom_snapshot.pose.pose.position.x;
            transformStamped.transform.translation.y = odom_snapshot.pose.pose.position.y;
            transformStamped.transform.translation.z = odom_snapshot.pose.pose.position.z;
            transformStamped.transform.rotation.x = odom_snapshot.pose.pose.orientation.x;
            transformStamped.transform.rotation.y = odom_snapshot.pose.pose.orientation.y;
            transformStamped.transform.rotation.z = odom_snapshot.pose.pose.orientation.z;
            transformStamped.transform.rotation.w = odom_snapshot.pose.pose.orientation.w;
            br_map_ego.sendTransform(transformStamped);

            visualization_msgs::Marker meshROS;
            meshROS.header.frame_id = "world";
            meshROS.header.stamp = odom_snapshot.header.stamp;
            meshROS.ns = "mesh";
            meshROS.id = 0;
            meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
            meshROS.action = visualization_msgs::Marker::ADD;
            meshROS.pose.position = odom_snapshot.pose.pose.position;
            meshROS.pose.orientation = odom_snapshot.pose.pose.orientation;
            meshROS.scale.x = 1;
            meshROS.scale.y = 1;
            meshROS.scale.z = 1;
            meshROS.mesh_resource = mesh_resource_;
            meshROS.mesh_use_embedded_materials = true;
            meshROS.color.a = 1.0;
            meshROS.color.r = 1.0;
            meshROS.color.g = 1.0;
            meshROS.color.b = 1.0;
            robot_pub_.publish(meshROS);
            static int slow_down = 0;
            if (slow_down++ % 10 == 0) {
                if ((position_snapshot.head(2) - Vec3(0, -50, 1.5).head(2)).norm() < 1) {
                    path_.poses.clear();
                    path_.poses.reserve(10000);
                }
                path_.poses.push_back(pose);
                path_.header = odom_snapshot.header;
                path_pub_.publish(path_);
            }
        }

        void updateFlatness(const Vec3 &pos, const Vec3 &vel,
                            const Vec3 &acc, const double yaw) {
            Vec3 gravity_ = 9.80 * Eigen::Vector3d(0, 0, 1);
            position_ = pos;
            velocity_ = vel;
            double a_T = (gravity_ + acc).norm();
            Eigen::Vector3d xB, yB, zB;
            Eigen::Vector3d xC(cos(yaw), sin(yaw), 0);

            zB = (gravity_ + acc).normalized();
            yB = ((zB).cross(xC)).normalized();
            xB = yB.cross(zB);
            Eigen::Matrix3d R;
            R << xB, yB, zB;
            q_ = Eigen::Quaterniond(R);
        }
    };
}


#endif
