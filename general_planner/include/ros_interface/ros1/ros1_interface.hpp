/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/


#ifdef USE_ROS1
#ifndef SRC_ROS1_VISUALIZER_HPP
#define SRC_ROS1_VISUALIZER_HPP

#include "ros_interface/ros1/ros1_adapter.hpp"


namespace ros_interface {

    class Ros1Interface : public RosInterface {
    public:

        explicit Ros1Interface(const ros::NodeHandle &nh)
                : nh_(nh) {

            /*=============================FOR Planner========================================*/
            goal_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/goal", 100);

            exp_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/exp_traj", 100);
            backup_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/backup_traj", 100);
            committed_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/committed_traj", 100);

            exp_sfcs_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/exp_sfc", 100);
            backup_sfc_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/backup_sfc", 100);

            guide_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/frontend_path", 100);

            yaw_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/yaw_traj", 100);
            fov_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/tracking_fov", 100);
            exploration_frontier_pub_ =
                    nh_.advertise<visualization_msgs::MarkerArray>("visualization/exploration/frontiers", 10);
            exploration_topo_pub_ =
                    nh_.advertise<visualization_msgs::MarkerArray>("visualization/exploration/topo_graph", 10);
            exploration_viewpoint_pub_ =
                    nh_.advertise<visualization_msgs::MarkerArray>("visualization/exploration/viewpoints", 10);
            exploration_global_tour_pub_ =
                    nh_.advertise<visualization_msgs::MarkerArray>("visualization/exploration/global_tour", 10);

            /*=============================FOR A* debug ========================================*/
            astar_mkr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/astar_debug", 100);

            /*=============================FOR A* debug ========================================*/
            ciri_mkr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/ciri_debug_mkr", 100);
            ciri_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("visualization/ciri_debug_pc", 100);
            /*=============================FOR replan log ========================================*/
            replan_log_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("visualization/replan_log_pc", 100);
            replan_log_mkr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/replan_log_mkr", 100);
        }


        /*=============================FOR ROS logger ========================================*/
        void debug(const std::string& msg) override { ROS_DEBUG("%s", msg.c_str()); }
        void info(const std::string& msg) override { ROS_INFO("%s", msg.c_str()); }
        void warn(const std::string& msg) override { ROS_WARN("%s", msg.c_str()); }
        void error(const std::string& msg) override { ROS_ERROR("%s", msg.c_str()); }
        void fatal(const std::string& msg) override { ROS_FATAL("%s", msg.c_str()); }

        double getSimTime() override {
            return ros::Time::now().toSec();
        }

        void getSimTime(int32_t &sec, uint32_t &nsec) override{
            ros::Time now = ros::Time::now();
            sec = now.sec;
            nsec = now.nsec;
        }

        void setSimTime(const double &sim_time) override {
            ros::Time::setNow(ros::Time(sim_time));
        }

        /*=============================FOR Planner========================================*/
        void vizExpTraj(const Trajectory &traj, const std::string & ns="exp_traj") override {
            if (!visualization_en_) {
                return;
            }
            if (exp_traj_pub_.getNumSubscribers() <= 0) {
                return;
            }
            if(traj.empty()){
                return;
            }
            Ros1Adapter::deleteAllMarkerArray(exp_traj_pub_);
            visualization_msgs::MarkerArray mkr_arr;
            Ros1Adapter::addTrajectoryToMarkerArray(mkr_arr, traj, ns, Color::Green(), 0.08, true, true);
            exp_traj_pub_.publish(mkr_arr);
        }

        void vizBackupTraj(const Trajectory &traj) {
            if (!visualization_en_) {
                return;
            }
            if (backup_traj_pub_.getNumSubscribers() <= 0) {
                return;
            }
            if(traj.empty()){
                return;
            }
            Ros1Adapter::deleteAllMarkerArray(backup_traj_pub_);
            visualization_msgs::MarkerArray mkr_arr;
            Ros1Adapter::addTrajectoryToMarkerArray(mkr_arr, traj, "backup_traj", Color::Green(), 0.08, true, false);
            Ros1Adapter::addPointToMarkerArray(mkr_arr,traj.getPos(0),  Color::Gray(),"backup_traj_start", 0.31);
            backup_traj_pub_.publish(mkr_arr);
        }

        void vizFrontendPath(const super_utils::vec_Vec3f &path) override {
            if (!visualization_en_) {
                return;
            }
            if(path.empty()){
                return;
            }
            Ros1Adapter::deleteAllMarkerArray(guide_path_pub_);
            visualization_msgs::MarkerArray mkr_arr;
            Ros1Adapter::addPathToMarkerArray(mkr_arr, path, Color::Pink(), "guide_path", 0.1, 0.05);
            guide_path_pub_.publish(mkr_arr);
        }

        void vizExpSfc(const PolytopeVec &sfcs) override {
            if (!visualization_en_) {
                return;
            }
            if (exp_sfcs_pub_.getNumSubscribers() <= 0) {
                return;
            }
            if(sfcs.empty()){
                return;
            }
            Ros1Adapter::deleteAllMarkerArray(exp_sfcs_pub_);
            visualization_msgs::MarkerArray mkr_arr;
            int color_num = sfcs.size();
            int color_id = 0;
            for (auto p: sfcs) {
                double color_ratio = 1.0 - (double) color_id / color_num;
                Vec3f color_mag = tinycolormap::GetColor(color_ratio, tinycolormap::ColormapType::Jet).ConvertToEigen();
                color_id++;
                Color c(color_mag[0], color_mag[1], color_mag[2]);
                Ros1Adapter::addPolytopeToMarkerArray(mkr_arr, p,
                                                      "exp_sfc", false,
                                                      Color::SteelBlue(), c,
                                                      Color::Orange(), 0.15,
                                                      resolution_ / 2);
            }
            exp_sfcs_pub_.publish(mkr_arr);
        }

        void vizBackupSfc(const Polytope &sfc) override {
            if (!visualization_en_) {
                return;
            }
            if (backup_sfc_pub_.getNumSubscribers() <= 0) {
                return;
            }
            Ros1Adapter::deleteAllMarkerArray(backup_sfc_pub_);
            visualization_msgs::MarkerArray mkr_arr;
            Ros1Adapter::addPolytopeToMarkerArray(mkr_arr, sfc, "backup_sfc", false, Color::Chartreuse(),
                                                  Color::Green(),
                                                  Color::Green(),
                                                  0.15,
                                                  resolution_ / 2);
            backup_sfc_pub_.publish(mkr_arr);
        }

        void vizGoalPath(const super_utils::vec_Vec3f &path) override {
            if (!visualization_en_) {
                return;
            }

            if (goal_pub_.getNumSubscribers() <= 0) {
                return;
            }

            if(path.empty()){
                return;
            }

            Ros1Adapter::deleteAllMarkerArray(goal_pub_);

            visualization_msgs::MarkerArray mkr_arr;
            Ros1Adapter::addPathToMarkerArray(mkr_arr, path, Color::Yellow(), "goal", 0.3, 0.15);
            goal_pub_.publish(mkr_arr);
        }

        void vizCommittedTraj(const geometry_utils::Trajectory &committed_traj,
                              const double &backup_traj_start_TT) override {
            if (!visualization_en_) {
                return;
            }
            if (committed_traj_pub_.getNumSubscribers() <= 0) {
                return;
            }

            if(committed_traj.empty()){
                return;
            }

            Ros1Adapter::deleteAllMarkerArray(committed_traj_pub_);

            visualization_msgs::MarkerArray mkr_arr;

            double traj_dur = committed_traj.getTotalDuration();

            if (backup_traj_start_TT > 0 && backup_traj_start_TT < traj_dur) {
                Trajectory exp_traj;
                if (!committed_traj.getPartialTrajectoryByTime(0, backup_traj_start_TT, exp_traj)) {
                    ROS_ERROR("Failed to get partial trajectory");
                    return;
                }
                Trajectory backup_traj;
                if (!committed_traj.getPartialTrajectoryByTime(backup_traj_start_TT, traj_dur, backup_traj)) {
                    ROS_ERROR("Failed to get partial trajectory");
                    return;
                }
                visualization_msgs::MarkerArray mkr_arr;
                Ros1Adapter::addTrajectoryToMarkerArray(mkr_arr, exp_traj, "committed_exp", Color::SteelBlue(), 0.08,
                                                        true, false);
                Ros1Adapter::addTrajectoryToMarkerArray(mkr_arr, backup_traj, "committed_backup", Color::Green(), 0.1,
                                                        false, false);
                committed_traj_pub_.publish(mkr_arr);
            } else {
                visualization_msgs::MarkerArray mkr_arr;
                Ros1Adapter::addTrajectoryToMarkerArray(mkr_arr, committed_traj, "committed_exp", Color::Green(), 0.1,
                                                        true, false);
                committed_traj_pub_.publish(mkr_arr);
            }

            committed_traj_pub_.publish(mkr_arr);
        }

        void vizYawTraj(const Trajectory &pos_traj, const Trajectory &yaw_traj) override {
            if (!visualization_en_ || pos_traj.empty() || yaw_traj.empty()) {
                return;
            }

            if (yaw_traj_pub_.getNumSubscribers() <= 0) {
                return;
            }

            if(pos_traj.empty() || yaw_traj.empty()){
                return;
            }

            Ros1Adapter::deleteAllMarkerArray(yaw_traj_pub_);
            visualization_msgs::MarkerArray mkr_arr;
            Ros1Adapter::addYawTrajectoryToMarkerArray(mkr_arr, pos_traj, yaw_traj);
            yaw_traj_pub_.publish(mkr_arr);
        }

        void vizTrackingFov(const Trajectory &pos_traj,
                            const Trajectory &yaw_traj,
                            const double &horizontal_fov_deg,
                            const double &vertical_fov_deg,
                            const double &range) override {
            if (!visualization_en_ || pos_traj.empty() || yaw_traj.empty()) {
                return;
            }
            if (fov_pub_.getNumSubscribers() <= 0) {
                return;
            }

            visualization_msgs::MarkerArray mkr_arr;
            Ros1Adapter::addTrackingFovToMarkerArray(mkr_arr,
                                                     pos_traj,
                                                     yaw_traj,
                                                     horizontal_fov_deg,
                                                     vertical_fov_deg,
                                                     range);
            fov_pub_.publish(mkr_arr);
        }

        void vizExplorationFrontierClusters(
                const std::vector<general_planner::exploration::FrontierRecord> &frontiers) override {
            if (!visualization_en_ || exploration_frontier_pub_.getNumSubscribers() <= 0) {
                return;
            }
            Ros1Adapter::deleteAllMarkerArray(exploration_frontier_pub_);
            if (frontiers.empty()) {
                return;
            }

            visualization_msgs::MarkerArray mkr_arr;
            int marker_id = 0;
            for (std::size_t i = 0; i < frontiers.size(); ++i) {
                const auto &frontier = frontiers[i];
                const double ratio = frontiers.size() > 1U
                                     ? static_cast<double>(i) / static_cast<double>(frontiers.size() - 1U)
                                     : 0.0;
                const Vec3f color_mag =
                        tinycolormap::GetColor(ratio, tinycolormap::ColormapType::Turbo).ConvertToEigen();
                const Color color(color_mag.x(), color_mag.y(), color_mag.z(), 0.85);

                visualization_msgs::Marker center;
                center.header.frame_id = DEFAULT_FRAME_ID;
                center.header.stamp = ros::Time::now();
                center.ns = "exploration_frontier_center";
                center.id = marker_id++;
                center.action = visualization_msgs::Marker::ADD;
                center.pose.orientation.w = 1.0;
                center.type = visualization_msgs::Marker::SPHERE;
                center.scale.x = 0.35;
                center.scale.y = 0.35;
                center.scale.z = 0.35;
                center.color = color;
                center.pose.position.x = frontier.center.x();
                center.pose.position.y = frontier.center.y();
                center.pose.position.z = frontier.center.z();
                mkr_arr.markers.emplace_back(center);

                if (frontier.normal.squaredNorm() > 1.0e-6) {
                    visualization_msgs::Marker normal;
                    normal.header.frame_id = DEFAULT_FRAME_ID;
                    normal.header.stamp = ros::Time::now();
                    normal.ns = "exploration_frontier_normal";
                    normal.id = marker_id++;
                    normal.action = visualization_msgs::Marker::ADD;
                    normal.pose.orientation.w = 1.0;
                    normal.type = visualization_msgs::Marker::LINE_LIST;
                    normal.scale.x = 0.045;
                    normal.color = color;
                    normal.color.a = 0.9;
                    geometry_msgs::Point p0;
                    p0.x = frontier.center.x();
                    p0.y = frontier.center.y();
                    p0.z = frontier.center.z();
                    const Vec3f normal_tip = frontier.center + frontier.normal.normalized() * 0.8;
                    geometry_msgs::Point p1;
                    p1.x = normal_tip.x();
                    p1.y = normal_tip.y();
                    p1.z = normal_tip.z();
                    normal.points.emplace_back(p0);
                    normal.points.emplace_back(p1);
                    mkr_arr.markers.emplace_back(normal);
                }

                if (!frontier.cells.empty()) {
                    visualization_msgs::Marker cells;
                    cells.header.frame_id = DEFAULT_FRAME_ID;
                    cells.header.stamp = ros::Time::now();
                    cells.ns = "exploration_frontier_cells";
                    cells.id = marker_id++;
                    cells.action = visualization_msgs::Marker::ADD;
                    cells.pose.orientation.w = 1.0;
                    cells.type = visualization_msgs::Marker::CUBE_LIST;
                    cells.scale.x = std::max(0.05, resolution_);
                    cells.scale.y = std::max(0.05, resolution_);
                    cells.scale.z = std::max(0.05, resolution_);
                    cells.color = Color(color, 0.35);
                    const std::size_t stride =
                            std::max<std::size_t>(1U, frontier.cells.size() / 160U);
                    for (std::size_t j = 0; j < frontier.cells.size(); j += stride) {
                        geometry_msgs::Point p;
                        p.x = frontier.cells[j].x();
                        p.y = frontier.cells[j].y();
                        p.z = frontier.cells[j].z();
                        cells.points.emplace_back(p);
                    }
                    mkr_arr.markers.emplace_back(cells);
                }
            }
            exploration_frontier_pub_.publish(mkr_arr);
        }

        void vizExplorationTopoGraph(
                const std::vector<general_planner::exploration::ExplorationTopoNode> &nodes,
                const std::vector<general_planner::exploration::ExplorationTopoEdge> &edges) override {
            if (!visualization_en_ || exploration_topo_pub_.getNumSubscribers() <= 0) {
                return;
            }
            Ros1Adapter::deleteAllMarkerArray(exploration_topo_pub_);
            if (nodes.empty() && edges.empty()) {
                return;
            }

            auto toPoint = [](const Vec3f &p) {
                geometry_msgs::Point out;
                out.x = p.x();
                out.y = p.y();
                out.z = p.z();
                return out;
            };

            visualization_msgs::MarkerArray mkr_arr;
            int marker_id = 0;
            auto makeNodeMarker = [&](const std::string &ns,
                                      const Color &color,
                                      const double scale) {
                visualization_msgs::Marker marker;
                marker.header.frame_id = DEFAULT_FRAME_ID;
                marker.header.stamp = ros::Time::now();
                marker.ns = ns;
                marker.id = marker_id++;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.orientation.w = 1.0;
                marker.type = visualization_msgs::Marker::SPHERE_LIST;
                marker.scale.x = scale;
                marker.scale.y = scale;
                marker.scale.z = scale;
                marker.color = color;
                return marker;
            };
            visualization_msgs::Marker odom_nodes =
                    makeNodeMarker("exploration_topo_odom", Color::Yellow(), 0.34);
            visualization_msgs::Marker history_nodes =
                    makeNodeMarker("exploration_topo_history", Color::SteelBlue(), 0.18);
            visualization_msgs::Marker region_nodes =
                    makeNodeMarker("exploration_topo_region", Color::Teal(), 0.20);
            visualization_msgs::Marker frontier_nodes =
                    makeNodeMarker("exploration_topo_frontier_viewpoint", Color::Orange(), 0.30);
            for (const auto &node : nodes) {
                switch (node.type) {
                    case general_planner::exploration::TopoNodeType::ODOM:
                        odom_nodes.points.emplace_back(toPoint(node.position));
                        break;
                    case general_planner::exploration::TopoNodeType::HISTORY_ODOM:
                        history_nodes.points.emplace_back(toPoint(node.position));
                        break;
                    case general_planner::exploration::TopoNodeType::FRONTIER_VIEWPOINT:
                        frontier_nodes.points.emplace_back(toPoint(node.position));
                        break;
                    case general_planner::exploration::TopoNodeType::REGION:
                    case general_planner::exploration::TopoNodeType::ROUTE_WAYPOINT:
                    default:
                        region_nodes.points.emplace_back(toPoint(node.position));
                        break;
                }
            }
            if (!odom_nodes.points.empty()) {
                mkr_arr.markers.emplace_back(odom_nodes);
            }
            if (!history_nodes.points.empty()) {
                mkr_arr.markers.emplace_back(history_nodes);
            }
            if (!region_nodes.points.empty()) {
                mkr_arr.markers.emplace_back(region_nodes);
            }
            if (!frontier_nodes.points.empty()) {
                mkr_arr.markers.emplace_back(frontier_nodes);
            }

            auto makeEdgeMarker = [&](const std::string &ns,
                                      const Color &color,
                                      const double width) {
                visualization_msgs::Marker marker;
                marker.header.frame_id = DEFAULT_FRAME_ID;
                marker.header.stamp = ros::Time::now();
                marker.ns = ns;
                marker.id = marker_id++;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.orientation.w = 1.0;
                marker.type = visualization_msgs::Marker::LINE_LIST;
                marker.scale.x = width;
                marker.color = color;
                return marker;
            };
            visualization_msgs::Marker local_edges =
                    makeEdgeMarker("exploration_topo_edges_local_astar", Color::Green(), 0.05);
            visualization_msgs::Marker bubble_edges =
                    makeEdgeMarker("exploration_topo_edges_bubble_astar", Color::SteelBlue(), 0.04);
            visualization_msgs::Marker observed_edges =
                    makeEdgeMarker("exploration_topo_edges_observed_line", Color::Yellow(), 0.03);
            visualization_msgs::Marker unknown_edges =
                    makeEdgeMarker("exploration_topo_edges_unknown", Color(0.8, 0.8, 0.8, 0.55), 0.025);
            for (const auto &edge : edges) {
                visualization_msgs::Marker *edge_marker = &unknown_edges;
                switch (edge.source) {
                    case general_planner::exploration::GuidanceEdgeSource::LOCAL_ASTAR:
                        edge_marker = &local_edges;
                        break;
                    case general_planner::exploration::GuidanceEdgeSource::BUBBLE_ASTAR:
                        edge_marker = &bubble_edges;
                        break;
                    case general_planner::exploration::GuidanceEdgeSource::OBSERVED_LINE:
                        edge_marker = &observed_edges;
                        break;
                    case general_planner::exploration::GuidanceEdgeSource::UNKNOWN:
                    default:
                        edge_marker = &unknown_edges;
                        break;
                }
                if (edge.path.size() >= 2U) {
                    for (std::size_t i = 0; i + 1 < edge.path.size(); ++i) {
                        edge_marker->points.emplace_back(toPoint(edge.path[i]));
                        edge_marker->points.emplace_back(toPoint(edge.path[i + 1]));
                    }
                }
            }
            if (!local_edges.points.empty()) {
                mkr_arr.markers.emplace_back(local_edges);
            }
            if (!bubble_edges.points.empty()) {
                mkr_arr.markers.emplace_back(bubble_edges);
            }
            if (!observed_edges.points.empty()) {
                mkr_arr.markers.emplace_back(observed_edges);
            }
            if (!unknown_edges.points.empty()) {
                mkr_arr.markers.emplace_back(unknown_edges);
            }
            exploration_topo_pub_.publish(mkr_arr);
        }

        void vizExplorationViewpoints(
                const std::vector<general_planner::exploration::ExplorationViewpoint> &viewpoints) override {
            if (!visualization_en_ || exploration_viewpoint_pub_.getNumSubscribers() <= 0) {
                return;
            }
            Ros1Adapter::deleteAllMarkerArray(exploration_viewpoint_pub_);
            if (viewpoints.empty()) {
                return;
            }

            auto toPoint = [](const Vec3f &p) {
                geometry_msgs::Point out;
                out.x = p.x();
                out.y = p.y();
                out.z = p.z();
                return out;
            };

            visualization_msgs::MarkerArray mkr_arr;
            visualization_msgs::Marker points;
            points.header.frame_id = DEFAULT_FRAME_ID;
            points.header.stamp = ros::Time::now();
            points.ns = "exploration_viewpoints";
            points.id = 0;
            points.action = visualization_msgs::Marker::ADD;
            points.pose.orientation.w = 1.0;
            points.type = visualization_msgs::Marker::SPHERE_LIST;
            points.scale.x = 0.18;
            points.scale.y = 0.18;
            points.scale.z = 0.18;
            points.color = Color::Orange();
            points.color.a = 0.95;

            visualization_msgs::Marker yaws;
            yaws.header.frame_id = DEFAULT_FRAME_ID;
            yaws.header.stamp = ros::Time::now();
            yaws.ns = "exploration_viewpoint_yaws";
            yaws.id = 1;
            yaws.action = visualization_msgs::Marker::ADD;
            yaws.pose.orientation.w = 1.0;
            yaws.type = visualization_msgs::Marker::LINE_LIST;
            yaws.scale.x = 0.04;
            yaws.color = Color::Yellow();
            yaws.color.a = 0.9;

            visualization_msgs::Marker links;
            links.header.frame_id = DEFAULT_FRAME_ID;
            links.header.stamp = ros::Time::now();
            links.ns = "exploration_viewpoint_frontier_links";
            links.id = 2;
            links.action = visualization_msgs::Marker::ADD;
            links.pose.orientation.w = 1.0;
            links.type = visualization_msgs::Marker::LINE_LIST;
            links.scale.x = 0.025;
            links.color = Color::Orange();
            links.color.a = 0.55;

            for (const auto &viewpoint : viewpoints) {
                points.points.emplace_back(toPoint(viewpoint.position));
                const Vec3f tip = viewpoint.position +
                                  Vec3f(std::cos(viewpoint.yaw), std::sin(viewpoint.yaw), 0.0) * 0.65;
                yaws.points.emplace_back(toPoint(viewpoint.position));
                yaws.points.emplace_back(toPoint(tip));
                if ((viewpoint.frontier_center - viewpoint.position).norm() > 1.0e-4) {
                    links.points.emplace_back(toPoint(viewpoint.position));
                    links.points.emplace_back(toPoint(viewpoint.frontier_center));
                }
            }
            mkr_arr.markers.emplace_back(points);
            mkr_arr.markers.emplace_back(yaws);
            if (!links.points.empty()) {
                mkr_arr.markers.emplace_back(links);
            }
            exploration_viewpoint_pub_.publish(mkr_arr);
        }

        void vizExplorationGlobalTour(const vec_Vec3f &tour) override {
            if (!visualization_en_ || exploration_global_tour_pub_.getNumSubscribers() <= 0) {
                return;
            }
            Ros1Adapter::deleteAllMarkerArray(exploration_global_tour_pub_);
            if (tour.empty()) {
                return;
            }
            visualization_msgs::MarkerArray mkr_arr;
            Ros1Adapter::addPathToMarkerArray(mkr_arr, tour, Color::Red(), "exploration_global_tour", 0.16, 0.08);
            exploration_global_tour_pub_.publish(mkr_arr);
        }


        /*=============================FOR A* debug ========================================*/
        void vizAstarBoundingBox(const super_utils::Vec3f &bbox_min, const super_utils::Vec3f &bbox_max) override {
            if (!visualization_en_) {
                return;
            }

            if (astar_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }

            visualization_msgs::MarkerArray mkr_arr;
            Ros1Adapter::addBoundingBoxToMarkerArray(mkr_arr, bbox_min, bbox_max, "local_map",
                                                     Color::Chartreuse());
            astar_mkr_pub_.publish(mkr_arr);
        }

        void vizAstarPoints(const super_utils::Vec3f &position, const Color &c,
                            const std::string &ns = "none", const double &size = 0.1,
                            const int &id = 0) override {
            if (!visualization_en_) {
                return;
            }

            if (astar_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }

            visualization_msgs::MarkerArray mkr_arr;
            Ros1Adapter::addPointToMarkerArray(mkr_arr, position, c, ns, size);
            astar_mkr_pub_.publish(mkr_arr);
        }

        /*=============================FOR replan log ========================================*/
        void vizReplanLog(const Trajectory &exp_traj, const Trajectory &backup_traj,
                          const Trajectory &exp_yaw_traj, const Trajectory &backup_yaw_traj,
                          const PolytopeVec &exp_sfc, const Polytope &backup_sfc,
                          const vec_Vec3f &pc_for_sfc, const int &ret_code) override {
            if (!visualization_en_) {
                return;
            }

            if (replan_log_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }

            ros_interface::Ros1Adapter::deleteAllMarkerArray(replan_log_mkr_pub_);

            visualization_msgs::MarkerArray mkr_arr;
            ros_interface::Ros1Adapter::addTrajectoryToMarkerArray(mkr_arr, exp_traj, "exp_traj", Color::Orange(), 0.1,
                                                                   false, false);
            ros_interface::Ros1Adapter::addTrajectoryToMarkerArray(mkr_arr, backup_traj, "backup_traj", Color::Green(),
                                                                   0.1, false, false);
            ros_interface::Ros1Adapter::addYawTrajectoryToMarkerArray(mkr_arr, exp_traj, exp_yaw_traj, "exp_yaw_traj");
            ros_interface::Ros1Adapter::addYawTrajectoryToMarkerArray(mkr_arr, backup_traj, backup_yaw_traj,
                                                                      "backup_yaw_traj");


            int color_id = 0;
            int color_num = exp_sfc.size();

            for (auto p: exp_sfc) {
                double color_ratio = 1.0 - (double) color_id / color_num;
                Vec3f color_mag = tinycolormap::GetColor(color_ratio, tinycolormap::ColormapType::Jet).ConvertToEigen();
                color_id++;
                Color c(color_mag[0], color_mag[1], color_mag[2]);
                ros_interface::Ros1Adapter::addPolytopeToMarkerArray(mkr_arr, p, "exp_sfc", false, Color::SteelBlue(),
                                                                     c,
                                                                     Color::Orange(), 0.15,
                                                                     0.02);
            }

            ros_interface::Ros1Adapter::addPolytopeToMarkerArray(mkr_arr, backup_sfc, "backup_sfc", false,
                                                                 Color::Chartreuse(), Color::Green(),
                                                                 Color::Green(),
                                                                 0.15,
                                                                 0.02);

            replan_log_mkr_pub_.publish(mkr_arr);

            // for pc
            // covert vecVec3f to pclPc
            rog_map::PointCloud pc;
            for (auto p_e: pc_for_sfc) {
                rog_map::PclPoint p;
                p.x = p_e.x();
                p.y = p_e.y();
                p.z = p_e.z();
                pc.push_back(p);
            }
            sensor_msgs::PointCloud2 pc2;
            pcl::toROSMsg(pc, pc2);
            pc2.header.frame_id = "world";
            pc2.header.stamp = ros::Time::now();
            replan_log_pc_pub_.publish(pc2);

            fmt::print("\tResult: {}\n", general_planner::GENERAL_RET_CODE_STR(ret_code));
        }

        void vizCiriSeedLine(const super_utils::Vec3f &a, const super_utils::Vec3f &b, const double &robot_r) override {
            if (!visualization_en_) {
                return;
            }
            if (ciri_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }
            visualization_msgs::MarkerArray mkr_arr;
            ros_interface::Ros1Adapter::addLineToMarkerArray(mkr_arr, a, b,
                                                             Color::Pink(), Color::Orange(), "seed_line",
                                                             robot_r * 2,
                                                             robot_r * 2);
            ciri_mkr_pub_.publish(mkr_arr);
        }

        void vizCiriEllipsoid(const geometry_utils::Ellipsoid &ellipsoid) override{
            if (!visualization_en_) {
                return;
            }
            if (ciri_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }
            visualization_msgs::MarkerArray mkr_arr;
            ros_interface::Ros1Adapter::addEllipsoidToMarkerArray(mkr_arr, ellipsoid, "ellipsoid", Color(Color::Orange(), 0.3));
            ciri_mkr_pub_.publish(mkr_arr);
        }

        void vizCiriInfeasiblePoint(const super_utils::Vec3f p) override{
            if (!visualization_en_) {
                return;
            }
            if (ciri_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }
            visualization_msgs::MarkerArray mkr_arr;
            ros_interface::Ros1Adapter::addPointToMarkerArray(mkr_arr, p, Color::Red(), "infeasible_pt", 0.1);
            ciri_mkr_pub_.publish(mkr_arr);
        }

        void vizCiriPolytope(const geometry_utils::Polytope &polytope, const std::string & ns) override{
            if (!visualization_en_) {
                return;
            }
            if (ciri_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }
            visualization_msgs::MarkerArray mkr_arr;
            ros_interface::Ros1Adapter::addPolytopeToMarkerArray(mkr_arr, polytope, ns, true,
                                                                 Color::Chartreuse(), Color::Green(),
                                                                 Color::Green(),
                                                                 0.15,
                                                                 0.02);
            ciri_mkr_pub_.publish(mkr_arr);
        }

        void vizCiriPointCloud(const vec_Vec3f & points) override {
            if (!visualization_en_) {
                return;
            }

            if (ciri_pc_pub_.getNumSubscribers() <= 0) {
                return;
            }

            sensor_msgs::PointCloud2 pc2;
            ros_interface::Ros1Adapter::addVecPointsToPointCloud2(points, pc2);
            ciri_pc_pub_.publish(pc2);
        }

    private:
        ros::NodeHandle nh_;
        // viz markers
        ros::Publisher goal_pub_, backup_sfc_pub_, backup_traj_pub_, committed_traj_pub_,
                receding_traj_pub_, exp_sfcs_pub_, point_pub_, fov_pub_,
                exp_traj_pub_, astar_pub_, receding_sfc_pub_, backup_traj_star_point_, yaw_traj_pub_, guide_path_pub_;

        ros::Publisher astar_mkr_pub_;

        ros::Publisher exploration_frontier_pub_, exploration_topo_pub_,
                exploration_viewpoint_pub_, exploration_global_tour_pub_;

        ros::Publisher replan_log_mkr_pub_, replan_log_pc_pub_;

        ros::Publisher ciri_mkr_pub_, ciri_pc_pub_;


    };
}

#endif //SRC_ROS1_VISUALIZER_HPP
#endif //USE_ROS1
