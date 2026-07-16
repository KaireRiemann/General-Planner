#include "goal_3d_tool.hpp"

#include <cmath>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <rviz/display_context.h>
#include <rviz/properties/string_property.h>

namespace general_planner_rviz_plugins {

    Goal3DTool::Goal3DTool() {
        shortcut_key_ = 'g';
        goal_3d_topic_property_ = new rviz::StringProperty(
                "3D Topic", "/goal_3d",
                "PoseStamped topic used when the height gesture was entered.",
                getPropertyContainer(), SLOT(updateTopics()), this);
        fallback_goal_topic_property_ = new rviz::StringProperty(
                "Fallback 2D Topic", "/goal",
                "PoseStamped topic used when no explicit height was selected.",
                getPropertyContainer(), SLOT(updateTopics()), this);
    }

    void Goal3DTool::onInitialize() {
        Pose3DTool::onInitialize();
        setName("3D Nav Goal");
        updateTopics();
    }

    void Goal3DTool::updateTopics() {
        goal_3d_publisher_.shutdown();
        fallback_goal_publisher_.shutdown();
        const std::string goal_3d_topic = goal_3d_topic_property_->getStdString();
        const std::string fallback_goal_topic = fallback_goal_topic_property_->getStdString();
        if (goal_3d_topic.empty() || fallback_goal_topic.empty()) {
            ROS_WARN("Both 3D Goal tool topics must be non-empty.");
            return;
        }
        if (node_handle_.resolveName(goal_3d_topic) ==
            node_handle_.resolveName(fallback_goal_topic)) {
            ROS_WARN("3D and fallback goal topics resolve to the same name; height semantics are ambiguous.");
        }
        goal_3d_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(
                goal_3d_topic, 1);
        fallback_goal_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(
                fallback_goal_topic, 1);
    }

    void Goal3DTool::onPoseSet(const double x,
                               const double y,
                               const double z,
                               const double yaw,
                               const bool height_selected) {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = context_->getFixedFrame().toStdString();
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = z;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = std::sin(0.5 * yaw);
        goal.pose.orientation.w = std::cos(0.5 * yaw);

        if (height_selected) {
            ROS_INFO("Explicit 3D goal: frame=%s, position=(%.3f, %.3f, %.3f), yaw=%.3f",
                     goal.header.frame_id.c_str(), x, y, z, yaw);
            goal_3d_publisher_.publish(goal);
        } else {
            ROS_INFO("Goal height was not selected; publish XY/yaw to the click-height fallback topic.");
            fallback_goal_publisher_.publish(goal);
        }
    }

} // namespace general_planner_rviz_plugins

PLUGINLIB_EXPORT_CLASS(general_planner_rviz_plugins::Goal3DTool, rviz::Tool)
