#pragma once

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include "pose_3d_tool.hpp"

namespace rviz {
    class StringProperty;
}

namespace general_planner_rviz_plugins {

    class Goal3DTool final : public Pose3DTool {
        Q_OBJECT

    public:
        Goal3DTool();
        ~Goal3DTool() override = default;

        void onInitialize() override;

    private Q_SLOTS:
        void updateTopics();

    private:
        void onPoseSet(double x,
                       double y,
                       double z,
                       double yaw,
                       bool height_selected) override;

        ros::NodeHandle node_handle_;
        ros::Publisher goal_3d_publisher_;
        ros::Publisher fallback_goal_publisher_;
        rviz::StringProperty *goal_3d_topic_property_{nullptr};
        rviz::StringProperty *fallback_goal_topic_property_{nullptr};
    };

} // namespace general_planner_rviz_plugins
