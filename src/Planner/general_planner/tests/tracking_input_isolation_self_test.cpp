#include <ros_interface/ros1/fsm_ros1.hpp>
#include <stdexcept>
#include <iostream>

namespace {
using general_utils::Vec3f;
void check(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
}
class TestFsm : public fsm::FsmRos1 {
public:
    TestFsm() {
        ros_ptr_ = std::make_shared<ros_interface::Ros1Interface>(ros::NodeHandle("~"));
        cfg_.diagnostic_log_en = false;
        cfg_.tracking_use_target_prediction_path = true;
        cfg_.tracking_target_prediction_topic = "/test/prediction";
    }
    void prepare(fsm::TaskMode mode, bool active_goal) {
        cfg_.task_mode = mode;
        gi_.goal_p = Vec3f(5, 6, 7);
        gi_.goal_yaw = .7;
        gi_.new_goal = active_goal;
        task_new_ = active_goal;
        started_ = active_goal;
        finish_plan = true;
    }
    void unchanged(bool active_goal) {
        check((gi_.goal_p - Vec3f(5, 6, 7)).norm() == 0, "target overwrote navigation goal");
        check(gi_.goal_yaw == .7, "target overwrote navigation yaw");
        check(gi_.new_goal == active_goal && task_new_ == active_goal &&
              started_ == active_goal && finish_plan, "target changed task lifecycle");
    }
    void trackingActivated() {
        check(started_ && gi_.new_goal && task_new_ && !finish_plan,
              "explicit tracking mode must still accept target input");
        check((gi_.goal_p - Vec3f(30, 40, 2)).norm() < 1.e-6,
              "tracking goal was not updated");
    }
    void cached() {
        check(!tracking_target_prediction_.empty(), "inactive mode should cache observations");
    }
};
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "tracking_input_isolation_self_test");
    auto odom = boost::make_shared<nav_msgs::Odometry>();
    odom->pose.pose.position.x = 30;
    odom->pose.pose.position.y = 40;
    odom->pose.pose.position.z = 2;
    odom->pose.pose.orientation.w = 1;
    auto path = boost::make_shared<nav_msgs::Path>();
    path->poses.resize(2);
    for (auto &pose : path->poses) pose.pose = odom->pose.pose;
    for (bool active_goal : {false, true}) {
        TestFsm fsm;
        fsm.prepare(fsm::TaskMode::STATE_TO_STATE, active_goal);
        fsm.trackingTargetCallback(odom);
        fsm.unchanged(active_goal);
        fsm.cached();
        fsm.trackingPredictionPathCallback(path);
        fsm.unchanged(active_goal);
        fsm.trackingTargetCallback(odom); // fresh prediction takes precedence
        fsm.unchanged(active_goal);
        fsm.trackingTargetCallback({});
        fsm.trackingPredictionPathCallback({});
        fsm.unchanged(active_goal);
    }
    for (auto mode : {fsm::TaskMode::TRACKING, fsm::TaskMode::TRACKING_PERCHING}) {
        TestFsm odom_fsm;
        odom_fsm.prepare(mode, false);
        odom_fsm.trackingTargetCallback(odom);
        odom_fsm.trackingActivated();
        TestFsm path_fsm;
        path_fsm.prepare(mode, false);
        path_fsm.trackingPredictionPathCallback(path);
        path_fsm.trackingActivated();
    }
    std::cout << "tracking_input_isolation_self_test: PASS\n";
}
