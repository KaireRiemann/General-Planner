#pragma once

#include <cstddef>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <data_structure/exp_traj.h>
#include <general_core/log_utils.hpp>
#include <general_core/tracking/tracking_backend.hpp>
#include <general_core/tracking/tracking_perching_transition_manager.hpp>
#include <ros_interface/ros_interface.hpp>
#include <utils/header/type_utils.hpp>
#include <traj_opt/tracking_perching_traj_opt.hpp>

namespace general_planner {

namespace tracking_task {

struct TrackingTaskServices {
    std::mutex &replan_lock;
    LogOneReplan &latest_replan;
    ExpTraj &last_exp_traj_info;
    general_utils::RobotState &robot_state;
    ros_interface::RosInterface::Ptr ros_ptr;
    TrackingPerchingTransitionManager *tracking_perching_manager{nullptr};
    std::vector<double> &time_consuming;

    std::function<void(const std::string &phase,
                       const std::string &reason,
                       std::size_t guide_path_size,
                       std::size_t committed_path_size,
                       std::size_t target_prediction_size,
                       double distance_to_target)> set_tracking_diagnostic;
    std::function<void(const general_utils::Vec3f &goal, double yaw, bool new_task)> set_goal_info;
    std::function<void(bool new_task, const std::string &context)> maybe_reset_tracking_runtime;
    std::function<general_utils::RET_CODE(const traj_opt::PerchingSurfaceState &surface,
                                        bool from_rest)> optimize_perching_task;
    std::function<general_utils::RET_CODE(const traj_opt::DynamicTargetStates &target_prediction,
                                        const traj_opt::PerchingSurfaceState &surface,
                                        general_utils::RET_CODE tracking_ret)> commit_perching_from_tracking;
};

general_utils::RET_CODE planFromRest(TrackingTaskServices &services,
                                   TrackingBackendServices &backend_services,
                                   const traj_opt::DynamicTargetStates &target_prediction,
                                   bool new_task);

general_utils::RET_CODE replanOnce(TrackingTaskServices &services,
                                 TrackingBackendServices &backend_services,
                                 const traj_opt::DynamicTargetStates &target_prediction,
                                 bool new_task);

general_utils::RET_CODE replanWithPerchingSurface(
        TrackingTaskServices &services,
        TrackingBackendServices &backend_services,
        const traj_opt::DynamicTargetStates &target_prediction,
        const traj_opt::PerchingSurfaceState &surface,
        bool new_task);

general_utils::RET_CODE tryCommitPerchingFromTracking(
        TrackingTaskServices &services,
        const traj_opt::DynamicTargetStates &target_prediction,
        const traj_opt::PerchingSurfaceState &surface,
        general_utils::RET_CODE tracking_ret);

} // namespace tracking_task
} // namespace general_planner
