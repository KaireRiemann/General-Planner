#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <general_core/state2state/state2state_backend_context.hpp>
#include <general_core/state2state/state2state_frontend_services.hpp>
#include <general_core/runtime_trajectory_safety.hpp>
#include <utils/header/type_utils.hpp>

namespace ros_interface {
class RosInterface;
}

namespace traj_opt {
class TrajManager;
}

namespace geometry_utils {
class Trajectory;
}

namespace general_planner {

class BackupTraj;
class CmdTraj;
class Config;
class CorridorGenerator;
class ExpTraj;
class FOVChecker;
class LogOneReplan;
class MapManager;

namespace state2state_task {

struct State2StateZSummary {
    bool valid{false};
    double start{0.0};
    double end{0.0};
    double min{0.0};
    double max{0.0};
};

struct State2StateZDebug {
    bool valid{false};
    std::string exp_mode{"none"};
    int guide_size{0};
    State2StateZSummary guide;
    State2StateZSummary optimized;
    State2StateZSummary exp_full;
    double goal_z{0.0};
    double robot_z{0.0};
    double local_target_z{0.0};
    double local_target_goal_xy_dist{0.0};
    double local_target_goal_dist{0.0};
    double local_target_goal_z_err{0.0};
    double opt_end_local_target_z_err{0.0};
    bool local_target_is_global_goal{false};
    bool guide_reused_command_prefix{false};
    bool z_lower_guard_enabled{false};
    bool z_lower_guard_passed{true};
    int z_lower_guard_samples{0};
    double z_lower_guard_tolerance{0.0};
    double z_lower_guard_min_margin{0.0};
    double z_lower_guard_max_violation{0.0};
};

struct StateToStateExpBackendServices {
    StateToStateFrontendServices frontend;
    RuntimeTrajectorySafetyServices runtime_safety;
    const Config &cfg;
    std::shared_ptr<MapManager> map_manager;
    std::shared_ptr<CorridorGenerator> corridor_generator;
    std::shared_ptr<ros_interface::RosInterface> ros_ptr;
    std::shared_ptr<traj_opt::TrajManager> traj_manager;
    general_utils::RobotState &robot_state;
    CmdTraj &cmd_traj_info;
    ExpTraj &last_exp_traj_info;
    general_utils::Vec3f &local_start_p;
    bool &robot_on_backup_traj;
    std::vector<double> &time_consuming;
    general_utils::Vec3f &shifted_sfc_start_pt;
    LogOneReplan &latest_replan;
    general_utils::Vec3f &goal_p;
    double &goal_yaw;
    bool &new_goal;
    State2StateZDebug &z_debug;
};

struct StateToStateBackupBackendServices {
    const Config &cfg;
    std::shared_ptr<MapManager> map_manager;
    std::shared_ptr<CorridorGenerator> corridor_generator;
    std::shared_ptr<FOVChecker> fov_checker;
    std::shared_ptr<ros_interface::RosInterface> ros_ptr;
    std::shared_ptr<traj_opt::TrajManager> traj_manager;
    general_utils::RobotState &robot_state;
    std::mutex &drone_state_mutex;
    general_utils::Vec3f &shifted_sfc_start_pt;
    const double &goal_yaw;
    LogOneReplan &latest_replan;
    CmdTraj &cmd_traj_info;
    std::vector<double> &time_consuming;
};

general_utils::RET_CODE generateExpTrajectory(StateToStateExpBackendServices &services,
                                            ExpTraj &last_exp_traj,
                                            ExpTraj &out_exp_traj);

general_utils::RET_CODE generateBackupTrajectory(StateToStateBackupBackendServices &services,
                                               ExpTraj &ref_exp_traj,
                                               BackupTraj &out_backup_traj);

} // namespace state2state_task
} // namespace general_planner
