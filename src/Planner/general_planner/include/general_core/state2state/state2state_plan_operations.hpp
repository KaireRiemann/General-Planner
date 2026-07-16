#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <general_core/state2state/state2state_backend_context.hpp>
#include <general_core/state2state/state2state_exp_backup_backend.hpp>
#include <utils/header/type_utils.hpp>

namespace ros_interface {
class RosInterface;
}

namespace general_planner {

class BackupTraj;
class CmdTraj;
class Config;
class ExpTraj;
class LogOneReplan;
class MapManager;

namespace state2state_task {

struct StateToStateTaskServices {
    std::mutex &replan_lock;
    LogOneReplan &latest_replan;
    const Config &cfg;
    std::shared_ptr<MapManager> map_manager;
    std::shared_ptr<ros_interface::RosInterface> ros_ptr;
    general_utils::RobotState &robot_state;
    CmdTraj &cmd_traj_info;
    ExpTraj &last_exp_traj_info;
    general_utils::Vec3f &local_start_p;
    bool &robot_on_backup_traj;
    std::vector<double> &time_consuming;
    double &frontend_time_sum;
    int &frontend_time_count;
    double &backend_time_sum;
    int &backend_time_count;
    StateToStateBackendContext &backend_context;
};

struct StateToStateSE3BackendServices;

general_utils::RET_CODE planFromRest(StateToStateTaskServices &services,
                                   StateToStateExpBackendServices &exp_services,
                                   StateToStateBackupBackendServices &backup_services,
                                   const general_utils::Vec3f &goal,
                                   double goal_yaw,
                                   bool new_goal);

general_utils::RET_CODE replanOnce(StateToStateTaskServices &services,
                                 StateToStateExpBackendServices &exp_services,
                                 StateToStateBackupBackendServices &backup_services,
                                 const general_utils::Vec3f &goal,
                                 double goal_yaw,
                                 bool new_goal);

general_utils::RET_CODE planSE3FromRest(StateToStateTaskServices &services,
                                      StateToStateSE3BackendServices &se3_services,
                                      const general_utils::Vec3f &goal,
                                      double goal_yaw,
                                      bool new_goal);

general_utils::RET_CODE replanSE3Once(StateToStateTaskServices &services,
                                    StateToStateSE3BackendServices &se3_services,
                                    const general_utils::Vec3f &goal,
                                    double goal_yaw,
                                    bool new_goal);

} // namespace state2state_task
} // namespace general_planner
