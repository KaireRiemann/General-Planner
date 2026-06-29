#pragma once

#include <memory>
#include <string>

#include <general_core/state2state/state2state_backend_context.hpp>
#include <utils/header/type_utils.hpp>

namespace geometry_utils {
class Trajectory;
}

namespace ros_interface {
class RosInterface;
}

namespace general_planner {

class Config;
class MapManager;
class SE3AggressiveManager;

namespace state2state_task {

class StateToStateSE3BackendRuntime {
public:
    virtual ~StateToStateSE3BackendRuntime() = default;

    virtual general_utils::StatePVAJ makeTaskHeadState(bool from_rest) = 0;
    virtual bool commitSE3AggressiveTrajectory(const geometry_utils::Trajectory &pos_traj,
                                               const std::string &traj_ns) = 0;
};

struct StateToStateSE3BackendServices {
    const Config &cfg;
    std::shared_ptr<MapManager> map_manager;
    std::shared_ptr<ros_interface::RosInterface> ros_ptr;
    general_utils::RobotState &robot_state;
    SE3AggressiveManager *manager;
    StateToStateBackendContext &goal_context;
    StateToStateSE3BackendRuntime &runtime;
};

general_utils::RET_CODE optimizeSE3AggressiveTask(StateToStateSE3BackendServices &services,
                                                const general_utils::Vec3f &goal,
                                                double goal_yaw,
                                                bool from_rest);

} // namespace state2state_task
} // namespace general_planner
