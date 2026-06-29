#pragma once

#include <utils/header/type_utils.hpp>

namespace general_planner {

namespace state2state_task {

class StateToStateBackendContext {
public:
    virtual ~StateToStateBackendContext() = default;

    virtual void setGoalInfo(const general_utils::Vec3f &goal,
                             double goal_yaw,
                             bool new_goal,
                             bool goal_valid) = 0;
    virtual void markGoalConsumed() = 0;
};

} // namespace state2state_task
} // namespace general_planner
