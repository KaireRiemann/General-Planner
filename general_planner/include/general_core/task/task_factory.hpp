#pragma once

#include <memory>
#include <string>

#include "general_core/general_planner.h"
#include "general_core/task/task_primitive.hpp"

namespace general_planner {

class TaskFactory {
public:
    static std::unique_ptr<TaskPrimitive> create(const std::string &mode_string,
                                                 const GeneralPlanner::Ptr &planner,
                                                 bool tracking_surface_transition = false);

    static std::string normalizeMode(std::string mode);
};

} // namespace general_planner
