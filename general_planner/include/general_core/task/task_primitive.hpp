#pragma once

#include <string>

#include "general_core/task/task_context.hpp"
#include "general_core/task/task_result.hpp"

namespace general_planner {

class TaskPrimitive {
public:
    virtual ~TaskPrimitive() = default;

    virtual TaskTickResult tick(const TaskContext &ctx) = 0;
    virtual void reset() = 0;
    virtual std::string name() const = 0;
};

} // namespace general_planner
