#pragma once

#include "general_core/task/task_primitive.hpp"

namespace general_planner {

class CompositeTask : public TaskPrimitive {
public:
    void reset() override {
        stage_ = 0;
    }

protected:
    int stage_{0};
};

} // namespace general_planner
