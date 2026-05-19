#pragma once

#include <memory>

#include "general_core/general_planner.h"
#include "general_core/task/task_primitive.hpp"

namespace general_planner {

class ExplorationSkill : public TaskPrimitive {
public:
    explicit ExplorationSkill(GeneralPlanner::Ptr planner);

    TaskTickResult tick(const TaskContext &ctx) override;
    void reset() override;
    std::string name() const override;

private:
    GeneralPlanner::Ptr planner_;
    bool from_rest_{true};
};

}  // namespace general_planner
