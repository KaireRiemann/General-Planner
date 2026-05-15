#pragma once

#include "general_core/general_planner.h"
#include "general_core/task/task_primitive.hpp"

namespace general_planner {

class TrackingSkill : public TaskPrimitive {
public:
    explicit TrackingSkill(GeneralPlanner::Ptr planner,
                           bool use_surface_transition = false);

    TaskTickResult tick(const TaskContext &ctx) override;
    void reset() override;
    std::string name() const override;

    void setUseSurfaceTransition(bool enable);

private:
    GeneralPlanner::Ptr planner_;
    bool from_rest_{true};
    bool use_surface_transition_{false};
};

} // namespace general_planner
