#pragma once

#include "general_core/general_planner.h"
#include "general_core/task/composite_task.hpp"
#include "general_core/task/composites/tracking_perching_task.hpp"
#include "general_core/task/skills/dynamic_takeoff_skill.hpp"
#include "general_core/task/skills/tracking_skill.hpp"

namespace general_planner {

class FullCycleTask : public CompositeTask {
public:
    explicit FullCycleTask(GeneralPlanner::Ptr planner);

    TaskTickResult tick(const TaskContext &ctx) override;
    void reset() override;
    std::string name() const override;

private:
    enum Stage {
        TAKEOFF = 0,
        TRACKING = 1,
        TRACKING_PERCHING = 2,
        CONTACT = 3
    };

    GeneralPlanner::Ptr planner_;
    DynamicTakeoffSkill takeoff_skill_;
    TrackingSkill tracking_skill_;
    TrackingPerchingTask tracking_perching_task_;
    bool takeoff_committed_{false};

    static const char *stageName(int stage);
    void changeStage(Stage next);
};

} // namespace general_planner
