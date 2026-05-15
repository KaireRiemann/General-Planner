#pragma once

#include "general_core/general_planner.h"
#include "general_core/task/composite_task.hpp"

namespace general_planner {

class TrackingPerchingTask : public CompositeTask {
public:
    explicit TrackingPerchingTask(GeneralPlanner::Ptr planner);

    TaskTickResult tick(const TaskContext &ctx) override;
    void reset() override;
    std::string name() const override;

private:
    // Ownership route:
    // TRACKING keeps the stable relative tracking state.
    // TRY_PERCHING only tests readiness and accepted perching candidates.
    // PERCHING_EXECUTING owns the committed final maneuver; tracking cannot overwrite it.
    enum Stage {
        TRACKING = 0,
        TRY_PERCHING = 1,
        PERCHING_EXECUTING = 2,
        CONTACT = 3,
        ABORT = 4
    };

    GeneralPlanner::Ptr planner_;
    bool from_rest_{true};

    static const char *stageName(int stage);
    void changeStage(Stage next);
};

} // namespace general_planner
