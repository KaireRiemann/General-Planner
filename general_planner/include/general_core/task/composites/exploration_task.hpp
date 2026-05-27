#pragma once

#include "general_core/general_planner.h"
#include "general_core/task/composite_task.hpp"

namespace general_planner {

class ExplorationTask : public CompositeTask {
public:
    explicit ExplorationTask(GeneralPlanner::Ptr planner);

    TaskTickResult tick(const TaskContext &ctx) override;
    void reset() override;
    std::string name() const override;

private:
    enum Stage {
        WAIT_OBSERVATION = 0,
        UPDATE_GLOBAL = 1,
        PLAN_LOCAL = 2,
        EXEC_LOCAL = 3,
        RECOVER = 4,
        FINISH = 5
    };

    GeneralPlanner::Ptr planner_;
    bool from_rest_{true};
    bool new_task_consumed_{false};
    double last_global_update_wt_{-1.0};

    static const char *stageName(int stage);
    void changeStage(Stage next);
    TaskTickResult makeResult(TaskStatus status,
                              RET_CODE legacy_ret,
                              const std::string &reason) const;
    TaskTickResult handlePlanResult(RET_CODE ret, const TaskContext &ctx);
};

}  // namespace general_planner
