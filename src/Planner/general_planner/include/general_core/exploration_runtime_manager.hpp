#pragma once

#include <memory>
#include <string>

#include "general_core/config.hpp"
#include "general_core/exploration_frontend.hpp"

namespace general_planner {

class ExplorationRuntimeManager {
public:
    enum class Status {
        IDLE,
        SELECTING_GOAL,
        GOAL_SELECTED,
        ACTIVE_COMMITTED,
        KEEP_CURRENT_GOAL,
        TEMPORARY_FAILURE,
        FINISHED
    };

    explicit ExplorationRuntimeManager(const Config &cfg);

    void reset();

    void onSelectingGoal();
    void onGoalSelected(const ExplorationGoal &goal);
    void onCommitted(const ExplorationGoal &goal);
    void onKeepCurrentGoal();
    void onTemporaryFailure(const ExplorationGoal &goal);
    void onFinished(const ExplorationGoal &goal);

    bool shouldKeepCurrentGoal(const ExplorationGoal &candidate,
                               const general_utils::Vec3f &robot_pos,
                               double committed_remaining,
                               bool new_task) const;

    bool shouldReuseLatestGoal(const general_utils::Vec3f &robot_pos,
                               double committed_remaining,
                               bool new_task) const;

    bool getLatestGoal(ExplorationGoal &goal) const;

    Status status() const;
    int consecutiveTemporaryFailures() const;
    bool hasCommittedGoal() const;

private:
    bool latestGoalReusable(const general_utils::Vec3f &robot_pos,
                            double committed_remaining,
                            bool new_task) const;

    const Config &cfg_;
    ExplorationGoal latest_goal_;
    Status status_{Status::IDLE};
    int consecutive_temporary_failures_{0};
    bool has_latest_goal_{false};
    bool has_committed_goal_{false};
};

} // namespace general_planner
