#include "general_core/exploration_runtime_manager.hpp"

#include <algorithm>
#include <cmath>

namespace general_planner {

ExplorationRuntimeManager::ExplorationRuntimeManager(const Config &cfg)
        : cfg_(cfg)
{
}

void ExplorationRuntimeManager::reset()
{
    latest_goal_ = ExplorationGoal{};
    status_ = Status::IDLE;
    consecutive_temporary_failures_ = 0;
    has_latest_goal_ = false;
    has_committed_goal_ = false;
}

void ExplorationRuntimeManager::onSelectingGoal()
{
    status_ = Status::SELECTING_GOAL;
}

void ExplorationRuntimeManager::onGoalSelected(const ExplorationGoal &goal)
{
    latest_goal_ = goal;
    has_latest_goal_ = goal.valid;
    consecutive_temporary_failures_ = 0;
    status_ = Status::GOAL_SELECTED;
}

void ExplorationRuntimeManager::onCommitted(const ExplorationGoal &goal)
{
    latest_goal_ = goal;
    has_latest_goal_ = goal.valid;
    has_committed_goal_ = goal.valid;
    consecutive_temporary_failures_ = 0;
    status_ = Status::ACTIVE_COMMITTED;
}

void ExplorationRuntimeManager::onKeepCurrentGoal()
{
    status_ = Status::KEEP_CURRENT_GOAL;
}

void ExplorationRuntimeManager::onTemporaryFailure(const ExplorationGoal &goal)
{
    if (goal.valid) {
        latest_goal_ = goal;
        has_latest_goal_ = true;
    }
    ++consecutive_temporary_failures_;
    status_ = Status::TEMPORARY_FAILURE;
}

void ExplorationRuntimeManager::onFinished(const ExplorationGoal &goal)
{
    latest_goal_ = goal;
    has_latest_goal_ = goal.valid;
    has_committed_goal_ = false;
    status_ = Status::FINISHED;
}

bool ExplorationRuntimeManager::latestGoalReusable(const general_utils::Vec3f &robot_pos,
                                                   const double committed_remaining,
                                                   const bool new_task) const
{
    if (new_task || !has_latest_goal_ || !latest_goal_.valid ||
        !latest_goal_.position.allFinite() || !robot_pos.allFinite()) {
        return false;
    }

    const double reached_distance = std::max(0.0, cfg_.exploration_goal_reached_distance);
    if ((robot_pos - latest_goal_.position).norm() <= reached_distance) {
        return false;
    }

    const double min_remaining = std::max(0.25, cfg_.replan_forward_dt);
    return committed_remaining > min_remaining;
}

bool ExplorationRuntimeManager::shouldKeepCurrentGoal(const ExplorationGoal &candidate,
                                                      const general_utils::Vec3f &robot_pos,
                                                      const double committed_remaining,
                                                      const bool new_task) const
{
    if (!candidate.valid ||
        !latestGoalReusable(robot_pos, committed_remaining, new_task)) {
        return false;
    }

    const double switch_margin =
            std::max(0.0, cfg_.exploration_goal_switch_min_score_improvement);
    return candidate.score >= latest_goal_.score - switch_margin;
}

bool ExplorationRuntimeManager::shouldReuseLatestGoal(const general_utils::Vec3f &robot_pos,
                                                      const double committed_remaining,
                                                      const bool new_task) const
{
    return latestGoalReusable(robot_pos, committed_remaining, new_task);
}

bool ExplorationRuntimeManager::getLatestGoal(ExplorationGoal &goal) const
{
    goal = latest_goal_;
    return has_latest_goal_ && latest_goal_.valid;
}

ExplorationRuntimeManager::Status ExplorationRuntimeManager::status() const
{
    return status_;
}

int ExplorationRuntimeManager::consecutiveTemporaryFailures() const
{
    return consecutive_temporary_failures_;
}

bool ExplorationRuntimeManager::hasCommittedGoal() const
{
    return has_committed_goal_;
}

} // namespace general_planner
