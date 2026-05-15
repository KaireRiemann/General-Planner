#include "general_core/task/task_factory.hpp"

#include <algorithm>
#include <cctype>
#include <iostream>

#include "general_core/task/composites/full_cycle_task.hpp"
#include "general_core/task/composites/tracking_perching_task.hpp"
#include "general_core/task/skills/dynamic_takeoff_skill.hpp"
#include "general_core/task/skills/perching_skill.hpp"
#include "general_core/task/skills/state_to_state_skill.hpp"
#include "general_core/task/skills/tracking_skill.hpp"

namespace general_planner {

std::string TaskFactory::normalizeMode(std::string mode) {
    std::transform(mode.begin(), mode.end(), mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (mode == "state_to_state" || mode == "state-2-state" ||
        mode == "state2state" || mode == "s2s" ||
        mode == "corridor" || mode == "esdf" || mode == "plain") {
        return "state2state";
    }
    if (mode == "track" || mode == "tracking") {
        return "tracking";
    }
    if (mode == "perch" || mode == "perching") {
        return "perching";
    }
    if (mode == "takeoff" || mode == "dynamic_takeoff" ||
        mode == "dynamic-takeoff" || mode == "unperching") {
        return "dynamic_takeoff";
    }
    if (mode == "tracking_perching" || mode == "tracking-perching" ||
        mode == "track_perch" || mode == "track-perch") {
        return "tracking_perching";
    }
    if (mode == "full_cycle" || mode == "takeoff_tracking_perching" ||
        mode == "takeoff-tracking-perching") {
        return "full_cycle";
    }
    return "state2state";
}

std::unique_ptr<TaskPrimitive> TaskFactory::create(const std::string &mode_string,
                                                   const GeneralPlanner::Ptr &planner,
                                                   const bool tracking_surface_transition) {
    const std::string mode = normalizeMode(mode_string);
    (void)tracking_surface_transition;
    std::unique_ptr<TaskPrimitive> task;
    if (mode == "tracking") {
        task = std::make_unique<TrackingSkill>(planner, false);
    } else if (mode == "perching") {
        task = std::make_unique<PerchingSkill>(planner);
    } else if (mode == "dynamic_takeoff") {
        task = std::make_unique<DynamicTakeoffSkill>(planner);
    } else if (mode == "tracking_perching") {
        task = std::make_unique<TrackingPerchingTask>(planner);
    } else if (mode == "full_cycle") {
        task = std::make_unique<FullCycleTask>(planner);
    } else {
        task = std::make_unique<StateToStateSkill>(planner);
    }

    std::cout << " -- [Task] TASK_FACTORY_CREATE mode=" << mode
              << ", task=" << task->name() << std::endl;
    return task;
}

} // namespace general_planner
