#pragma once

#include <general_core/planning_result.hpp>

namespace general_planner::architecture {

template <typename ContextT>
class TaskPlugin {
public:
    virtual ~TaskPlugin() = default;

    virtual TaskIdentity identity(const ContextT &context) const = 0;
    virtual TaskPluginDescriptor descriptor(const ContextT &context) const {
        TaskPluginDescriptor descriptor;
        descriptor.identity = identity(context);
        descriptor.continuous_replan = descriptor.identity.tracking_like ||
                                       descriptor.identity.task == TaskType::EXPLORATION;
        return descriptor;
    }

    virtual bool ready(ContextT &context) = 0;
    virtual bool replanAllowed(const ContextT &context) const = 0;
    virtual PlanResult plan(ContextT &context, const PlanRequest &request) = 0;
    virtual PlanResult replan(ContextT &context, const PlanRequest &request) = 0;
    virtual bool shouldGenerateAfterTrajFinish(ContextT &context) = 0;
};

} // namespace general_planner::architecture
