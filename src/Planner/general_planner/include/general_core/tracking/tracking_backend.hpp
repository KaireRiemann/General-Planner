#pragma once

#include <general_core/planning_semantics.hpp>
#include <utils/header/type_utils.hpp>
#include <traj_opt/tracking_perching_traj_opt.hpp>

namespace general_planner {

class Config;

namespace tracking_task {

class TrackingBackendRuntime {
public:
    virtual ~TrackingBackendRuntime() = default;

    virtual general_utils::RET_CODE optimizeTrackingTask(
            const traj_opt::DynamicTargetStates &target_prediction,
            bool from_rest) = 0;
};

struct TrackingBackendServices {
    architecture::BackendType backend{architecture::BackendType::JERK_TRACKING};
    TrackingBackendRuntime &runtime;
};

architecture::BackendType resolveTrackingBackend(const Config &cfg);

general_utils::RET_CODE runTrackingBackend(TrackingBackendServices &services,
                                         const traj_opt::DynamicTargetStates &target_prediction,
                                         bool from_rest);

} // namespace tracking_task
} // namespace general_planner
