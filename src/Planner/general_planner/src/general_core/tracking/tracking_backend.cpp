#include <general_core/tracking/tracking_backend.hpp>

#include <general_core/config.hpp>

namespace general_planner::tracking_task {

namespace {

class TrackingBackend {
public:
    virtual ~TrackingBackend() = default;

    virtual architecture::BackendType type() const = 0;

    virtual general_utils::RET_CODE run(TrackingBackendServices &services,
                                        const traj_opt::DynamicTargetStates &target_prediction,
                                        bool from_rest) const = 0;
};

class JerkTrackingBackend final : public TrackingBackend {
public:
    architecture::BackendType type() const override {
        return architecture::BackendType::JERK_TRACKING;
    }

    general_utils::RET_CODE run(TrackingBackendServices &services,
                                const traj_opt::DynamicTargetStates &target_prediction,
                                const bool from_rest) const override {
        return services.runtime.optimizeTrackingTask(target_prediction, from_rest);
    }
};

class SnapTrackingBackend final : public TrackingBackend {
public:
    architecture::BackendType type() const override {
        return architecture::BackendType::SNAP_TRACKING;
    }

    general_utils::RET_CODE run(TrackingBackendServices &services,
                                const traj_opt::DynamicTargetStates &target_prediction,
                                const bool from_rest) const override {
        return services.runtime.optimizeTrackingTask(target_prediction, from_rest);
    }
};

const TrackingBackend &trackingBackendFor(const architecture::BackendType backend) {
    static const JerkTrackingBackend jerk_backend;
    static const SnapTrackingBackend snap_backend;
    if (backend == architecture::BackendType::SNAP_TRACKING) {
        return snap_backend;
    }
    return jerk_backend;
}

} // namespace

architecture::BackendType resolveTrackingBackend(const Config &cfg) {
    return cfg.tracking_use_snap
               ? architecture::BackendType::SNAP_TRACKING
               : architecture::BackendType::JERK_TRACKING;
}

general_utils::RET_CODE runTrackingBackend(TrackingBackendServices &services,
                                         const traj_opt::DynamicTargetStates &target_prediction,
                                         const bool from_rest) {
    const TrackingBackend &backend = trackingBackendFor(services.backend);
    return backend.run(services, target_prediction, from_rest);
}

} // namespace general_planner::tracking_task
