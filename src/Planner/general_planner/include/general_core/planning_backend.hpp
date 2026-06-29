#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <general_core/planning_semantics.hpp>

namespace general_planner::architecture {

class PlanningBackend {
public:
    virtual ~PlanningBackend() = default;

    virtual BackendDescriptor descriptor() const = 0;

    virtual bool supports(const TaskIdentity &identity) const {
        const auto desc = descriptor();
        if (desc.type != BackendType::AUTO && identity.backend != BackendType::AUTO &&
            identity.backend != desc.type) {
            return false;
        }
        if (desc.supported_tasks.empty()) {
            return true;
        }
        return std::find(desc.supported_tasks.begin(),
                         desc.supported_tasks.end(),
                         identity.task) != desc.supported_tasks.end();
    }
};

class StaticPlanningBackend final : public PlanningBackend {
public:
    explicit StaticPlanningBackend(BackendDescriptor descriptor)
        : descriptor_(std::move(descriptor)) {}

    BackendDescriptor descriptor() const override {
        return descriptor_;
    }

private:
    BackendDescriptor descriptor_;
};

class PlanningBackendRegistry {
public:
    void registerBackend(std::shared_ptr<const PlanningBackend> backend) {
        if (backend) {
            backends_.push_back(std::move(backend));
        }
    }

    const PlanningBackend *resolve(const TaskIdentity &identity) const {
        for (const auto &backend: backends_) {
            if (backend && backend->supports(identity)) {
                return backend.get();
            }
        }
        return nullptr;
    }

    const std::vector<std::shared_ptr<const PlanningBackend>> &backends() const {
        return backends_;
    }

    static PlanningBackendRegistry makeDefault() {
        PlanningBackendRegistry registry;
        registry.registerBackend(std::make_shared<StaticPlanningBackend>(
                BackendDescriptor{BackendType::CORRIDOR,
                                  "corridor",
                                  {TaskType::STATE_TO_STATE, TaskType::TRACKING, TaskType::PERCHING, TaskType::TAKEOFF},
                                  true,
                                  true}));
        registry.registerBackend(std::make_shared<StaticPlanningBackend>(
                BackendDescriptor{BackendType::ESDF,
                                  "esdf",
                                  {TaskType::STATE_TO_STATE, TaskType::TRACKING},
                                  true,
                                  true}));
        registry.registerBackend(std::make_shared<StaticPlanningBackend>(
                BackendDescriptor{BackendType::PLAIN,
                                  "plain",
                                  {TaskType::STATE_TO_STATE, TaskType::TRACKING},
                                  false,
                                  true}));
        registry.registerBackend(std::make_shared<StaticPlanningBackend>(
                BackendDescriptor{BackendType::JERK_TRACKING,
                                  "jerk_tracking",
                                  {TaskType::TRACKING},
                                  true,
                                  true}));
        registry.registerBackend(std::make_shared<StaticPlanningBackend>(
                BackendDescriptor{BackendType::SNAP_TRACKING,
                                  "snap_tracking",
                                  {TaskType::TRACKING},
                                  true,
                                  true}));
        registry.registerBackend(std::make_shared<StaticPlanningBackend>(
                BackendDescriptor{BackendType::SE3,
                                  "se3",
                                  {TaskType::STATE_TO_STATE},
                                  false,
                                  true}));
        return registry;
    }

private:
    std::vector<std::shared_ptr<const PlanningBackend>> backends_;
};

} // namespace general_planner::architecture
