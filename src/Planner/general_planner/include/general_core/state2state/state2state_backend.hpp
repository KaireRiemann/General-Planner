#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <general_core/planner_context.hpp>
#include <general_core/planning_backend.hpp>
#include <general_core/planning_result.hpp>

namespace general_planner::architecture {

struct StateToStateBackendRequest {
    PlanInvocation invocation{PlanInvocation::PLAN_FROM_REST};
    general_utils::Vec3f goal{general_utils::Vec3f::Zero()};
    double goal_yaw{0.0};
    bool new_goal{false};
    BackendType requested_backend{BackendType::AUTO};
};

struct StateToStateBackendResult {
    int ret_code{general_utils::FAILED};
    BackendType backend{BackendType::AUTO};
    std::string detail;
};

class StateToStateBackend {
public:
    virtual ~StateToStateBackend() = default;

    virtual BackendDescriptor descriptor() const = 0;
    virtual StateToStateBackendResult run(const StateToStateBackendRequest &request) const = 0;

    bool supports(BackendType backend) const;
};

class LegacyStateToStateBackend final : public StateToStateBackend {
public:
    LegacyStateToStateBackend(PlannerContext context, BackendType backend);

    BackendDescriptor descriptor() const override;

    StateToStateBackendResult run(const StateToStateBackendRequest &request) const override;

private:
    PlannerContext context_;
    BackendType backend_{BackendType::CORRIDOR};
};

class LegacySE3StateToStateBackend final : public StateToStateBackend {
public:
    explicit LegacySE3StateToStateBackend(PlannerContext context);

    BackendDescriptor descriptor() const override;

    StateToStateBackendResult run(const StateToStateBackendRequest &request) const override;

private:
    PlannerContext context_;
};

class StateToStateBackendRouter final {
public:
    StateToStateBackendRouter() = default;

    static StateToStateBackendRouter makeLegacy(PlannerContext context);

    void registerBackend(std::shared_ptr<const StateToStateBackend> backend);

    StateToStateBackendResult run(const StateToStateBackendRequest &request) const;

private:
    const StateToStateBackend *resolve(BackendType backend) const;

    std::vector<std::shared_ptr<const StateToStateBackend>> backends_;
};

} // namespace general_planner::architecture
