#include <general_core/state2state/state2state_backend.hpp>
#include <general_core/state2state/state2state_plan_operations.hpp>

namespace general_planner::architecture {

bool StateToStateBackend::supports(const BackendType backend) const {
    const BackendType resolved = backend == BackendType::AUTO ? BackendType::CORRIDOR : backend;
    return descriptor().type == resolved;
}

LegacyStateToStateBackend::LegacyStateToStateBackend(PlannerContext context,
                                                     const BackendType backend)
    : context_(std::move(context)),
      backend_(backend == BackendType::AUTO ? BackendType::CORRIDOR : backend) {}

BackendDescriptor LegacyStateToStateBackend::descriptor() const {
    return BackendDescriptor{backend_, toString(backend_), {TaskType::STATE_TO_STATE}, true, true};
}

StateToStateBackendResult
LegacyStateToStateBackend::run(const StateToStateBackendRequest &request) const {
    StateToStateBackendResult result;
    result.backend = backend_;
    result.detail = std::string("legacy_state2state_backend=") + toString(backend_);
    if (!context_.valid()) {
        result.ret_code = general_utils::INIT_ERROR;
        return result;
    }
    auto services = context_.planner().makeStateToStateTaskServices();
    auto exp_services = context_.planner().makeStateToStateExpBackendServices();
    auto backup_services = context_.planner().makeStateToStateBackupBackendServices();
    if (request.invocation == PlanInvocation::PLAN_FROM_REST) {
        result.ret_code = state2state_task::planFromRest(services,
                                                         exp_services,
                                                         backup_services,
                                                         request.goal,
                                                         request.goal_yaw,
                                                         request.new_goal);
        return result;
    }
    result.ret_code = state2state_task::replanOnce(services,
                                                   exp_services,
                                                   backup_services,
                                                   request.goal,
                                                   request.goal_yaw,
                                                   request.new_goal);
    return result;
}

LegacySE3StateToStateBackend::LegacySE3StateToStateBackend(PlannerContext context)
    : context_(std::move(context)) {}

BackendDescriptor LegacySE3StateToStateBackend::descriptor() const {
    return BackendDescriptor{BackendType::SE3, "se3", {TaskType::STATE_TO_STATE}, false, true};
}

StateToStateBackendResult
LegacySE3StateToStateBackend::run(const StateToStateBackendRequest &request) const {
    StateToStateBackendResult result;
    result.backend = BackendType::SE3;
    result.detail = "legacy_state2state_backend=se3";
    if (!context_.valid()) {
        result.ret_code = general_utils::INIT_ERROR;
        return result;
    }
    auto services = context_.planner().makeStateToStateTaskServices();
    auto se3_services = context_.planner().makeStateToStateSE3BackendServices();
    if (request.invocation == PlanInvocation::PLAN_FROM_REST) {
        result.ret_code = state2state_task::planSE3FromRest(services,
                                                            se3_services,
                                                            request.goal,
                                                            request.goal_yaw,
                                                            request.new_goal);
        return result;
    }
    result.ret_code = state2state_task::replanSE3Once(services,
                                                      se3_services,
                                                      request.goal,
                                                      request.goal_yaw,
                                                      request.new_goal);
    return result;
}

StateToStateBackendRouter StateToStateBackendRouter::makeLegacy(PlannerContext context) {
    StateToStateBackendRouter router;
    router.registerBackend(std::make_shared<LegacyStateToStateBackend>(context, BackendType::CORRIDOR));
    router.registerBackend(std::make_shared<LegacyStateToStateBackend>(context, BackendType::ESDF));
    router.registerBackend(std::make_shared<LegacyStateToStateBackend>(context, BackendType::PLAIN));
    router.registerBackend(std::make_shared<LegacySE3StateToStateBackend>(std::move(context)));
    return router;
}

void StateToStateBackendRouter::registerBackend(std::shared_ptr<const StateToStateBackend> backend) {
    if (backend) {
        backends_.push_back(std::move(backend));
    }
}

StateToStateBackendResult
StateToStateBackendRouter::run(const StateToStateBackendRequest &request) const {
    const StateToStateBackend *backend = resolve(request.requested_backend);
    if (!backend) {
        StateToStateBackendResult result;
        result.ret_code = general_utils::INIT_ERROR;
        result.backend = request.requested_backend;
        result.detail = std::string("state2state_backend_unavailable=") +
                        toString(request.requested_backend);
        return result;
    }
    return backend->run(request);
}

const StateToStateBackend *StateToStateBackendRouter::resolve(const BackendType backend) const {
    for (const auto &candidate: backends_) {
        if (candidate && candidate->supports(backend)) {
            return candidate.get();
        }
    }
    return nullptr;
}

} // namespace general_planner::architecture
