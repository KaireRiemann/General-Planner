#pragma once

#include <string>
#include <utility>

#include <general_core/planning_backend.hpp>
#include <general_core/planning_semantics.hpp>

namespace general_planner::architecture {

struct MissionSnapshot {
    MissionMode mission{MissionMode::IDLE};
    TaskType active_task{TaskType::STATE_TO_STATE};
    BackendType backend{BackendType::AUTO};
    ExecutionPhase phase{ExecutionPhase::WAITING_INPUT};
    std::string plugin_name{"none"};
    std::string transition_reason;
};

class MissionOrchestrator {
public:
    MissionOrchestrator()
        : backend_registry_(PlanningBackendRegistry::makeDefault()) {}

    void setActiveTask(TaskIdentity identity, const std::string &reason) {
        snapshot_.mission = identity.mission;
        snapshot_.active_task = identity.task;
        snapshot_.backend = identity.backend;
        snapshot_.plugin_name = identity.plugin_name;
        snapshot_.transition_reason = reason;
        active_identity_ = std::move(identity);
    }

    void setExecutionPhase(ExecutionPhase phase, const std::string &reason) {
        snapshot_.phase = phase;
        snapshot_.transition_reason = reason;
    }

    void setIdle(const std::string &reason) {
        snapshot_.mission = MissionMode::IDLE;
        snapshot_.phase = ExecutionPhase::WAITING_INPUT;
        snapshot_.plugin_name = "none";
        snapshot_.transition_reason = reason;
        active_identity_ = TaskIdentity{};
    }

    const MissionSnapshot &snapshot() const {
        return snapshot_;
    }

    const TaskIdentity &activeIdentity() const {
        return active_identity_;
    }

    const PlanningBackend *activeBackend() const {
        return backend_registry_.resolve(active_identity_);
    }

    const PlanningBackendRegistry &backendRegistry() const {
        return backend_registry_;
    }

private:
    TaskIdentity active_identity_;
    MissionSnapshot snapshot_;
    PlanningBackendRegistry backend_registry_;
};

} // namespace general_planner::architecture
