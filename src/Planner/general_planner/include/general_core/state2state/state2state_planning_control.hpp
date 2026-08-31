#pragma once

#include <atomic>
#include <cstdint>

namespace general_planner::state2state_task {

// State2state is invoked from a dedicated runtime queue.  The command queue
// must be able to retire a timed-out task without starting a second planner
// invocation.  This control is deliberately small and lock-free: individual
// frontend/backend stages poll it and return normally, releasing replan_lock.
enum class State2StatePlanningStage : std::uint8_t {
    IDLE = 0,
    INPUT,
    TOPOLOGY_QUERY,
    TOPOLOGY_ATTACH,
    TOPOLOGY_PREFIX,
    LOCAL_FRONTEND,
    EXP_TRAJECTORY,
    BACKUP_TRAJECTORY,
    COMMIT
};

inline const char *state2StatePlanningStageName(
        const State2StatePlanningStage stage) {
    switch (stage) {
        case State2StatePlanningStage::IDLE: return "idle";
        case State2StatePlanningStage::INPUT: return "input";
        case State2StatePlanningStage::TOPOLOGY_QUERY: return "topology_query";
        case State2StatePlanningStage::TOPOLOGY_ATTACH: return "topology_attach";
        case State2StatePlanningStage::TOPOLOGY_PREFIX: return "topology_prefix";
        case State2StatePlanningStage::LOCAL_FRONTEND: return "local_frontend";
        case State2StatePlanningStage::EXP_TRAJECTORY: return "exp_trajectory";
        case State2StatePlanningStage::BACKUP_TRAJECTORY: return "backup_trajectory";
        case State2StatePlanningStage::COMMIT: return "commit";
    }
    return "unknown";
}

class State2StatePlanningControl {
public:
    void begin() {
        cancel_requested_.store(false, std::memory_order_release);
        setStage(State2StatePlanningStage::INPUT);
    }

    void requestCancel() {
        cancel_requested_.store(true, std::memory_order_release);
    }

    bool cancelRequested() const {
        return cancel_requested_.load(std::memory_order_acquire);
    }

    void setStage(const State2StatePlanningStage stage) {
        stage_.store(static_cast<std::uint8_t>(stage), std::memory_order_release);
    }

    State2StatePlanningStage stage() const {
        return static_cast<State2StatePlanningStage>(
            stage_.load(std::memory_order_acquire));
    }

    void finish() {
        setStage(State2StatePlanningStage::IDLE);
    }

private:
    std::atomic<bool> cancel_requested_{false};
    std::atomic<std::uint8_t> stage_{
        static_cast<std::uint8_t>(State2StatePlanningStage::IDLE)};
};

} // namespace general_planner::state2state_task
