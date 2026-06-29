#pragma once

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace general_planner::architecture {

enum class MissionMode {
    IDLE,
    SINGLE_TASK,
    TRACKING_MISSION,
    EXPLORATION_MISSION,
    PERCHING_MISSION
};

enum class TaskType {
    STATE_TO_STATE,
    TRACKING,
    PERCHING,
    EXPLORATION,
    TAKEOFF
};

enum class BackendType {
    AUTO,
    CORRIDOR,
    ESDF,
    PLAIN,
    JERK_TRACKING,
    SNAP_TRACKING,
    SE3
};

enum class ExecutionPhase {
    WAITING_INPUT,
    PLANNING,
    EXECUTING,
    HOLDING,
    RECOVERING,
    EMERGENCY
};

enum class SafetyAction {
    ACCEPT,
    ACCEPT_WITH_WARNING,
    RETRY_RELAXED,
    KEEP_CURRENT,
    HOLD_CURRENT,
    REPLAN,
    EMERGENCY_STOP
};

enum class SafetySeverity {
    INFO,
    WARN,
    ERROR
};

struct TaskIdentity {
    MissionMode mission{MissionMode::IDLE};
    TaskType task{TaskType::STATE_TO_STATE};
    BackendType backend{BackendType::AUTO};
    std::string plugin_name{"state2state"};
    bool goal_like{true};
    bool tracking_like{false};
};

struct TaskPlanContext {
    bool handled{false};
    bool missing_input{false};
    bool tracking_context{false};
    bool tracking_prediction_static{false};
    std::size_t tracking_input_prediction_size{0};
    std::string mission_node;
    std::string decision_detail;
};

struct TaskPluginDescriptor {
    TaskIdentity identity;
    std::vector<BackendType> allowed_backends;
    bool continuous_replan{false};
};

struct BackendDescriptor {
    BackendType type{BackendType::AUTO};
    std::string name{"auto"};
    std::vector<TaskType> supported_tasks;
    bool requires_map{false};
    bool supports_continuous_replan{true};
};

inline std::string canonicalizeToken(std::string token) {
    std::transform(token.begin(), token.end(), token.begin(), [](unsigned char c) {
        const char lowered = static_cast<char>(std::tolower(c));
        return lowered == '-' ? '_' : lowered;
    });
    return token;
}

inline const char *toString(const MissionMode mode) {
    switch (mode) {
        case MissionMode::IDLE:
            return "idle";
        case MissionMode::TRACKING_MISSION:
            return "tracking_mission";
        case MissionMode::EXPLORATION_MISSION:
            return "exploration_mission";
        case MissionMode::PERCHING_MISSION:
            return "perching_mission";
        case MissionMode::SINGLE_TASK:
        default:
            return "single_task";
    }
}

inline const char *toString(const TaskType task) {
    switch (task) {
        case TaskType::TRACKING:
            return "tracking";
        case TaskType::PERCHING:
            return "perching";
        case TaskType::EXPLORATION:
            return "exploration";
        case TaskType::TAKEOFF:
            return "takeoff";
        case TaskType::STATE_TO_STATE:
        default:
            return "state2state";
    }
}

inline const char *toString(const BackendType backend) {
    switch (backend) {
        case BackendType::CORRIDOR:
            return "corridor";
        case BackendType::ESDF:
            return "esdf";
        case BackendType::PLAIN:
            return "plain";
        case BackendType::JERK_TRACKING:
            return "jerk_tracking";
        case BackendType::SNAP_TRACKING:
            return "snap_tracking";
        case BackendType::SE3:
            return "se3";
        case BackendType::AUTO:
        default:
            return "auto";
    }
}

inline const char *toString(const ExecutionPhase phase) {
    switch (phase) {
        case ExecutionPhase::WAITING_INPUT:
            return "waiting_input";
        case ExecutionPhase::PLANNING:
            return "planning";
        case ExecutionPhase::EXECUTING:
            return "executing";
        case ExecutionPhase::HOLDING:
            return "holding";
        case ExecutionPhase::RECOVERING:
            return "recovering";
        case ExecutionPhase::EMERGENCY:
            return "emergency";
        default:
            return "waiting_input";
    }
}

inline const char *toString(const SafetyAction action) {
    switch (action) {
        case SafetyAction::ACCEPT_WITH_WARNING:
            return "accept_with_warning";
        case SafetyAction::RETRY_RELAXED:
            return "retry_relaxed";
        case SafetyAction::KEEP_CURRENT:
            return "keep_current";
        case SafetyAction::HOLD_CURRENT:
            return "hold_current";
        case SafetyAction::REPLAN:
            return "replan";
        case SafetyAction::EMERGENCY_STOP:
            return "emergency_stop";
        case SafetyAction::ACCEPT:
        default:
            return "accept";
    }
}

inline const char *toString(const SafetySeverity severity) {
    switch (severity) {
        case SafetySeverity::WARN:
            return "WARN";
        case SafetySeverity::ERROR:
            return "ERROR";
        case SafetySeverity::INFO:
        default:
            return "INFO";
    }
}

inline std::optional<BackendType> backendTypeFromLegacyMode(const std::string &mode) {
    const std::string normalized = canonicalizeToken(mode);
    if (normalized == "corridor") {
        return BackendType::CORRIDOR;
    }
    if (normalized == "esdf") {
        return BackendType::ESDF;
    }
    if (normalized == "plain") {
        return BackendType::PLAIN;
    }
    if (normalized == "se3" || normalized == "se3_aggressive" ||
        normalized == "aggressive" || normalized == "racing") {
        return BackendType::SE3;
    }
    return std::nullopt;
}

inline BackendType backendTypeFromString(const std::string &backend) {
    const std::string normalized = canonicalizeToken(backend);
    if (normalized == "corridor" || normalized == "sfc") {
        return BackendType::CORRIDOR;
    }
    if (normalized == "esdf") {
        return BackendType::ESDF;
    }
    if (normalized == "plain" || normalized == "unconstrained") {
        return BackendType::PLAIN;
    }
    if (normalized == "tracking_jerk" || normalized == "jerk_tracking" || normalized == "jerk") {
        return BackendType::JERK_TRACKING;
    }
    if (normalized == "tracking_snap" || normalized == "snap_tracking" || normalized == "snap") {
        return BackendType::SNAP_TRACKING;
    }
    if (normalized == "se3" || normalized == "se3_aggressive") {
        return BackendType::SE3;
    }
    return BackendType::AUTO;
}

inline MissionMode missionModeForTask(const TaskType task) {
    switch (task) {
        case TaskType::TRACKING:
            return MissionMode::TRACKING_MISSION;
        case TaskType::PERCHING:
            return MissionMode::PERCHING_MISSION;
        case TaskType::EXPLORATION:
            return MissionMode::EXPLORATION_MISSION;
        case TaskType::TAKEOFF:
            return MissionMode::PERCHING_MISSION;
        case TaskType::STATE_TO_STATE:
        default:
            return MissionMode::SINGLE_TASK;
    }
}

inline BackendType defaultBackendForTask(const TaskType task) {
    switch (task) {
        case TaskType::TRACKING:
            return BackendType::JERK_TRACKING;
        case TaskType::STATE_TO_STATE:
            return BackendType::CORRIDOR;
        case TaskType::PERCHING:
        case TaskType::EXPLORATION:
        case TaskType::TAKEOFF:
        default:
            return BackendType::AUTO;
    }
}

inline bool isGoalLikeTask(const TaskType task) {
    return task == TaskType::STATE_TO_STATE;
}

inline bool isTrackingLikeTask(const TaskType task, const MissionMode mission) {
    return task == TaskType::TRACKING ||
           mission == MissionMode::TRACKING_MISSION;
}

} // namespace general_planner::architecture
