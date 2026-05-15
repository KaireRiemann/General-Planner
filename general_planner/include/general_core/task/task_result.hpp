#pragma once

#include <string>

#include "data_structure/exp_traj.h"
#include "utils/header/type_utils.hpp"

namespace general_planner {

using RET_CODE = super_utils::RET_CODE;

enum class TaskStatus {
    IDLE,
    RUNNING,
    NOT_READY,
    CANDIDATE_REJECTED,
    KEEP_CURRENT,
    COMMIT_TRAJECTORY,
    FINISHED,
    FAILED_RECOVERABLE,
    FAILED_FATAL,
    EMERGENCY
};

struct TaskCommand {
    bool has_trajectory{false};
    ExpTraj exp_traj;
    std::string traj_ns;
};

struct TaskTickResult {
    TaskStatus status{TaskStatus::IDLE};
    TaskCommand command;
    RET_CODE legacy_ret{super_utils::FAILED};
    std::string reason;

    static TaskTickResult fromRetCode(RET_CODE ret, const std::string &reason) {
        TaskTickResult result;
        result.legacy_ret = ret;
        result.reason = reason;
        switch (ret) {
            case super_utils::SUCCESS:
            case super_utils::NO_NEED:
            case super_utils::NEW_TRAJ:
                result.status = TaskStatus::RUNNING;
                break;
            case super_utils::FINISH:
                result.status = TaskStatus::FINISHED;
                break;
            case super_utils::EMER:
                result.status = TaskStatus::EMERGENCY;
                break;
            case super_utils::FAILED:
            default:
                result.status = TaskStatus::FAILED_RECOVERABLE;
                break;
        }
        return result;
    }
};

inline const char *taskStatusName(const TaskStatus status) {
    switch (status) {
        case TaskStatus::IDLE:
            return "IDLE";
        case TaskStatus::RUNNING:
            return "RUNNING";
        case TaskStatus::NOT_READY:
            return "NOT_READY";
        case TaskStatus::CANDIDATE_REJECTED:
            return "CANDIDATE_REJECTED";
        case TaskStatus::KEEP_CURRENT:
            return "KEEP_CURRENT";
        case TaskStatus::COMMIT_TRAJECTORY:
            return "COMMIT_TRAJECTORY";
        case TaskStatus::FINISHED:
            return "FINISHED";
        case TaskStatus::FAILED_RECOVERABLE:
            return "FAILED_RECOVERABLE";
        case TaskStatus::FAILED_FATAL:
            return "FAILED_FATAL";
        case TaskStatus::EMERGENCY:
            return "EMERGENCY";
    }
    return "UNKNOWN";
}

inline const char *legacyRetCodeName(const RET_CODE ret) {
    const int idx = static_cast<int>(ret);
    if (idx >= 0 && idx < static_cast<int>(super_utils::RET_CODE_STR.size())) {
        return super_utils::RET_CODE_STR[static_cast<std::size_t>(idx)].c_str();
    }
    return "UNKNOWN";
}

} // namespace general_planner
