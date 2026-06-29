#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <general_core/general_planner.h>

namespace general_planner::architecture {

class PlannerContext {
public:
    using DiagnosticCallback =
            std::function<void(const std::string &level,
                               const std::string &event,
                               const std::string &detail,
                               int ret_code)>;

    PlannerContext() = default;

    PlannerContext(GeneralPlanner::Ptr planner,
                   DiagnosticCallback diagnostic_callback = {})
        : planner_(std::move(planner)),
          diagnostic_callback_(std::move(diagnostic_callback)) {}

    bool valid() const {
        return static_cast<bool>(planner_);
    }

    GeneralPlanner &planner() const {
        return *planner_;
    }

    GeneralPlanner::Ptr plannerPtr() const {
        return planner_;
    }

    MapManager::Ptr mapManager() const {
        return planner_ ? planner_->getMapManager() : nullptr;
    }

    void recordDiagnostic(const std::string &level,
                          const std::string &event,
                          const std::string &detail = "",
                          const int ret_code = -1) const {
        if (diagnostic_callback_) {
            diagnostic_callback_(level, event, detail, ret_code);
        }
    }

private:
    GeneralPlanner::Ptr planner_;
    DiagnosticCallback diagnostic_callback_;
};

} // namespace general_planner::architecture
