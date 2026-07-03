#pragma once

#include <algorithm>
#include <string>

#include <general_core/nhbp/navigation_memory.hpp>

namespace general_planner::nhbp {

struct GuideConsistencyContext {
    bool enabled{true};
    bool recovery_mode{false};
    NdoState ndo_state{NdoState::STABLE};
    std::string guide_path_key;
    double weight_scale{1.0};
    double lateral_tube_radius{0.0};
    double vertical_tube_radius{0.0};
    std::string reason{"stable"};
};

class GuideConsistencyPolicy {
public:
    struct Config {
        bool enable{true};
        double suspect_weight_scale{0.5};
        double deadlocked_weight_scale{0.0};
        double unsafe_weight_scale{0.0};
        double recovery_weight_scale{1.0};
        double default_lateral_tube_radius{0.0};
        double default_vertical_tube_radius{0.0};
    };

    GuideConsistencyPolicy()
            : config_(Config{})
    {
    }

    explicit GuideConsistencyPolicy(Config config)
            : config_(config)
    {
    }

    GuideConsistencyContext decide(const NdoDiagnosis &ndo,
                                   bool recovery_mode,
                                   bool topology_edge_valid,
                                   bool safety_warning,
                                   const std::string &guide_path_key) const
    {
        GuideConsistencyContext context;
        context.enabled = config_.enable;
        context.recovery_mode = recovery_mode;
        context.ndo_state = ndo.state;
        context.guide_path_key = guide_path_key;
        context.lateral_tube_radius = std::max(0.0, config_.default_lateral_tube_radius);
        context.vertical_tube_radius = std::max(0.0, config_.default_vertical_tube_radius);

        if (!config_.enable || guide_path_key.empty()) {
            context.enabled = false;
            context.weight_scale = 0.0;
            context.reason = !config_.enable ? "guide_policy_disabled" : "empty_guide_key";
            return context;
        }
        if (safety_warning) {
            context.weight_scale = std::clamp(config_.unsafe_weight_scale, 0.0, 1.0);
            context.enabled = context.weight_scale > 0.0;
            context.reason = "safety_warning_reduce_guide";
            return context;
        }
        if (!topology_edge_valid) {
            context.weight_scale = 0.0;
            context.enabled = false;
            context.reason = "topology_edge_not_valid";
            return context;
        }
        if (recovery_mode) {
            context.weight_scale = std::clamp(config_.recovery_weight_scale, 0.0, 1.0);
            context.enabled = context.weight_scale > 0.0;
            context.reason = "recovery_guide";
            return context;
        }
        if (ndo.state == NdoState::DEADLOCKED) {
            context.weight_scale = std::clamp(config_.deadlocked_weight_scale, 0.0, 1.0);
            context.enabled = context.weight_scale > 0.0;
            context.reason = "ndo_deadlocked_disable_old_guide";
            return context;
        }
        if (ndo.state == NdoState::SUSPECT) {
            context.weight_scale = std::clamp(config_.suspect_weight_scale, 0.0, 1.0);
            context.enabled = context.weight_scale > 0.0;
            context.reason = "ndo_suspect_reduce_guide";
            return context;
        }
        context.weight_scale = 1.0;
        context.reason = "stable";
        return context;
    }

private:
    Config config_;
};

} // namespace general_planner::nhbp
