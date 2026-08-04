/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>

#include <sstream>

namespace general_planner {

    std::string GeneralPlanner::getLatestState2StateZDebugInfo() const {
        if (!latest_state2state_z_debug_.valid) {
            return "";
        }
        const auto &d = latest_state2state_z_debug_;
        std::ostringstream oss;
        oss << ";z_debug_valid=1"
            << ";exp_mode=" << d.exp_mode
            << ";goal_z=" << d.goal_z
            << ";robot_z=" << d.robot_z
            << ";guide_size=" << d.guide_size
            << ";guide_z_valid=" << static_cast<int>(d.guide.valid)
            << ";guide_z_start=" << d.guide.start
            << ";guide_z_end=" << d.guide.end
            << ";guide_z_min=" << d.guide.min
            << ";guide_z_max=" << d.guide.max
            << ";local_target_z=" << d.local_target_z
            << ";local_target_goal_z_err=" << d.local_target_goal_z_err
            << ";local_target_goal_dist=" << d.local_target_goal_dist
            << ";local_target_goal_xy_dist=" << d.local_target_goal_xy_dist
            << ";local_target_is_goal=" << static_cast<int>(d.local_target_is_global_goal)
            << ";opt_z_valid=" << static_cast<int>(d.optimized.valid)
            << ";opt_z_start=" << d.optimized.start
            << ";opt_z_end=" << d.optimized.end
            << ";opt_z_min=" << d.optimized.min
            << ";opt_z_max=" << d.optimized.max
            << ";opt_end_local_target_z_err=" << d.opt_end_local_target_z_err
            << ";exp_full_z_valid=" << static_cast<int>(d.exp_full.valid)
            << ";exp_full_z_start=" << d.exp_full.start
            << ";exp_full_z_end=" << d.exp_full.end
            << ";exp_full_z_min=" << d.exp_full.min
            << ";exp_full_z_max=" << d.exp_full.max
            << ";guide_reused_command_prefix=" << static_cast<int>(d.guide_reused_command_prefix)
            << ";z_lower_guard_enabled=" << static_cast<int>(d.z_lower_guard_enabled)
            << ";z_lower_guard_passed=" << static_cast<int>(d.z_lower_guard_passed)
            << ";z_lower_guard_samples=" << d.z_lower_guard_samples
            << ";z_lower_guard_tolerance=" << d.z_lower_guard_tolerance
            << ";z_lower_guard_min_margin=" << d.z_lower_guard_min_margin
            << ";z_lower_guard_max_violation=" << d.z_lower_guard_max_violation;
        return oss.str();
    }

}
