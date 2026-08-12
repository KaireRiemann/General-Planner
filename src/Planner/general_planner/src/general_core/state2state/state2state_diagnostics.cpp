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
            << ";z_floor_enabled=" << static_cast<int>(d.z_floor_enabled)
            << ";z_floor_anchor=" << d.z_floor_anchor
            << ";z_floor_reference=" << d.z_floor_reference
            << ";z_floor_tolerance=" << d.z_floor_tolerance
            << ";z_floor_min_margin=" << d.z_floor_min_margin;
        return oss.str();
    }

    std::string GeneralPlanner::getLatestState2StateTopologyDebugInfo() const {
        std::lock_guard<std::mutex> lock(replan_lock_);
        const auto &runtime = state2state_topology_route_runtime_;
        const auto &route = runtime.route;
        std::ostringstream oss;
        oss << ";topology_policy_enabled="
            << static_cast<int>(runtime.policy_enabled.load(std::memory_order_acquire))
            << ";topology_route_valid=" << static_cast<int>(route.valid)
            << ";topology_route_id=" << route.route_id
            << ";topology_route_points=" << route.raw_topology_route.size()
            << ";topology_route_reaches_goal=" << static_cast<int>(route.reaches_goal)
            << ";topology_route_task_epoch=" << route.task_epoch
            << ";topology_route_world_epoch=" << route.world_epoch
            << ";topology_route_map_revision=" << route.map_revision_at_query
            << ";topology_route_revision=" << route.topo_revision
            << ";topology_route_committed_s=" << route.committed_route_s
            << ";topology_route_result=" << route.last_result;
        return oss.str();
    }

}
