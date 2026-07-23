/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>
#include <general_core/state2state/state2state_frontend_services.hpp>
#include <general_core/state2state/state2state_path_utils.hpp>
#include <algorithm>
#include <cmath>
#include <fmt/color.h>

using namespace general_utils;

namespace general_planner {

namespace state2state_task {

    bool prepareESDFGuideEndpoint(StateToStateFrontendServices &services,
                                  vec_Vec3f &guide_path,
                                  std::vector<double> &guide_stamp) {
        if (!services.cfg.esdf_traj_en || services.cfg.plain_traj_en ||
            services.map_manager == nullptr || !services.map_manager->hasESDF()) {
            return true;
        }
        if (guide_path.size() < 2 || guide_path.size() != guide_stamp.size()) {
            return false;
        }

        const double hard_required_dist = 0.5 * services.cfg.esdf_safe_distance;
        auto is_inflated_free = [&](const Vec3f &pos) {
            const auto inf_type = services.map_manager->getInfGridType(pos);
            return inf_type != rog_map::GridType::OCCUPIED &&
                   inf_type != rog_map::GridType::OUT_OF_MAP;
        };
        auto is_esdf_hard_safe = [&](const Vec3f &pos, double &dist) {
            Vec3f grad = Vec3f::Zero();
            if (!services.map_manager->evaluateESDF(pos, dist, grad)) {
                dist = -1.0;
                return false;
            }
            return std::isfinite(dist) && dist >= hard_required_dist;
        };
        auto is_guide_point_safe = [&](const Vec3f &pos, double &dist) {
            return is_inflated_free(pos) && is_esdf_hard_safe(pos, dist);
        };
        auto refresh_guide_stamp = [&]() {
            if (guide_path.size() != guide_stamp.size()) {
                guide_stamp.resize(guide_path.size(), 0.0);
            }
            for (size_t i = 1; i < guide_path.size(); ++i) {
                const double dt = (guide_path[i] - guide_path[i - 1]).norm() /
                                  std::max(1.0e-3, services.cfg.exp_traj_cfg.max_vel);
                guide_stamp[i] = guide_stamp[i - 1] + std::max(0.05, dt);
            }
        };

        const double repair_radius = std::clamp(2.0 * services.cfg.esdf_safe_distance, 0.8, 1.5);
        int repaired_mid_points = 0;
        int unresolved_mid_points = 0;
        for (size_t i = 1; i + 1 < guide_path.size(); ++i) {
            double dist = 0.0;
            if (is_guide_point_safe(guide_path[i], dist)) {
                continue;
            }

            Vec3f shifted_point = guide_path[i];
            if (services.map_manager->findNearestESDFSafe(guide_path[i],
                                                  hard_required_dist,
                                                  shifted_point,
                                                  repair_radius)) {
                guide_path[i] = shifted_point;
                ++repaired_mid_points;
            } else {
                ++unresolved_mid_points;
            }
        }

        int repaired_segments = 0;
        int unresolved_segments = 0;
        const int max_segment_repair_iter = 3;
        const size_t max_guide_points = 96;
        for (int iter = 0; iter < max_segment_repair_iter && guide_path.size() < max_guide_points; ++iter) {
            bool inserted = false;
            for (size_t i = 0; i + 1 < guide_path.size() && guide_path.size() < max_guide_points; ++i) {
                if (services.map_manager->isLineFree(guide_path[i], guide_path[i + 1], true, false)) {
                    continue;
                }

                const Vec3f mid_point = 0.5 * (guide_path[i] + guide_path[i + 1]);
                Vec3f safe_mid = mid_point;
                if (services.map_manager->findNearestESDFSafe(mid_point,
                                                      hard_required_dist,
                                                      safe_mid,
                                                      repair_radius)) {
                    guide_path.insert(guide_path.begin() + static_cast<long>(i + 1), safe_mid);
                    guide_stamp.insert(guide_stamp.begin() + static_cast<long>(i + 1), guide_stamp[i]);
                    ++repaired_segments;
                    inserted = true;
                    ++i;
                } else {
                    ++unresolved_segments;
                }
            }
            if (!inserted) {
                break;
            }
        }
        if (repaired_mid_points > 0 || repaired_segments > 0) {
            refresh_guide_stamp();
            if (services.cfg.print_log) {
                services.ros_ptr->warn(" -- [GeneralPlanner] ESDF guide repaired: mid_points={}, segments={}, unresolved_mid={}, unresolved_segments={}, guide_size={}.",
                               repaired_mid_points,
                               repaired_segments,
                               unresolved_mid_points,
                               unresolved_segments,
                               guide_path.size());
            }
        } else if ((unresolved_mid_points > 0 || unresolved_segments > 0) && services.cfg.print_log) {
            services.ros_ptr->warn(" -- [GeneralPlanner] ESDF guide has unresolved unsafe samples: mid_points={}, segments={}.",
                           unresolved_mid_points,
                           unresolved_segments);
        }

        double endpoint_dist = 0.0;
        const bool endpoint_inflated_free = is_inflated_free(guide_path.back());
        const bool endpoint_esdf_ready = is_esdf_hard_safe(guide_path.back(), endpoint_dist);
        if (endpoint_inflated_free && endpoint_esdf_ready) {
            return true;
        }

        const bool connected_global_goal =
                (guide_path.back() - services.goal_p).norm() < 2.0 * services.cfg.resolution;
        if (connected_global_goal && endpoint_inflated_free) {
            return true;
        }

        if (endpoint_inflated_free && endpoint_esdf_ready) {
            return true;
        }

        const Vec3f original_endpoint = guide_path.back();
        Vec3f shifted_endpoint = original_endpoint;
        if (services.map_manager->findNearestESDFSafe(original_endpoint, hard_required_dist, shifted_endpoint, 0.8)) {
            const Vec3f prev_pt = guide_path[guide_path.size() - 2];
            if (services.map_manager->isLineFree(prev_pt, shifted_endpoint, true, false)) {
                double shifted_dist = 0.0;
                Vec3f shifted_grad = Vec3f::Zero();
                services.map_manager->evaluateESDF(shifted_endpoint, shifted_dist, shifted_grad);
                guide_path.back() = shifted_endpoint;
                guide_stamp.back() = guide_stamp[guide_stamp.size() - 2] +
                                     std::max(0.05, (shifted_endpoint - prev_pt).norm() / services.cfg.exp_traj_cfg.max_vel);
                services.ros_ptr->warn(" -- [GeneralPlanner] ESDF local endpoint hard clearance {} < {}, shift local endpoint from [{}, {}, {}] to [{}, {}, {}], shifted_dist={}.",
                               endpoint_dist,
                               hard_required_dist,
                               original_endpoint.x(), original_endpoint.y(), original_endpoint.z(),
                               shifted_endpoint.x(), shifted_endpoint.y(), shifted_endpoint.z(),
                               shifted_dist);
                return true;
            }
        }

        const int original_size = static_cast<int>(guide_path.size());
        for (int i = static_cast<int>(guide_path.size()) - 2; i >= 1; --i) {
            double dist = 0.0;
            if (!is_inflated_free(guide_path[i]) || !is_esdf_hard_safe(guide_path[i], dist)) {
                continue;
            }
            guide_path.resize(i + 1);
            guide_stamp.resize(i + 1);
            services.ros_ptr->warn(" -- [GeneralPlanner] ESDF local endpoint is unavailable or too close: dist={}, required={}. Truncate guide {} -> {}, new endpoint=[{}, {}, {}], new_dist={}.",
                           endpoint_dist,
                           hard_required_dist,
                           original_size,
                           guide_path.size(),
                           guide_path.back().x(), guide_path.back().y(), guide_path.back().z(),
                           dist);
            return (guide_path.back() - guide_path.front()).norm() > services.cfg.resolution * 2.0;
        }

        services.ros_ptr->warn(" -- [GeneralPlanner] Failed to find a valid ESDF local endpoint for the rolling guide. endpoint_dist={}, required={}.",
                       endpoint_dist,
                       hard_required_dist);
        return false;
    }

} // namespace state2state_task

namespace state2state_task {

    bool pathSearch(StateToStateFrontendServices &services,
                    const Vec3f &start_pt,
                    const Vec3f &goal,
                    const double searching_horizon,
                    vec_Vec3f &path) {
        using namespace path_search;
        if (searching_horizon <= 0.0) {
            services.ros_ptr->error(" -- [GeneralPlanner] Goal waypoints empty or searching horizon negative, force return.");
            return false;
        }

        // 1) check and shift pts
        // 		For start point, must be collision free
        rog_map::GridType start_type;
        start_type = services.map_manager->getGridType(start_pt);

        /// If the start_pt is obstacle in prob map, just shift it to the nearest free point.
        if (start_type == rog_map::GridType::OCCUPIED ||
            start_type == rog_map::GridType::OUT_OF_MAP) {
            services.ros_ptr->warn(
                    " -- [GeneralPlanner] The start point in obstacle, this should not happen since the start point should be shift before pathsearch.");
            return false;
        }
        vec_E<Vec3f> start_point_escape_path;

        int flag_es = ON_PROB_MAP | (services.cfg.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE);
        vec_Vec3f out_path;
        RET_CODE ret_es = services.astar->escapePathSearch(start_pt, flag_es, out_path);
        if (ret_es != NO_NEED) {
            if (ret_es != REACH_HORIZON && ret_es != REACH_GOAL) {
                services.ros_ptr->error(
                        " -- [GeneralPlanner] Escape path search failed with [{}], force return.",
                        RET_CODE_STR[ret_es].c_str());
                return false;
            } else {
                start_point_escape_path = out_path;
            }
        }

        Vec3f shifted_start_pt = start_pt;

        if (!start_point_escape_path.empty()) {
            shifted_start_pt = start_point_escape_path.back();
        }

        Vec3f temp_goal_point, temp_start_point;
        temp_start_point = shifted_start_pt;
        double temp_plannning_horizon = searching_horizon;
        //            int start_id = getNearestFurtherGoalPoint(goal_waypoints, start_pt);

        const bool goal_inside_local_map = services.map_manager->insideLocalMap(goal);
        const rog_map::GridType goal_inf_type =
                goal_inside_local_map ? services.map_manager->getInfGridType(goal) : OUT_OF_MAP;
        const bool hidden_unknown_goal = services.cfg.unknown_goal_reveal_en &&
                                         goal_inside_local_map &&
                                         goal_inf_type == UNKNOWN;
        const bool unknown_as_occupied_for_frontend = services.cfg.frontend_in_known_free || hidden_unknown_goal;
        if (hidden_unknown_goal && services.cfg.print_log) {
            services.ros_ptr->info(" -- [GeneralPlanner] Click goal is unknown; search a reveal/frontier waypoint first.");
        }

        auto pointUsable = [&](const Vec3f &point) {
            if (!point.allFinite() || !services.map_manager->insideLocalMap(point)) {
                return false;
            }
            if (services.dynamic_obstacle_layer != nullptr &&
                services.dynamic_obstacle_layer->enabled() &&
                services.dynamic_obstacle_layer->pointOccupied(point, services.ros_ptr->getSimTime())) {
                return false;
            }
            const rog_map::GridType inf_type = services.map_manager->getInfGridType(point);
            if (inf_type == OCCUPIED || inf_type == OUT_OF_MAP) {
                return false;
            }
            return !(unknown_as_occupied_for_frontend && inf_type == UNKNOWN);
        };

        auto lineUsable = [&](const Vec3f &a, const Vec3f &b) {
            return pointUsable(a) &&
                   pointUsable(b) &&
                   (services.dynamic_obstacle_layer == nullptr ||
                    !services.dynamic_obstacle_layer->enabled() ||
                    !services.dynamic_obstacle_layer->lineOccupied(a, b, services.ros_ptr->getSimTime())) &&
                   services.map_manager->isLineFree(a, b, true, unknown_as_occupied_for_frontend);
        };

        auto buildDirectLineCandidate = [&](vec_Vec3f &candidate, RET_CODE &candidate_ret) {
            candidate.clear();
            candidate_ret = FAILED;
            if (!services.cfg.state2state_direct_line_frontend_enable) {
                return false;
            }
            const Vec3f delta = goal - temp_start_point;
            const double dist = delta.norm();
            if (dist < 1.0e-4) {
                appendPathPointUnique(goal, candidate);
                candidate_ret = REACH_GOAL;
                return true;
            }
            const double usable_horizon = std::max(0.0, searching_horizon);
            const bool reaches_goal = dist <= usable_horizon + std::max(1.0e-3, services.cfg.resolution);
            const Vec3f direct_end = reaches_goal
                                         ? goal
                                         : temp_start_point + delta / dist * usable_horizon;
            if (!lineUsable(temp_start_point, direct_end)) {
                return false;
            }
            appendPathPointUnique(direct_end, candidate);
            candidate_ret = reaches_goal ? REACH_GOAL : REACH_HORIZON;
            return true;
        };

        auto buildTopologyCandidate = [&](vec_Vec3f &candidate,
                                          RET_CODE &candidate_ret) {
            candidate.clear();
            candidate_ret = FAILED;
            const double query_distance = (goal - temp_start_point).norm();
            if (!services.cfg.state2state_topology_query_enable ||
                !services.cfg.state2state_topology_enable ||
                !services.map_manager->topologyReady() ||
                query_distance < std::max(0.0,
                    services.cfg.state2state_topology_min_query_distance)) {
                return false;
            }

            // Match USS-Nav's build/query separation: planning only tells the
            // persistent map where the robot currently is. Expanding from the
            // robot (not a possibly unobserved goal midpoint) lets the graph
            // accumulate connected navigation memory along executed routes.
            services.map_manager->requestTopologyUpdateAround(temp_start_point);
            const auto topology_snapshot =
                services.map_manager->topologySearchSnapshot();
            if (!topology_snapshot || topology_snapshot->graph.empty()) {
                return false;
            }

            vec_Vec3f topology_path;
            auto routeUsable = [&](const vec_Vec3f &route) {
                if (route.size() < 2) {
                    return false;
                }
                for (std::size_t i = 1; i < route.size(); ++i) {
                    if (!lineUsable(route[i - 1], route[i])) {
                        return false;
                    }
                }
                return true;
            };

            bool reaches_goal = services.map_manager->findTopologyPath(
                                    topology_snapshot, temp_start_point, goal,
                                    topology_path) &&
                                routeUsable(topology_path);

            // A distant click goal can lie outside the rolling local map. In
            // that case choose a reachable graph node which advances toward
            // it; the next replanning cycle continues over the persistent map.
            if (!reaches_goal) {
                struct RouteTarget {
                    Vec3f position;
                    double score{0.0};
                };
                std::vector<RouteTarget> targets;
                Vec3f direction = Vec3f::Zero();
                if (query_distance > 1.0e-9) {
                    direction = (goal - temp_start_point) / query_distance;
                }
                for (const auto &entry : topology_snapshot->graph) {
                    const auto &node = entry.second.node;
                    const Vec3f offset = node.position - temp_start_point;
                    const double radial_distance = offset.norm();
                    const double progress = offset.dot(direction);
                    if (progress <= services.cfg.resolution ||
                        radial_distance > searching_horizon * 1.35 ||
                        !pointUsable(node.position)) {
                        continue;
                    }
                    targets.push_back({node.position,
                                       progress - 0.15 * std::abs(
                                           radial_distance - searching_horizon)});
                }
                std::sort(targets.begin(), targets.end(),
                          [](const RouteTarget &lhs, const RouteTarget &rhs) {
                              return lhs.score > rhs.score;
                          });

                vec_Vec3f best_route;
                double best_progress = -1.0;
                // Targets are already ordered by goal progress. Stop at the
                // first safe route instead of ray-checking several equivalent
                // graph paths in the latency-sensitive frontend.
                const std::size_t attempts = std::min<std::size_t>(3, targets.size());
                const double required_length = 0.65 * std::min(
                    searching_horizon, query_distance);
                for (std::size_t i = 0; i < attempts; ++i) {
                    vec_Vec3f route;
                    if (!services.map_manager->findTopologyPath(
                            topology_snapshot, temp_start_point,
                            targets[i].position, route) ||
                        !routeUsable(route)) {
                        continue;
                    }
                    double route_length = 0.0;
                    for (std::size_t j = 1; j < route.size(); ++j) {
                        route_length += (route[j] - route[j - 1]).norm();
                    }
                    const double progress =
                        (route.back() - temp_start_point).dot(direction);
                    if (route_length + services.cfg.resolution >= required_length &&
                        progress > best_progress) {
                        best_progress = progress;
                        best_route = std::move(route);
                        break;
                    }
                }
                topology_path = std::move(best_route);
                if (topology_path.size() < 2) {
                    return false;
                }
            }

            appendPathPointUnique(temp_start_point, candidate);
            double remaining = std::max(0.0, searching_horizon);
            for (std::size_t i = 1; i < topology_path.size(); ++i) {
                const Vec3f segment = topology_path[i] - topology_path[i - 1];
                const double length = segment.norm();
                if (length <= 1.0e-9) {
                    continue;
                }
                if (length <= remaining + 1.0e-9) {
                    appendPathPointUnique(topology_path[i], candidate);
                    remaining -= length;
                    continue;
                }
                appendPathPointUnique(topology_path[i - 1] +
                                      segment * (remaining / length), candidate);
                candidate_ret = REACH_HORIZON;
                return candidate.size() >= 2;
            }

            candidate_ret = reaches_goal ? REACH_GOAL : REACH_HORIZON;
            if (reaches_goal) {
                appendPathPointUnique(goal, candidate);
            }
            return candidate.size() >= 2;
        };

        int flag = ON_INF_MAP |
                   (unknown_as_occupied_for_frontend ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) |
                   DONT_USE_INF_NEIGHBOR;

        vec_Vec3f normal_path;
        RET_CODE ret_code = FAILED;
        const bool direct_line_frontend =
                buildDirectLineCandidate(normal_path, ret_code);
        const bool topology_frontend = !direct_line_frontend &&
                buildTopologyCandidate(normal_path, ret_code);
        if (!direct_line_frontend && !topology_frontend) {
            ret_code = services.astar->pointToPointPathSearch(temp_start_point,
                                                          goal,
                                                          flag,
                                                          temp_plannning_horizon,
                                                          normal_path,
                                                          services.cfg.frontend_astar_time_out);
        } else if (services.cfg.print_log && direct_line_frontend) {
            services.ros_ptr->info(" -- [GeneralPlanner] Use direct-line frontend candidate: ret={}.",
                           RET_CODE_STR[ret_code]);
        } else if (services.cfg.print_log && topology_frontend) {
            services.ros_ptr->info(
                " -- [GeneralPlanner] Use incremental-topology frontend candidate: ret={}, points={}.",
                RET_CODE_STR[ret_code], normal_path.size());
        }

        if(ret_code == INIT_ERROR){
            services.goal_valid = false;
            return false;
        }
        //add may23, if failed on inf map, use prob map try again

        const bool distance_field_frontend = services.cfg.esdf_traj_en || services.cfg.plain_traj_en;
        if (!direct_line_frontend && !topology_frontend &&
            ret_code == NO_PATH && !distance_field_frontend) {
            flag = ON_PROB_MAP |
                   (unknown_as_occupied_for_frontend ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) |
                   USE_INF_NEIGHBOR;
            fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                       " -- [Astar] Path search failed on inf map, try again on prob map.\n");
            ret_code = services.astar->pointToPointPathSearch(temp_start_point, goal, flag, temp_plannning_horizon,
                                                          normal_path, services.cfg.frontend_astar_time_out);
            if (ret_code == SUCCESS || ret_code == REACH_HORIZON || ret_code == REACH_GOAL) {
                fmt::print(fg(fmt::color::lime_green) | fmt::emphasis::bold,
                           " -- [Astar] Path search on prob map success.\n");
            } else {
                fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                           " -- [Astar] Path search failed on prob map still failed.\n");
            }
        } else if (!direct_line_frontend && !topology_frontend &&
                   ret_code == NO_PATH && distance_field_frontend) {
            fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                       " -- [Astar] Path search failed on inf map in distance-field mode; skip prob-map fallback.\n");
        }

        auto blockedSpanOnDirectLine = [&]() {
            const Vec3f delta = goal - temp_start_point;
            const double len = std::min(delta.norm(), std::max(0.0, searching_horizon));
            if (len < 1.0e-4) {
                return 0.0;
            }
            const Vec3f dir = delta / std::max(1.0e-6, delta.norm());
            const double step = std::max(0.2, services.map_manager->getInfResolution());
            double blocked_span = 0.0;
            double current_span = 0.0;
            for (double s = 0.0; s <= len + 1.0e-6; s += step) {
                const Vec3f sample = temp_start_point + dir * std::min(s, len);
                const bool blocked = !pointUsable(sample);
                if (blocked) {
                    current_span += step;
                    blocked_span = std::max(blocked_span, current_span);
                } else {
                    current_span = 0.0;
                }
            }
            return blocked_span;
        };

        auto buildOverWallCandidate = [&](vec_Vec3f &candidate, RET_CODE &candidate_ret) {
            candidate.clear();
            candidate_ret = FAILED;
            if (!services.cfg.over_wall_search_en) {
                return false;
            }

            const Vec3f delta = goal - temp_start_point;
            const double goal_dist = delta.norm();
            if (goal_dist < std::max(0.5, services.cfg.resolution * 5.0)) {
                return false;
            }
            const Vec3f horizontal_delta(delta.x(), delta.y(), 0.0);
            const double horizontal_dist = horizontal_delta.norm();
            if (horizontal_dist < 1.0e-3) {
                return false;
            }
            const double blocked_span = blockedSpanOnDirectLine();
            if (blocked_span < std::max(0.0, services.cfg.over_wall_min_blocked_span) &&
                ret_code == REACH_GOAL) {
                return false;
            }

            const Vec3f dir_xy = horizontal_delta / horizontal_dist;
            const double max_climb = std::max(0.0, services.cfg.over_wall_max_climb);
            const double height_step = std::max(services.map_manager->getInfResolution(), services.cfg.over_wall_height_step);
            const double base_z = std::max(temp_start_point.z(), goal.z());
            const double forward_ratio = std::clamp(services.cfg.over_wall_forward_ratio, 0.2, 1.0);

            for (double climb = height_step; climb <= max_climb + 1.0e-6; climb += height_step) {
                const double level_z = base_z + climb;
                const Vec3f elevated_start(temp_start_point.x(), temp_start_point.y(), level_z);
                if (!lineUsable(temp_start_point, elevated_start)) {
                    continue;
                }

                const double goal_detour = (elevated_start - temp_start_point).norm() +
                                           (Vec3f(goal.x(), goal.y(), level_z) - elevated_start).norm() +
                                           std::abs(level_z - goal.z());
                if (goal_detour <= searching_horizon * 1.15) {
                    const Vec3f elevated_goal(goal.x(), goal.y(), level_z);
                    if (lineUsable(elevated_start, elevated_goal) &&
                        lineUsable(elevated_goal, goal)) {
                        appendPathPointUnique(temp_start_point, candidate);
                        appendPathPointUnique(elevated_start, candidate);
                        appendPathPointUnique(elevated_goal, candidate);
                        appendPathPointUnique(goal, candidate);
                        candidate_ret = REACH_GOAL;
                        return true;
                    }
                }

                double forward_dist = std::min(horizontal_dist,
                                               std::max(0.0, searching_horizon - climb) * forward_ratio);
                while (forward_dist > std::max(0.5, services.cfg.resolution * 5.0)) {
                    const Vec3f elevated_forward = elevated_start + dir_xy * forward_dist;
                    if (lineUsable(elevated_start, elevated_forward)) {
                        appendPathPointUnique(temp_start_point, candidate);
                        appendPathPointUnique(elevated_start, candidate);
                        appendPathPointUnique(elevated_forward, candidate);
                        candidate_ret = REACH_HORIZON;
                        return true;
                    }
                    forward_dist -= std::max(0.5, 2.0 * services.map_manager->getInfResolution());
                }
            }
            return false;
        };

        vec_Vec3f selected_path = normal_path;
        RET_CODE selected_ret = ret_code;
        vec_Vec3f over_wall_path;
        RET_CODE over_wall_ret = FAILED;
        if (buildOverWallCandidate(over_wall_path, over_wall_ret)) {
            const Vec3f goal_dir = (goal - temp_start_point).norm() > 1.0e-6
                                       ? (goal - temp_start_point).normalized()
                                       : Vec3f::Zero();
            const double normal_progress = pathForwardProgress(normal_path, temp_start_point, goal_dir);
            const double over_wall_progress = pathForwardProgress(over_wall_path, temp_start_point, goal_dir);
            const bool normal_failed = ret_code != REACH_HORIZON && ret_code != REACH_GOAL;
            const bool over_reaches_goal = over_wall_ret == REACH_GOAL && ret_code != REACH_GOAL;
            const bool progress_better =
                    over_wall_progress > normal_progress + std::max(0.0, services.cfg.over_wall_min_progress_gain);
            if (normal_failed || over_reaches_goal || (ret_code == REACH_HORIZON && progress_better)) {
                selected_path = over_wall_path;
                selected_ret = over_wall_ret;
                if (services.cfg.print_log) {
                    services.ros_ptr->info(" -- [GeneralPlanner] Use over-wall frontend candidate: ret={}, progress {:.2f}->{:.2f}.",
                                   RET_CODE_STR[over_wall_ret],
                                   normal_progress,
                                   over_wall_progress);
                }
            }
        }

        if (selected_ret != REACH_HORIZON && selected_ret != REACH_GOAL) {
            services.ros_ptr->error(
                    " -- [GeneralPlanner] Path search failed with [{}], force return.\n",
                    RET_CODE_STR[ret_code].c_str());
            return false;
        }

        auto assembleSelectedPath = [&]() {
            vec_Vec3f assembled;
            appendPathPointUnique(start_pt, assembled);
            for (const auto &point : start_point_escape_path) {
                appendPathPointUnique(point, assembled);
            }
            for (const auto &point : selected_path) {
                appendPathPointUnique(point, assembled);
            }
            if (selected_ret == REACH_GOAL) {
                appendPathPointUnique(goal, assembled);
            }
            return assembled;
        };

        path = assembleSelectedPath();
        if (path.size() < 2) {
            services.ros_ptr->warn(
                    " -- [GeneralPlanner] Path search failed with empty segments, force return.");
            return false;
        }

        auto shortcutPathByLineOfSight = [&](const vec_Vec3f &input) {
            if (input.size() <= 2) {
                return input;
            }
            vec_Vec3f shortcut;
            shortcut.reserve(input.size());
            std::size_t anchor = 0;
            appendPathPointUnique(input.front(), shortcut);
            while (anchor + 1 < input.size()) {
                std::size_t next = anchor + 1;
                for (std::size_t j = input.size() - 1; j > anchor; --j) {
                    if (lineUsable(input[anchor], input[j])) {
                        next = j;
                        break;
                    }
                }
                appendPathPointUnique(input[next], shortcut);
                anchor = next;
            }
            return shortcut;
        };

        const std::size_t raw_path_size = path.size();
        path = shortcutPathByLineOfSight(path);
        if (services.cfg.print_log && path.size() + 2 < raw_path_size) {
            services.ros_ptr->info(" -- [GeneralPlanner] Frontend line-of-sight shortcut: {} -> {} points.",
                           raw_path_size,
                           path.size());
        }

        if (services.cfg.state2state_over_goal_guard_enable) {
            const double near_goal_radius = std::max(services.cfg.resolution * 3.0,
                                                     services.cfg.state2state_near_goal_radius);
            const double near_goal_xy = (temp_start_point.head<2>() - goal.head<2>()).norm();
            const double start_over =
                    state2stateGoalOvershoot(temp_start_point,
                                             services.local_start_p,
                                             temp_start_point,
                                             goal);
            const double max_allowed_over =
                    std::max(std::max(0.0, services.cfg.state2state_over_goal_tolerance),
                             start_over + std::max(0.0, services.cfg.state2state_over_goal_tolerance));
            const double max_path_over =
                    state2stateMaxGoalOvershoot(path,
                                                services.local_start_p,
                                                temp_start_point,
                                                goal);
            if (near_goal_xy < near_goal_radius &&
                max_path_over > max_allowed_over + 1.0e-6) {
                vec_Vec3f direct_path;
                RET_CODE direct_ret = FAILED;
                if (buildDirectLineCandidate(direct_path, direct_ret) &&
                    direct_ret == REACH_GOAL) {
                    selected_path = direct_path;
                    selected_ret = direct_ret;
                    path = assembleSelectedPath();
                    path = shortcutPathByLineOfSight(path);
                    services.ros_ptr->warn(" -- [GeneralPlanner] Near-goal frontend path overshoots goal by {:.2f}m; clamp to direct goal segment.",
                                   max_path_over);
                } else {
                    services.ros_ptr->warn(" -- [GeneralPlanner] Near-goal frontend path overshoots goal by {:.2f}m but direct goal segment is not usable; keep A* path.",
                                   max_path_over);
                }
            }
        }

        if (path.size() >= 2) {
            const double max_segment =
                    std::max(services.cfg.resolution,
                             0.8 * std::max(services.cfg.resolution, services.cfg.corridor_line_max_length));
            if (std::isfinite(max_segment) && max_segment > 1.0e-3) {
                vec_Vec3f dense_path;
                appendPathPointUnique(path.front(), dense_path);
                for (std::size_t i = 1; i < path.size(); ++i) {
                    const Vec3f a = dense_path.back();
                    const Vec3f b = path[i];
                    const double len = (b - a).norm();
                    const int pieces = std::max(1, static_cast<int>(std::ceil(len / max_segment)));
                    for (int k = 1; k <= pieces; ++k) {
                        const double alpha = static_cast<double>(k) / static_cast<double>(pieces);
                        appendPathPointUnique(a + alpha * (b - a), dense_path);
                    }
                }
                path = dense_path;
            }
        }
        services.map_manager->observePlannedTopologyPath(path);
        return true;
    }

} // namespace state2state_task

}
