#pragma once

#include <memory>

#include <map_manager/map_manager.hpp>
#include <path_search/astar.h>

#include "exploration/exploration_types.hpp"

namespace general_planner {
namespace exploration {

class LocalGuideBuilder {
public:
    struct Config {
        double local_goal_lookahead{4.0};
        double local_goal_min_distance{1.0};
        double final_goal_radius{0.7};
        double planning_horizon{8.0};
        double max_segment_length{1.0};
        double safe_distance{0.45};
        double start_safe_distance{0.20};
        double line_step{0.20};
        bool shortcut_enable{true};
        bool astar_repair_enable{true};
        bool unknown_as_occupied{false};
        MapBackend backend{MapBackend::HYBRID};
    };

    struct Request {
        GlobalRoute route;
        super_utils::Vec3f robot_pos{super_utils::Vec3f::Zero()};
        double current_yaw{0.0};
        super_utils::Vec3f final_goal{super_utils::Vec3f::Zero()};
        double final_yaw{0.0};
        int target_frontier_id{-1};
        int target_viewpoint_id{-1};
    };

    struct Result {
        bool valid{false};
        super_utils::vec_E<super_utils::Vec3f> guide_path;
        super_utils::Vec3f next_goal{super_utils::Vec3f::Zero()};
        double next_yaw{0.0};
        bool local_goal_is_final{false};
        double route_progress_length{0.0};
        std::string reason;
    };

    LocalGuideBuilder(Config cfg,
                      MapManager::Ptr map_manager,
                      path_search::Astar::Ptr astar);

    bool build(const Request &request, Result &result) const;

private:
    bool segmentSafe(const super_utils::Vec3f &a,
                     const super_utils::Vec3f &b) const;
    bool segmentSafe(const super_utils::Vec3f &a,
                     const super_utils::Vec3f &b,
                     double safe_distance) const;
    bool repairSegment(const super_utils::Vec3f &a,
                       const super_utils::Vec3f &b,
                       super_utils::vec_E<super_utils::Vec3f> &path) const;
    bool appendSafeOrRepairedSegment(const super_utils::Vec3f &a,
                                     const super_utils::Vec3f &b,
                                     super_utils::vec_E<super_utils::Vec3f> &path,
                                     bool &trimmed,
                                     std::string &reason) const;
    bool appendSafePrefixOfSegment(const super_utils::Vec3f &a,
                                   const super_utils::Vec3f &b,
                                   super_utils::vec_E<super_utils::Vec3f> &path,
                                   bool &trimmed,
                                   std::string &reason) const;
    bool stateSafe(const super_utils::Vec3f &p) const;
    bool stateSafe(const super_utils::Vec3f &p,
                   double safe_distance) const;
    super_utils::vec_E<super_utils::Vec3f> shortcutPath(
            const super_utils::vec_E<super_utils::Vec3f> &path) const;
    void densifyPath(super_utils::vec_E<super_utils::Vec3f> &path) const;
    static double pathLength(const super_utils::vec_E<super_utils::Vec3f> &path);
    static double headingFromPath(const super_utils::vec_E<super_utils::Vec3f> &path,
                                  double fallback_yaw);

private:
    Config cfg_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;
};

}  // namespace exploration
}  // namespace general_planner
