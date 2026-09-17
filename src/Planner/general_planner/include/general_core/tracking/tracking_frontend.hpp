#pragma once

#include <chrono>
#include <memory>
#include <map_manager/map_manager.hpp>
#include "traj_opt/tracking_problem.hpp"

namespace general_planner {

class TrackingFrontend
{
public:
    using Ptr = std::shared_ptr<TrackingFrontend>;

    struct Config
    {
        double nominal_horizon{3.0};
        double max_extrapolation{2.0};
        double search_budget_seconds{0.025};
        double sample_dt{0.25};
        double max_speed{5.0};
        double max_acc{3.0};
        double max_jerk{6.0};
        double closing_gain{0.6};
        double max_closing_speed{2.0};
        double tracking_distance{3.0};
        double distance_tolerance{0.8};
        double distance_lower_tolerance{0.8};
        double distance_upper_tolerance{0.8};
        double height_offset{0.8};
        double height_tolerance{0.6};
        double target_half_width{0.5};
        double target_half_height{0.7};
        double safe_distance{0.45};
        double visibility_angle_clearance{0.08726646259971647};
        double searching_horizon{8.0};
        bool unknown_as_occupied{true};
        bool use_astar{true};
        bool use_visible_region{true};
    };

    TrackingFrontend(const Config &cfg,
                     const MapManager::Ptr &map_manager);

    bool buildProblem(const general_utils::StatePVAJ &head_pvaj,
                      const traj_opt::DynamicTargetStates &target_prediction,
                      traj_opt::TrackingProblem &problem,
                      const general_utils::Vec3f *reference_viewpoint = nullptr,
                      const traj_opt::DynamicTargetState *reference_target = nullptr) const;

private:
    bool safe(const general_utils::Vec3f &point) const;
    bool visible(const general_utils::Vec3f &point, const general_utils::Vec3f &target) const;
    bool connectRegion(const general_utils::Vec3f &start,
                       const traj_opt::DynamicTargetState &target,
                       const general_utils::Vec3f &preferred,
                       general_utils::vec_E<general_utils::Vec3f> &path,
                       const std::chrono::steady_clock::time_point &deadline) const;
    traj_opt::TrackingVisibleRegion visibleRegion(const traj_opt::DynamicTargetState &target,
                                                  const general_utils::Vec3f &seed) const;

private:
    Config cfg_;
    MapManager::Ptr map_manager_;
};

} // namespace general_planner
