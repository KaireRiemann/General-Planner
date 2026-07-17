#pragma once

#include <memory>
#include <string>
#include <vector>

#include "path_search/astar.h"
#include <map_manager/map_manager.hpp>
#include "traj_opt/tracking_perching_traj_opt.hpp"

namespace general_planner
{

class TrackingFrontend
{
public:
    using Ptr = std::shared_ptr<TrackingFrontend>;

    struct Config
    {
        double tracking_distance{3.0};
        double distance_tolerance{0.8};
        double distance_lower_tolerance{0.8};
        double distance_upper_tolerance{0.8};
        double height_offset{0.8};
        double height_tolerance{0.6};
        double safe_distance{0.45};
        double visibility_safe_distance{0.25};
        double visibility_cone_ratio{0.12};
        double visibility_angle_clearance{0.08726646259971647};
        double reacquire_distance{6.0};
        bool soft_recovery_enable{true};
        double soft_recovery_margin{0.2};
        double searching_horizon{8.0};
        double low_speed_velocity_threshold{0.25};
        double angular_hysteresis{0.35};
        double candidate_angle_step{0.3926990817};
        int candidate_radius_num{3};
        int visibility_samples{5};
        bool fallback_relax_enable{true};
        double fallback_distance_tolerance_scale{1.6};
        double fallback_height_tolerance_scale{1.5};
        int fallback_candidate_radius_extra{2};
        double fallback_candidate_angle_step_scale{0.5};
        double fallback_search_horizon_scale{1.3};
        bool elastic_guide_enable{true};
        double elastic_distance_tolerance_scale{2.0};
        double elastic_height_tolerance_scale{2.0};
        bool partial_guide_enable{true};
        double partial_guide_min_duration{0.45};
        int partial_guide_min_samples{2};
        bool fov_feasibility_enable{true};
        bool yaw_rate_feasibility_enable{true};
        double fov_horizontal_deg{90.0};
        double fov_vertical_deg{60.0};
        double fov_range{4.0};
        double fov_range_margin{0.05};
        double fov_front_margin{0.05};
        double max_yaw_rate{3.14};
        double yaw_rate_margin{0.10};
        bool obstacle_recovery_enable{true};
        int grid_neighbor_mode{26};
        bool over_wall_enable{true};
        double over_wall_max_climb{2.0};
        bool side_pass_enable{true};
        double side_pass_width{1.5};
        bool reacquire_relax_yaw_rate{true};
        bool unknown_as_occupied{true};
        bool use_astar{true};
        bool use_visible_region{true};
        bool print_log{false};
    };

    TrackingFrontend(const Config &cfg,
                     const MapManager::Ptr &map_manager,
                     const path_search::Astar::Ptr &astar);

    bool buildProblem(const general_utils::StatePVAJ &head_pvaj,
                      const traj_opt::DynamicTargetStates &target_prediction,
                      traj_opt::TrackingProblem &problem,
                      const general_utils::Vec3f *reference_viewpoint = nullptr,
                      const traj_opt::DynamicTargetState *reference_target = nullptr) const;

private:
    struct ViewpointCandidate
    {
        general_utils::Vec3f point{general_utils::Vec3f::Zero()};
        double score{0.0};
    };

    bool isViewpointVisible(const general_utils::Vec3f &viewpoint,
                            const general_utils::Vec3f &target) const;
    bool isViewpointSafe(const general_utils::Vec3f &viewpoint) const;
    bool isViewpointFovFeasible(const general_utils::Vec3f &viewpoint,
                                const general_utils::Vec3f &target,
                                std::string *reason = nullptr) const;
    bool isViewpointYawRateFeasible(
        const general_utils::Vec3f &reference_viewpoint,
        const traj_opt::DynamicTargetState &reference_target,
        const general_utils::Vec3f &candidate,
        const traj_opt::DynamicTargetState &target,
        std::string *reason = nullptr) const;
    bool isGuideStartUsable(const general_utils::Vec3f &point) const;
    bool repairViewpointEndpoint(const general_utils::Vec3f &raw_viewpoint,
                                 const general_utils::Vec3f &target,
                                 general_utils::Vec3f &repaired_viewpoint) const;
    bool chooseVisibleViewpoint(const general_utils::Vec3f &seed,
                                const traj_opt::DynamicTargetState &target,
                                general_utils::Vec3f &viewpoint,
                                const general_utils::Vec3f *reference_viewpoint = nullptr,
                                const traj_opt::DynamicTargetState *reference_target = nullptr,
                                bool allow_large_bearing_change = true) const;
    bool collectVisibleViewpointCandidates(const general_utils::Vec3f &seed,
                                           const traj_opt::DynamicTargetState &target,
                                           std::vector<ViewpointCandidate> &candidates,
                                           const general_utils::Vec3f *reference_viewpoint = nullptr,
                                           const traj_opt::DynamicTargetState *reference_target = nullptr,
                                           bool allow_large_bearing_change = true) const;
    bool chooseConnectedVisibleViewpoint(const general_utils::Vec3f &seed,
                                         const traj_opt::DynamicTargetState &target,
                                         general_utils::vec_E<general_utils::Vec3f> &path,
                                         general_utils::Vec3f &viewpoint,
                                         const general_utils::Vec3f *reference_viewpoint = nullptr,
                                         const traj_opt::DynamicTargetState *reference_target = nullptr,
                                         bool allow_large_bearing_change = true) const;
    bool computeVisibleRegion(const traj_opt::DynamicTargetState &target,
                              const general_utils::Vec3f &seed,
                              traj_opt::TrackingVisibleRegion &region) const;
    bool findOcclusionAwareSeed(const general_utils::Vec3f &last_viewpoint,
                                const general_utils::Vec3f &last_target,
                                const general_utils::Vec3f &target,
                                general_utils::Vec3f &seed) const;
    bool extendToTrackingViewpoint(const general_utils::Vec3f &seed,
                                   const general_utils::Vec3f &target,
                                   const general_utils::Vec3f &fallback,
                                   general_utils::Vec3f &viewpoint) const;
    bool searchVisibleViewpointOnGrid(const general_utils::Vec3f &start,
                                      const traj_opt::DynamicTargetState &target,
                                      general_utils::Vec3f &viewpoint,
                                      general_utils::vec_E<general_utils::Vec3f> &path_to_viewpoint,
                                      const general_utils::Vec3f *reference_viewpoint = nullptr,
                                      const traj_opt::DynamicTargetState *reference_target = nullptr,
                                      bool allow_large_bearing_change = true) const;
    bool choosePropagatedViewpoint(const general_utils::Vec3f &reference_viewpoint,
                                   const traj_opt::DynamicTargetState &reference_target,
                                   const general_utils::Vec3f &connect_start,
                                   const traj_opt::DynamicTargetState &target,
                                   bool reacquire_mode,
                                   general_utils::Vec3f &viewpoint,
                                   general_utils::vec_E<general_utils::Vec3f> &path_to_viewpoint) const;
    bool chooseRelaxedFallbackViewpoint(const general_utils::Vec3f &last_viewpoint,
                                        const traj_opt::DynamicTargetState &target,
                                        general_utils::Vec3f &viewpoint,
                                        general_utils::vec_E<general_utils::Vec3f> &path_to_viewpoint,
                                        const general_utils::Vec3f *reference_viewpoint = nullptr,
                                        const traj_opt::DynamicTargetState *reference_target = nullptr,
                                        bool allow_large_bearing_change = true) const;
    bool chooseObstacleRecoveryViewpoint(const general_utils::Vec3f &start,
                                         const traj_opt::DynamicTargetState &target,
                                         general_utils::Vec3f &viewpoint,
                                         general_utils::vec_E<general_utils::Vec3f> &path_to_viewpoint,
                                         const general_utils::Vec3f *reference_viewpoint = nullptr,
                                         const traj_opt::DynamicTargetState *reference_target = nullptr,
                                         bool allow_large_bearing_change = true) const;
    bool centerViewpointInVisibleRegion(const general_utils::Vec3f &start,
                                        const traj_opt::DynamicTargetState &target,
                                        general_utils::Vec3f &viewpoint,
                                        general_utils::vec_E<general_utils::Vec3f> &path_to_viewpoint,
                                        traj_opt::TrackingVisibleRegion &region,
                                        const general_utils::Vec3f *reference_viewpoint = nullptr,
                                        const traj_opt::DynamicTargetState *reference_target = nullptr,
                                        bool allow_large_bearing_change = true) const;
    // Fail closed: blocked tracking guide segments must not append unsafe goals.
    bool appendPathSegment(const general_utils::Vec3f &start,
                           const general_utils::Vec3f &goal,
                           general_utils::vec_E<general_utils::Vec3f> &path,
                           bool verbose = true) const;
    bool appendLineSegmentSamples(const general_utils::Vec3f &start,
                                  const general_utils::Vec3f &goal,
                                  general_utils::vec_E<general_utils::Vec3f> &path) const;

private:
    Config cfg_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;
};

class PerchingFrontend
{
public:
    using Ptr = std::shared_ptr<PerchingFrontend>;

    struct Config
    {
        double robot_l{0.25};
        double v_plus{1.0};
        double pre_contact_distance{0.45};
        double terminal_relax_time{0.35};
        double safe_distance{0.45};
        double platform_radius{0.35};
        double robot_radius{0.25};
        double platform_clearance{0.05};
        double thrust_nominal{9.81};
        double thrust_range{2.0};
        double weight_nu{1.0e-2};
        double weight_tau_f{1.0e-3};
        double min_duration{0.6};
        double max_duration{4.0};
        double reference_speed{2.0};
        double max_speed{5.0};
        double max_acc{4.0};
        double max_jerk{20.0};
        double max_omega{4.0};
        double relative_z_min{0.1};
        double relative_z_max{3.0};
        double weight_relative_height{1.0};
        double visual_min_distance{0.2};
        double visual_activation_distance{3.0};
        double visual_fx{1.0};
        double visual_fy{1.0};
        double gravity{9.81};
        double searching_horizon{8.0};
        int piece_num{0};
        double min_piece_duration{0.12};
        double min_total_duration{0.0};
        double max_total_duration{-1.0};
        double time_lower_bound_weight{0.0};
        double time_upper_bound_weight{0.0};
        double duration_seed_weight{0.0};
        double duration_margin{0.20};
        bool allow_long_standalone{false};
        double max_piece_duration{1.2};
        int min_piece_num{3};
        int max_piece_num{8};
        bool multi_point_guide_enable{true};
        int moving_guide_sample_num{4};
        double tau_f_seed_limit{1.30};
        bool reset_surface_time{true};
        bool use_astar{true};
        bool use_dynamics_terminal_accel{true};
        bool rotate_surface_with_yaw_rate{true};
    };

    PerchingFrontend(const Config &cfg,
                     const MapManager::Ptr &map_manager,
                     const path_search::Astar::Ptr &astar);

    bool buildProblem(const general_utils::StatePVAJ &head_pvaj,
                      const traj_opt::PerchingSurfaceState &surface,
                      traj_opt::PerchingProblem &problem) const;

private:
    bool appendPathSegment(const general_utils::Vec3f &start,
                           const general_utils::Vec3f &goal,
                           general_utils::vec_E<general_utils::Vec3f> &path) const;

private:
    Config cfg_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;
};

} // namespace general_planner
