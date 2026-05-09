#pragma once

#include <memory>
#include <vector>

#include "path_search/astar.h"
#include "general_core/map_manager.hpp"
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
        double height_offset{0.8};
        double height_tolerance{0.6};
        double safe_distance{0.45};
        double visibility_safe_distance{0.25};
        double visibility_cone_ratio{0.12};
        double visibility_angle_clearance{0.08726646259971647};
        double reacquire_distance{6.0};
        double searching_horizon{8.0};
        double candidate_angle_step{0.3926990817};
        int candidate_radius_num{3};
        int visibility_samples{5};
        double score_path_weight{1.0};
        double score_preferred_weight{0.25};
        double score_od_weight{6.0};
        double score_oe_weight{5.0};
        double score_clearance_reward{1.5};
        double score_source_bias_weight{1.0};
        bool unknown_as_occupied{true};
        bool use_astar{true};
        bool use_visible_region{true};
        bool save_log{false};
        bool print_log{false};
    };

    TrackingFrontend(const Config &cfg,
                     const MapManager::Ptr &map_manager,
                     const path_search::Astar::Ptr &astar);

    bool buildProblem(const super_utils::StatePVAJ &head_pvaj,
                      const traj_opt::DynamicTargetStates &target_prediction,
                      traj_opt::TrackingProblem &problem) const;

private:
    struct ViewpointCandidate
    {
        super_utils::Vec3f point{super_utils::Vec3f::Zero()};
        double score{0.0};
        double od_cost{0.0};
        double oe_margin{0.0};
        double path_cost{0.0};
        double preferred_cost{0.0};
        double source_bias{0.0};
    };

    super_utils::Vec3f preferredViewpoint(const super_utils::Vec3f &seed,
                                          const traj_opt::DynamicTargetState &target) const;
    double estimateVisibilityMargin(const super_utils::Vec3f &viewpoint,
                                    const super_utils::Vec3f &target) const;
    bool scoreViewpointCandidate(const super_utils::Vec3f &candidate,
                                 const super_utils::Vec3f &seed,
                                 const super_utils::Vec3f &preferred,
                                 const traj_opt::DynamicTargetState &target,
                                 double source_bias,
                                 ViewpointCandidate &scored) const;
    bool isViewpointVisible(const super_utils::Vec3f &viewpoint,
                            const super_utils::Vec3f &target) const;
    bool isViewpointSafe(const super_utils::Vec3f &viewpoint) const;
    bool isGuideStartUsable(const super_utils::Vec3f &point) const;
    bool repairViewpointEndpoint(const super_utils::Vec3f &raw_viewpoint,
                                 const super_utils::Vec3f &target,
                                 super_utils::Vec3f &repaired_viewpoint) const;
    bool chooseVisibleViewpoint(const super_utils::Vec3f &seed,
                                const traj_opt::DynamicTargetState &target,
                                super_utils::Vec3f &viewpoint) const;
    bool collectVisibleViewpointCandidates(const super_utils::Vec3f &seed,
                                           const traj_opt::DynamicTargetState &target,
                                           std::vector<ViewpointCandidate> &candidates) const;
    bool chooseConnectedVisibleViewpoint(const super_utils::Vec3f &seed,
                                         const traj_opt::DynamicTargetState &target,
                                         super_utils::vec_E<super_utils::Vec3f> &path,
                                         super_utils::Vec3f &viewpoint) const;
    bool computeVisibleRegion(const traj_opt::DynamicTargetState &target,
                              const super_utils::Vec3f &seed,
                              traj_opt::TrackingVisibleRegion &region) const;
    bool findOcclusionAwareSeed(const super_utils::Vec3f &last_viewpoint,
                                const super_utils::Vec3f &last_target,
                                const super_utils::Vec3f &target,
                                super_utils::Vec3f &seed) const;
    bool extendToTrackingViewpoint(const super_utils::Vec3f &seed,
                                   const super_utils::Vec3f &target,
                                   const super_utils::Vec3f &fallback,
                                   super_utils::Vec3f &viewpoint) const;
    bool searchVisibleViewpointOnGrid(const super_utils::Vec3f &start,
                                      const traj_opt::DynamicTargetState &target,
                                      super_utils::Vec3f &viewpoint,
                                      super_utils::vec_E<super_utils::Vec3f> &path_to_viewpoint) const;
    bool chooseApproachViewpoint(const super_utils::Vec3f &start,
                                 const traj_opt::DynamicTargetState &target,
                                 super_utils::Vec3f &viewpoint,
                                 super_utils::vec_E<super_utils::Vec3f> &path_to_viewpoint) const;
    bool choosePropagatedViewpoint(const super_utils::Vec3f &last_viewpoint,
                                   const traj_opt::DynamicTargetState &last_target,
                                   const traj_opt::DynamicTargetState &target,
                                   super_utils::Vec3f &viewpoint,
                                   super_utils::vec_E<super_utils::Vec3f> &path_to_viewpoint) const;
    // Fail closed: blocked tracking guide segments must not append unsafe goals.
    bool appendPathSegment(const super_utils::Vec3f &start,
                           const super_utils::Vec3f &goal,
                           super_utils::vec_E<super_utils::Vec3f> &path,
                           bool verbose = true) const;
    bool appendLineSegmentSamples(const super_utils::Vec3f &start,
                                  const super_utils::Vec3f &goal,
                                  super_utils::vec_E<super_utils::Vec3f> &path) const;

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
        double searching_horizon{8.0};
        bool use_astar{true};
        bool use_dynamics_terminal_accel{true};
    };

    PerchingFrontend(const Config &cfg,
                     const MapManager::Ptr &map_manager,
                     const path_search::Astar::Ptr &astar);

    bool buildProblem(const super_utils::StatePVAJ &head_pvaj,
                      const traj_opt::PerchingSurfaceState &surface,
                      traj_opt::PerchingProblem &problem) const;

private:
    bool appendPathSegment(const super_utils::Vec3f &start,
                           const super_utils::Vec3f &goal,
                           super_utils::vec_E<super_utils::Vec3f> &path) const;

private:
    Config cfg_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;
};

} // namespace general_planner
