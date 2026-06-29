#pragma once

#include <vector>

#include <Eigen/Eigen>

#include "data_structure/base/trajectory.h"
#include "general_core/config.hpp"
#include "traj_opt/tracking_perching_traj_opt.hpp"

namespace general_planner {

struct TrackingToPerchingInitialGuess {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid{false};

    double handover_delay{0.0};
    double total_time{0.0};

    general_utils::StatePVAJ head_pvaj{general_utils::StatePVAJ::Zero()};
    Eigen::Matrix<double, 1, 2> head_yaw{Eigen::Matrix<double, 1, 2>::Zero()};

    general_utils::vec_E<general_utils::Vec3f> guide_path;
    std::vector<double> guide_t;

    Eigen::Vector2d nu_seed{Eigen::Vector2d::Zero()};
    double tau_f_seed{0.0};

    traj_opt::PerchingSurfaceState rebased_surface;
};

class TrackingToPerchingInitializer {
public:
    bool build(const geometry_utils::Trajectory &tracking_pos,
               const geometry_utils::Trajectory &tracking_yaw,
               double current_tracking_local_t,
               const traj_opt::PerchingSurfaceState &surface,
               const Config &cfg,
               TrackingToPerchingInitialGuess &guess) const;
};

} // namespace general_planner
