#pragma once

#include <limits>
#include <memory>
#include <string>

#include <Eigen/Eigen>

#include "data_structure/base/trajectory.h"
#include "general_core/config.hpp"
#include <map_manager/map_manager.hpp>
#include "traj_opt/tracking_perching_traj_opt.hpp"

namespace general_planner {

class PerchingRuntimeManager {
public:
    enum class Status {
        IDLE,
        PREPARE,
        CANDIDATE_OPTIMIZED,
        EXECUTING,
        CONTACT_IMMINENT,
        CONTACT,
        ABORT,
        LOST
    };

    enum class DecisionType {
        COMMIT_CANDIDATE,
        KEEP_CURRENT_PERCHING,
        ABORT,
        REJECT
    };

    struct CheckResult {
        bool valid{false};
        bool safe{false};
        bool terminal_sync{false};
        bool dynamics_feasible{false};
        bool contact_imminent{false};
        double terminal_position_error{0.0};
        double terminal_velocity_error{0.0};
        double max_thrust{0.0};
        double max_omega{0.0};
        double min_esdf_clearance{std::numeric_limits<double>::infinity()};
        double min_platform_margin{std::numeric_limits<double>::infinity()};
        std::string reason;
    };

    PerchingRuntimeManager(const Config &cfg,
                           const MapManager::Ptr &map_manager);

    void reset();

    CheckResult checkCandidate(const geometry_utils::Trajectory &pos_traj,
                               const geometry_utils::Trajectory *yaw_traj,
                               const traj_opt::PerchingProblem &problem,
                               const traj_opt::PerchingSurfaceState &surface) const;

    DecisionType decideCommit(const CheckResult &candidate_check,
                              const CheckResult *current_perching_check);

    bool candidateAccepted(const CheckResult &candidate_check) const;
    void rememberRejectedCandidate(const traj_opt::PerchingProblem &problem,
                                   const std::string &reason,
                                   double stamp);
    bool shouldSkipRejectedCandidate(const traj_opt::PerchingProblem &problem,
                                     double stamp,
                                     std::string *reason = nullptr) const;

    void updateStatusAfterCommit();
    void updateStatusAfterContact();

    Status status() const;
    bool hasCommittedPerching() const;
    int consecutiveReject() const;

private:
    struct SurfaceFrame {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
        Eigen::Vector3d x{Eigen::Vector3d::UnitX()};
        Eigen::Vector3d y{Eigen::Vector3d::UnitY()};
        Eigen::Vector3d z{Eigen::Vector3d::UnitZ()};
    };

    SurfaceFrame frameAt(const traj_opt::PerchingSurfaceState &surface,
                         const traj_opt::PerchingProblem &problem,
                         double t) const;

    bool isFinalContactWindow(double eval_t, double duration) const;

    bool isExpectedContactSample(const Eigen::Vector3d &p,
                                 const SurfaceFrame &frame,
                                 const traj_opt::PerchingProblem &problem,
                                 double eval_t,
                                 double duration,
                                 double *normal_dist = nullptr,
                                 double *tangent_dist = nullptr) const;

    std::string gridTypeName(rog_map::GridType type) const;

    double platformMargin(const Eigen::Vector3d &position,
                          const Eigen::Vector3d &acceleration,
                          const SurfaceFrame &frame,
                          const traj_opt::PerchingProblem &problem) const;

    Eigen::Vector3d expectedTerminalPosition(const traj_opt::PerchingProblem &problem,
                                             const traj_opt::PerchingSurfaceState &surface,
                                             double T) const;

    Eigen::Vector3d expectedTerminalVelocityBase(const traj_opt::PerchingProblem &problem,
                                                 const traj_opt::PerchingSurfaceState &surface,
                                                 double T) const;

    struct RejectedCandidateSignature {
        bool valid{false};
        Eigen::Vector3d head_position{Eigen::Vector3d::Zero()};
        Eigen::Vector3d head_velocity{Eigen::Vector3d::Zero()};
        Eigen::Vector3d surface_position{Eigen::Vector3d::Zero()};
        Eigen::Vector3d surface_velocity{Eigen::Vector3d::Zero()};
        Eigen::Vector3d surface_normal{Eigen::Vector3d::UnitZ()};
        double duration_seed{0.0};
        int piece_num{0};
        double stamp{0.0};
        std::string reason;
    };

    const Config &cfg_;
    MapManager::Ptr map_manager_;
    Status status_{Status::IDLE};
    bool has_committed_perching_{false};
    int consecutive_reject_{0};
    RejectedCandidateSignature last_rejected_candidate_;
};

} // namespace general_planner
