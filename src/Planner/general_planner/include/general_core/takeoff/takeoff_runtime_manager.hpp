#pragma once

#include <limits>
#include <memory>
#include <string>

#include <Eigen/Eigen>

#include "data_structure/base/trajectory.h"
#include "general_core/config.hpp"
#include <map_manager/map_manager.hpp>
#include "traj_opt/tracking_perching_traj_opt.hpp"

namespace general_planner
{

class TakeoffRuntimeManager
{
public:
    enum class Status
    {
        IDLE,
        ON_PLATFORM,
        PREPARE,
        EXECUTING,
        PLATFORM_CLEAR,
        REACQUIRE_TRACKING,
        ABORT
    };

    struct CheckResult
    {
        bool valid{false};
        bool safe{false};
        bool dynamics_feasible{false};
        bool platform_clear_after_release{false};
        bool terminal_escape_valid{false};
        double max_thrust{0.0};
        double max_omega{0.0};
        double min_esdf_clearance{std::numeric_limits<double>::infinity()};
        double min_platform_margin_after_release{std::numeric_limits<double>::infinity()};
        std::string reason;
    };

    TakeoffRuntimeManager(const Config &cfg,
                          MapManager::Ptr map_manager);

    void reset();

    CheckResult checkCandidate(const geometry_utils::Trajectory &pos_traj,
                               const traj_opt::DynamicTakeoffProblem &problem) const;

    bool decideCommit(const CheckResult &candidate_check);

    void updateStatusAfterCommit();
    void updateStatusByPosition(const Eigen::Vector3d &position,
                                const traj_opt::DynamicTakeoffProblem &problem);

    Status status() const;
    bool hasCommittedTakeoff() const;

private:
    struct SurfaceFrame
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
        Eigen::Vector3d x{Eigen::Vector3d::UnitX()};
        Eigen::Vector3d y{Eigen::Vector3d::UnitY()};
        Eigen::Vector3d z{Eigen::Vector3d::UnitZ()};
    };

    SurfaceFrame frameAt(const traj_opt::DynamicTakeoffProblem &problem,
                         double t) const;

    std::string gridTypeName(rog_map::GridType type) const;

    double platformMargin(const Eigen::Vector3d &position,
                          const Eigen::Vector3d &acceleration,
                          const SurfaceFrame &frame,
                          const traj_opt::DynamicTakeoffProblem &problem) const;

private:
    const Config &cfg_;
    MapManager::Ptr map_manager_;
    Status status_{Status::IDLE};
    bool has_committed_takeoff_{false};
};

} // namespace general_planner
