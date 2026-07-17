#pragma once

#include <memory>
#include <string>

#include <Eigen/Eigen>

#include "data_structure/base/trajectory.h"
#include "general_core/config.hpp"
#include <map_manager/map_manager.hpp>
#include "traj_opt/tracking_perching_traj_opt.hpp"

namespace general_planner {

class TrackingRuntimeManager {
public:
    struct MotionMetrics {
        double speed_xy{0.0};
        double speed_z{0.0};
        double speed_3d{0.0};
        double displacement_xy{0.0};
        double displacement_z{0.0};
        double displacement_3d{0.0};
        double progress_xy{0.0};
        double progress_3d{0.0};
        double target_speed_xy{0.0};
        double target_speed_z{0.0};
        double target_speed_3d{0.0};
        bool target_vertical_moving{false};
        bool target_moving{false};
    };

    struct Activity {
        bool valid{false};
        bool safe{false};
        bool active{false};
        bool target_moving{false};
        bool target_vertical_moving{false};

        double remaining{0.0};
        double speed0{0.0};
        double speed_xy{0.0};
        double speed_z{0.0};
        double speed_3d{0.0};
        double displacement{0.0};
        double displacement_xy{0.0};
        double displacement_z{0.0};
        double displacement_3d{0.0};
        double progress{0.0};
        double progress_xy{0.0};
        double progress_3d{0.0};
        double expected_progress{0.0};
        double target_speed_xy{0.0};
        double target_speed_z{0.0};
        double target_speed_3d{0.0};
        double tracking_error{0.0};
        double avg_tracking_error{0.0};

        std::string reason;
    };

    enum class Status {
        IDLE,
        ACQUIRE,
        ACTIVE_COMMITTED,
        CANDIDATE_ACCEPTED,
        CANDIDATE_REJECTED_ANTI_ROLLBACK,
        CANDIDATE_REJECTED_NO_MOTION,
        KEEP_OLD_ACTIVE,
        KEEP_OLD_INACTIVE,
        FORCE_COMMIT_SAFE_CANDIDATE,
        LOST
    };

    enum class DecisionType {
        COMMIT_CANDIDATE,
        KEEP_OLD,
        FORCE_COMMIT_CANDIDATE,
        REJECT_AND_FAIL
    };

    struct Decision {
        DecisionType type{DecisionType::REJECT_AND_FAIL};
        Status status{Status::LOST};
        Activity old_activity;
        bool candidate_commandable{false};
        bool candidate_safe{false};
        bool bypass_anti_rollback{false};
        std::string reason;
    };

    TrackingRuntimeManager(const Config &cfg,
                           const MapManager::Ptr &map_manager);

    void reset();

    Activity evaluateActivity(const geometry_utils::Trajectory &traj,
                              double local_start_t,
                              const traj_opt::DynamicTargetStates &target_prediction,
                              double horizon,
                              double dt) const;

    bool candidateCommandable(const geometry_utils::Trajectory &candidate,
                              const traj_opt::DynamicTargetStates &target_prediction,
                              double candidate_eval_start_t = 0.0,
                              double target_eval_start_t = 0.0,
                              std::string *reason = nullptr) const;

    bool trajectorySafe(const geometry_utils::Trajectory &traj,
                        double start_t,
                        double horizon,
                        double dt,
                        std::string *reason = nullptr) const;

    Decision decide(const geometry_utils::Trajectory *old_committed,
                    double old_local_t,
                    const geometry_utils::Trajectory &candidate,
                    const traj_opt::DynamicTargetStates &target_prediction,
                    bool candidate_safe,
                    bool anti_rollback_pass,
                    double candidate_eval_start_t = 0.0,
                    double target_eval_start_t = 0.0);

    MotionMetrics computeMotionMetrics(const geometry_utils::Trajectory &candidate,
                                       const traj_opt::DynamicTargetStates &target_prediction,
                                       double candidate_eval_start_t,
                                       double target_eval_start_t,
                                       double horizon) const;

    void onCommitted();
    void onKeepOld();
    void onRejected();

    int consecutiveKeepOld() const;
    int consecutiveReject() const;
    Status status() const;
    bool hasCommittedTracking() const;

private:
    const Config &cfg_;
    MapManager::Ptr map_manager_;

    int consecutive_keep_old_{0};
    int consecutive_reject_{0};
    Status status_{Status::IDLE};
    bool has_committed_tracking_{false};

    general_utils::Vec3f targetDirection(const traj_opt::DynamicTargetStates &prediction) const;
    double trackingDistanceError(const general_utils::Vec3f &tracker,
                                 const general_utils::Vec3f &target) const;
};

} // namespace general_planner
