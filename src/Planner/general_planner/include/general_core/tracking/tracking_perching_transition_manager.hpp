#pragma once

#include <string>

#include "data_structure/base/trajectory.h"
#include "general_core/config.hpp"
#include "traj_opt/tracking_perching_traj_opt.hpp"

namespace general_planner {

class TrackingPerchingTransitionManager {
public:
    enum class Status {
        TRACKING_ONLY,
        PERCHING_REQUESTED,
        PERCHING_READY,
        PERCHING_CANDIDATE_TESTING,
        PERCHING_COMMITTED,
        PERCHING_EXECUTING,
        CONTACT_IMMINENT,
        CONTACT,
        ABORT_TO_TRACKING
    };

    struct Readiness {
        bool ready{false};
        bool request{false};
        bool tracking_active{false};
        bool surface_valid{false};
        bool relative_state_good{false};
        bool duration_good{false};
        bool prediction_good{false};

        int ready_count{0};
        double distance{0.0};
        double relative_speed{0.0};
        double lateral_speed{0.0};
        double estimated_duration{0.0};
        std::string reason;
    };

    void reset();
    void setPerchingRequest(bool request);
    bool perchingRequested() const;

    Readiness evaluateReadiness(bool tracking_active,
                                const geometry_utils::Trajectory &tracking_pos,
                                const geometry_utils::Trajectory &tracking_yaw,
                                double tracking_local_t,
                                const traj_opt::PerchingSurfaceState &surface,
                                const Config &cfg);

    void onCandidateTesting();
    void onPerchingCommitted();
    void onContact();
    void onPerchingRejected();
    void onAbortToTracking();

    Status status() const;

private:
    Status status_{Status::TRACKING_ONLY};
    bool perching_request_{false};
    int ready_count_{0};
};

} // namespace general_planner
