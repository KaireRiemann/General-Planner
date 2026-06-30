#pragma once

#include <string>
#include <vector>

#include <data_structure/base/trajectory.h>
#include <general_core/nhbp/navigation_memory.hpp>
#include <general_core/runtime_trajectory_safety.hpp>
#include <utils/header/type_utils.hpp>

namespace general_planner {
class Config;
}

namespace general_planner::nhbp {

enum class State2StateGateAction {
    ACCEPT_CANDIDATE,
    KEEP_CURRENT
};

struct TrajectoryBranchSignature {
    bool valid{false};
    int branch_id{1};
    int lateral_sign{0};
    double lateral_peak{0.0};
    double length{0.0};
    double duration{0.0};
    double goal_distance{0.0};
    general_utils::Vec3f start{general_utils::Vec3f::Zero()};
    general_utils::Vec3f end{general_utils::Vec3f::Zero()};
    std::string goal_key;
    std::string branch_key;
};

struct State2StateNHBPDecision {
    State2StateGateAction action{State2StateGateAction::ACCEPT_CANDIDATE};
    std::string reason{"accept_candidate"};
    NdoDiagnosis ndo;
    TrajectoryBranchSignature candidate;
    TrajectoryBranchSignature current;
    bool current_safe{false};
    bool same_branch{false};
    bool commit_churn{false};
    std::string current_safety_reason{"not_checked"};
    double current_remaining{0.0};
    double current_safety_ttc{0.0};
    double current_safety_collision_t{0.0};
    double current_safety_check_horizon{0.0};
    general_utils::Vec3f current_safety_collision_pos{general_utils::Vec3f::Zero()};
    int current_safety_grid_type{0};
    int current_safety_hit_count{0};
    double candidate_score{0.0};
    double current_score{0.0};
    double score_improvement{0.0};
    double time_since_last_commit{0.0};
    double endpoint_delta{0.0};
    double lateral_delta{0.0};
    int commits_in_window{0};
};

class State2StateNHBPAdapter {
public:
    struct Config {
        bool enable{true};
        int decision_history{16};
        double blacklist_ttl{12.0};
        double min_commit_time{0.6};
        double min_commit_interval{0.5};
        double min_progress_distance{0.25};
        double switch_margin{0.35};
        double same_branch_margin{0.2};
        double endpoint_change_threshold{0.8};
        double lateral_oscillation_threshold{0.35};
        double commit_churn_window{2.0};
        int max_commits_in_window{5};
        int max_switches{4};
        double no_progress_time{2.5};
        double branch_lateral_threshold{0.35};
        double signature_horizon{3.0};
        double safety_check_horizon{1.5};
        double reuse_safety_check_horizon{0.6};
        int reuse_safety_consecutive_hits{2};
        double goal_key_resolution{0.5};
        double goal_progress_weight{2.0};
    };

    State2StateNHBPAdapter();
    explicit State2StateNHBPAdapter(Config config);

    void configure(Config config);
    void reset();

    State2StateNHBPDecision evaluateReplan(
            const geometry_utils::Trajectory &candidate_traj,
            const geometry_utils::Trajectory &current_traj,
            double current_backup_start_t,
            const general_utils::RobotState &robot_state,
            const general_utils::Vec3f &goal,
            double stamp,
            bool new_goal,
            const RuntimeTrajectorySafetyServices &safety_services,
            const ::general_planner::Config &planner_cfg);

    void recordCommitted(const geometry_utils::Trajectory &traj,
                         const general_utils::RobotState &robot_state,
                         const general_utils::Vec3f &goal,
                         double stamp,
                         const std::string &source);

    void recordFailure(const geometry_utils::Trajectory *candidate_traj,
                       const general_utils::RobotState &robot_state,
                       const general_utils::Vec3f &goal,
                       double stamp,
	                       FailureReason reason);

    std::string diagnosticSummary(double stamp) const;
    std::string formatDecisionDiagnostic(const State2StateNHBPDecision &decision) const;

private:
    TrajectoryBranchSignature makeSignature(const geometry_utils::Trajectory &traj,
                                            const general_utils::Vec3f &goal,
                                            double stamp) const;
    double scoreSignature(const TrajectoryBranchSignature &signature) const;
    int candidateId(const TrajectoryBranchSignature &signature) const;
    std::string makeGoalKey(const general_utils::Vec3f &goal) const;
    std::string makeBranchKey(const std::string &goal_key, int branch_id) const;
    bool sameLongRangeIntent(const std::string &goal_key) const;
    bool currentTrajectoryReusable(const geometry_utils::Trajectory &current_traj,
                                   double current_backup_start_t,
                                   double stamp,
                                   const RuntimeTrajectorySafetyServices &safety_services,
	                                   const ::general_planner::Config &planner_cfg,
	                                   State2StateNHBPDecision &decision) const;
    int commitsInWindow(double stamp) const;
    void pruneRecentCommitHistory(double stamp);
    void clearCommitHistory();

    Config config_;
    NavigationMemory memory_;
    std::string active_goal_key_;
    bool has_last_new_commit_{false};
    double last_new_commit_stamp_{0.0};
    TrajectoryBranchSignature last_new_commit_signature_;
    general_utils::Vec3f last_new_commit_robot_position_{general_utils::Vec3f::Zero()};
    std::vector<double> recent_new_commit_stamps_;
};

const char *toString(State2StateGateAction action);

} // namespace general_planner::nhbp
