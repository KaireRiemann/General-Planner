#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <general_core/config.hpp>
#include <general_core/exploration/exploration_frontend.hpp>
#include <general_core/map_manager.hpp>

namespace general_planner {

class ExplorationManager {
public:
    struct Config {
        bool enable{true};
        bool print_log{false};

        bool global_box_enable{false};
        general_utils::Vec3f global_box_min{-50.0, -50.0, 0.0};
        general_utils::Vec3f global_box_max{50.0, 50.0, 3.0};

        double frontier_merge_radius{2.0};
        double frontier_record_ttl{120.0};
        double frontier_failure_ttl{15.0};
        double frontier_covered_radius{2.0};
        int max_frontier_records{1024};

        double coverage_resolution{1.0};
        double coverage_radius{10.0};
        double coverage_z_half{1.5};
        int max_coverage_cells{200000};

        double unknown_gain_radius{4.0};
        double unknown_gain_resolution{0.8};
        double unknown_gain_score_weight{-0.04};
        double revisit_score_weight{2.0};
        double covered_score_weight{20.0};
        double stale_frontier_score_weight{0.25};
        double committed_frontier_score_weight{8.0};
        double recent_commit_score_weight{14.0};
        double recent_commit_time_window{10.0};
        int max_commits_before_cooldown{3};
        double overcommit_cooldown{8.0};

        double completion_scan_resolution{1.0};
        int completion_max_unknown_voxels{25};
        double completion_min_coverage_ratio{0.85};
        int completion_stable_cycles{3};
        int completion_max_active_frontiers{0};
    };

    struct Diagnostics {
        int active_frontiers{0};
        int covered_frontiers{0};
        int failed_frontiers{0};
        int coverage_cells{0};
        int unknown_voxels{0};
        double coverage_ratio{0.0};
        int stable_finish_cycles{0};
    };

    static Config makeConfig(const general_planner::Config &planner_cfg);

    ExplorationManager(const Config &cfg, MapManager::Ptr map_manager);

    void reset();

    bool enabled() const;

    bool globalBoxEnabled() const;

    bool insideTaskBox(const general_utils::Vec3f &position) const;

    bool clipSearchBox(general_utils::Vec3f &box_min,
                       general_utils::Vec3f &box_max) const;

    void beginCycle(const general_utils::Vec3f &robot_pos, double stamp);

    void filterAndScoreCandidates(const general_utils::Vec3f &robot_pos,
                                  double stamp,
                                  general_utils::vec_E<ExplorationGoal> &candidates);

    void updateNoFrontier(const general_utils::Vec3f &robot_pos,
                          double stamp,
                          bool map_observation_ready);

    bool recoverGoal(const general_utils::Vec3f &robot_pos,
                     double current_yaw,
                     double stamp,
                     ExplorationGoal &goal);

    void recordCommitted(const ExplorationGoal &goal,
                         const general_utils::Vec3f &robot_pos,
                         double stamp);

    void recordFailure(const ExplorationGoal &goal, double stamp);

    bool isFinished() const;

    std::string finishReason() const;

    Diagnostics diagnostics(double stamp) const;

    std::string diagnosticSummary(double stamp) const;

private:
    enum class FrontierState {
        ACTIVE,
        COMMITTED,
        COVERED,
        FAILED
    };

    struct Key {
        int x{0};
        int y{0};
        int z{0};

        bool operator==(const Key &other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct KeyHasher {
        std::size_t operator()(const Key &key) const;
    };

    struct FrontierRecord {
        ExplorationGoal goal;
        general_utils::Vec3f reference{general_utils::Vec3f::Zero()};
        FrontierState state{FrontierState::ACTIVE};
        double first_seen_stamp{0.0};
        double last_seen_stamp{0.0};
        double last_state_stamp{0.0};
        double blocked_until{0.0};
        int seen_count{0};
        int failure_count{0};
        int commit_count{0};
        double last_commit_stamp{0.0};
    };

    struct CoverageRecord {
        double first_seen_stamp{0.0};
        double last_seen_stamp{0.0};
        int visits{0};
    };

    Key keyForPosition(const general_utils::Vec3f &position, double resolution) const;

    std::string keyForFrontier(const ExplorationGoal &goal) const;

    general_utils::Vec3f frontierReference(const ExplorationGoal &goal) const;

    FrontierRecord &upsertFrontier(const ExplorationGoal &goal,
                                   double stamp,
                                   const general_utils::Vec3f &reference);

    const FrontierRecord *findRecord(const ExplorationGoal &goal) const;

    FrontierRecord *findRecordMutable(const ExplorationGoal &goal);

    void prune(double stamp);

    void observeCoverageAround(const general_utils::Vec3f &robot_pos,
                               double stamp);

    int countCoverageNear(const general_utils::Vec3f &position,
                          double radius) const;

    int countUnknownNear(const general_utils::Vec3f &position,
                         double radius,
                         double resolution) const;

    int countUnknownInTaskBox() const;

    double estimateCoverageRatio() const;

    bool frontierStillUseful(const general_utils::Vec3f &reference) const;

    bool isFrontierCovered(const general_utils::Vec3f &reference,
                           const general_utils::Vec3f &robot_pos) const;

    void updateCoveredRecords(const general_utils::Vec3f &robot_pos,
                              double stamp);

    void updateFinishState(const general_utils::Vec3f &robot_pos,
                           double stamp,
                           bool local_frontier_available,
                           bool map_observation_ready);

    int activeFrontierCount(double stamp) const;

    int stateCount(FrontierState state, double stamp) const;

    bool recordExpired(const FrontierRecord &record, double stamp) const;

    Config cfg_;
    MapManager::Ptr map_manager_;
    std::unordered_map<std::string, FrontierRecord> frontier_records_;
    std::unordered_map<Key, CoverageRecord, KeyHasher> coverage_cells_;
    bool finished_{false};
    std::string finish_reason_;
    int stable_finish_cycles_{0};
    mutable int last_unknown_voxels_{0};
    mutable double last_coverage_ratio_{0.0};
};

} // namespace general_planner
