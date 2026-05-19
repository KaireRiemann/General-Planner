#pragma once

#include <unordered_map>
#include <unordered_set>

#include "exploration/observation_map.hpp"

namespace general_planner {
namespace exploration {

class FrontierDatabase {
public:
    using Ptr = std::shared_ptr<FrontierDatabase>;

    struct Config {
        double association_distance{1.2};
        double bbox_overlap_min_ratio{0.08};
        int max_failed_count{3};
        int max_selected_count_without_gain{4};
        double blacklist_time{8.0};
        double covered_gain_threshold{4.0};
        double missing_frontier_timeout{2.0};
        double dormant_time{4.0};
    };

    explicit FrontierDatabase(Config cfg);

    void update(const std::vector<SurfaceFrontierCluster> &clusters,
                double stamp);

    std::vector<FrontierRecord> getActiveFrontiers() const;
    std::vector<FrontierRecord> getReachableFrontiers() const;

    bool getFrontier(int stable_id, FrontierRecord &out) const;
    bool isFrontierActive(int stable_id) const;

    void setViewpoints(int stable_id,
                       const std::vector<ExplorationViewpoint> &viewpoints,
                       const ExplorationViewpoint &best_viewpoint,
                       double stamp);

    void markSelected(int stable_id, double stamp);
    void markCovered(int stable_id, double stamp);
    void markDormant(int stable_id, double stamp);
    void markUnreachable(int stable_id, double stamp);
    void markBlacklisted(int stable_id, double stamp);
    void markViewpointVisited(int frontier_id, int viewpoint_id, double stamp);
    void onLowGain(int stable_id, double actual_gain, double stamp);
    void onFailed(int stable_id, double stamp);

    void reset();

    int activeCount() const;

private:
    int associateOrCreateId(const SurfaceFrontierCluster &cluster, double stamp);
    static double bboxOverlapRatio(const FrontierRecord &record,
                                   const SurfaceFrontierCluster &cluster);
    static bool expiredBlacklist(const FrontierRecord &record,
                                 const Config &cfg,
                                 double stamp);

private:
    Config cfg_;
    int next_id_{0};
    std::unordered_map<int, FrontierRecord> records_;
};

}  // namespace exploration
}  // namespace general_planner

