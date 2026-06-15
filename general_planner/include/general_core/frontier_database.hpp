#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <general_core/exploration_memory_grid.hpp>

namespace general_planner {

class FrontierDatabase {
public:
    using Ptr = std::shared_ptr<FrontierDatabase>;

    struct Config {
        double cluster_radius{0.8};
        int min_cluster_size{5};
        double merge_center_distance{1.0};
        double stale_timeout{5.0};
        int max_fail_count{3};
        double covered_gain_threshold{1.0};
        double dormant_timeout{10.0};
    };

    FrontierDatabase();
    explicit FrontierDatabase(const Config &cfg);

    void reset();

    void update(const super_utils::vec_E<CompleteFrontierCell> &frontier_cells,
                const ExplorationMemoryGrid &memory,
                double stamp);

    std::vector<int> activeFrontierIds() const;
    std::vector<int> reachableFrontierIds() const;

    std::vector<CompleteFrontierCluster> getActiveFrontiers() const;
    std::vector<CompleteFrontierCluster> getAllFrontiers() const;

    bool getFrontier(int id, CompleteFrontierCluster &out) const;

    void markSelected(int id, double stamp);
    void markFailed(int id, const std::string &reason, double stamp);
    void markCovered(int id, double stamp);
    void markDormant(int id, double stamp);
    void markUnreachable(int id, double stamp);
    void markBlacklisted(int id, double stamp);
    void reviveUnreachableNear(const super_utils::Vec3f &pos, double radius, double stamp);

    int activeCount() const;
    int reachableCount() const;
    int coveredCount() const;
    int unreachableCount() const;
    int dormantCount() const;
    int blacklistedCount() const;

private:
    struct ClusterBucketKey {
        int x{0};
        int y{0};
        int z{0};

        bool operator==(const ClusterBucketKey &other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct ClusterBucketHasher {
        std::size_t operator()(const ClusterBucketKey &key) const;
    };

    ClusterBucketKey makeBucketKey(const super_utils::Vec3f &pos) const;
    void clusterObservedFrontiers(const super_utils::vec_E<CompleteFrontierCell> &cells,
                                  super_utils::vec_E<CompleteFrontierCluster> &clusters) const;
    void finalizeCluster(CompleteFrontierCluster &cluster,
                         const ExplorationMemoryGrid &memory,
                         double stamp) const;
    int matchExistingCluster(const CompleteFrontierCluster &cluster,
                             const std::unordered_map<int, bool> &used_ids) const;
    bool clusterStillFrontier(const CompleteFrontierCluster &cluster,
                              const ExplorationMemoryGrid &memory) const;
    void setStatus(int id, FrontierStatus status, double stamp);
    int countStatus(FrontierStatus status) const;

    Config cfg_;
    std::unordered_map<int, CompleteFrontierCluster> frontiers_;
    int next_id_{1};
};

}  // namespace general_planner
