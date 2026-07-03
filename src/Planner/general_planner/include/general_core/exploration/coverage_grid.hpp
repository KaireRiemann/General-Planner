#pragma once

#include <cstddef>
#include <iostream>
#include <unordered_map>

#include <general_utils/type_utils.hpp>

namespace general_planner {

struct CoverageCellRecord {
    int visits{0};
    double first_visit_stamp{0.0};
    double last_visit_stamp{0.0};
    int observed_free_count{0};
    int observed_unknown_count{0};
    int observed_frontier_count{0};
    double information_gain_ema{0.0};
    double last_information_gain_stamp{0.0};
    bool covered{false};
    bool stale{false};
    bool no_progress_basin{false};
};

class CoverageGrid {
public:
    struct Config {
        bool enable{false};
        double resolution{1.0};
        double revisit_radius{0.8};
        double revisit_time_window{5.0};
        double intent_radius{4.0};
        int max_cells{4096};
        double information_gain_alpha{0.35};
        int covered_visit_threshold{2};
        double stale_time{30.0};
    };

    CoverageGrid();
    explicit CoverageGrid(Config config);

    void reset();
    void observePose(const general_utils::Vec3f &position, double stamp);
    void observeFrontierEvidence(const general_utils::Vec3f &position,
                                 double stamp,
                                 int unknown_count,
                                 int frontier_count,
                                 double information_gain);
    void markCoveredNear(const general_utils::Vec3f &position,
                         double stamp,
                         double radius);
    void markNoProgressBasin(const general_utils::Vec3f &position,
                             double stamp,
                             double radius);
    bool recentlyVisited(const general_utils::Vec3f &position, double stamp) const;
    double revisitPenalty(const general_utils::Vec3f &position, double stamp) const;
    double intentReward(const general_utils::Vec3f &position, double stamp) const;

    int visitedCellCount() const;
    int totalVisitCount() const;
    int coveredCellCount() const;
    int noProgressBasinCount(double stamp) const;
    double recentInformationGain(double stamp) const;

private:
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

    Key keyForPosition(const general_utils::Vec3f &position) const;
    bool recordStale(const CoverageCellRecord &record, double stamp) const;
    template <typename Fn>
    void forEachCellNear(const general_utils::Vec3f &position,
                         double radius,
                         Fn &&fn);
    void prune();

    Config config_;
    std::unordered_map<Key, CoverageCellRecord, KeyHasher> cells_;
    int total_visits_{0};
};

} // namespace general_planner
