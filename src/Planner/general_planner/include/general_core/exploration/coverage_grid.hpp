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
};

class CoverageGrid {
public:
    struct Config {
        bool enable{false};
        double resolution{1.0};
        double revisit_radius{0.8};
        double revisit_time_window{5.0};
        int max_cells{4096};
    };

    CoverageGrid();
    explicit CoverageGrid(Config config);

    void reset();
    void observePose(const general_utils::Vec3f &position, double stamp);
    bool recentlyVisited(const general_utils::Vec3f &position, double stamp) const;
    double revisitPenalty(const general_utils::Vec3f &position, double stamp) const;

    int visitedCellCount() const;
    int totalVisitCount() const;

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
    void prune();

    Config config_;
    std::unordered_map<Key, CoverageCellRecord, KeyHasher> cells_;
    int total_visits_{0};
};

} // namespace general_planner
