#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <general_utils/type_utils.hpp>

namespace general_planner::nhbp {

enum class SparseCellState {
    UNKNOWN,
    FREE_BOUNDARY,
    OCCUPIED_BOUNDARY,
    FRONTIER_BOUNDARY
};

struct SparseCellRecord {
    general_utils::Vec3f position{general_utils::Vec3f::Zero()};
    SparseCellState state{SparseCellState::UNKNOWN};
    double stamp{0.0};
    int observations{0};
};

class SparseGlobalMap {
public:
    struct Config {
        bool enable{false};
        double resolution{0.5};
        int max_records{200000};
        double stale_time{600.0};
    };

    SparseGlobalMap();
    explicit SparseGlobalMap(Config config);

    void configure(Config config);
    void reset();

    void observeBoundaryCell(const general_utils::Vec3f &position,
                             SparseCellState state,
                             double stamp);

    SparseCellState query(const general_utils::Vec3f &position,
                          double stamp) const;

    std::vector<SparseCellRecord> frontierRecords(const general_utils::Vec3f &center,
                                                  double radius,
                                                  double stamp,
                                                  int max_count) const;

    int recordCount() const;
    int frontierCount(double stamp) const;

private:
    std::string key(const general_utils::Vec3f &position) const;
    bool stale(const SparseCellRecord &record, double stamp) const;
    void prune(double stamp);

    Config config_;
    std::unordered_map<std::string, SparseCellRecord> records_;
};

const char *toString(SparseCellState state);

} // namespace general_planner::nhbp
