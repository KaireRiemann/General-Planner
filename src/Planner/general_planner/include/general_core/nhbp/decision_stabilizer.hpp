#pragma once

#include <string>

#include <general_core/nhbp/navigation_memory.hpp>

namespace general_planner::nhbp {

struct DecisionCandidate {
    bool valid{false};
    int candidate_id{-1};
    int frontier_id{-1};
    NavIdentity identity;
    std::string key;
    general_utils::Vec3f position{general_utils::Vec3f::Zero()};
    double score{0.0};
};

struct DecisionContext {
    bool new_task{false};
    bool current_reusable{false};
    double committed_remaining{0.0};
    double stamp{0.0};
};

enum class StabilizerAction {
    ACCEPT_CANDIDATE,
    KEEP_CURRENT,
    REJECT_CANDIDATE
};

struct StabilizerDecision {
    StabilizerAction action{StabilizerAction::ACCEPT_CANDIDATE};
    std::string reason{"accept_candidate"};
    NdoDiagnosis ndo;
};

class DecisionStabilizer {
public:
    struct Config {
        bool enable{false};
        double min_commit_time{1.0};
        double switch_margin{0.25};
        bool recovery_enable{true};
    };

    DecisionStabilizer();
    explicit DecisionStabilizer(Config config);

    StabilizerDecision stabilize(const DecisionCandidate &candidate,
                                 const DecisionCandidate &current,
                                 const DecisionContext &context,
                                 const NavigationMemory &memory) const;

private:
    bool sameCandidate(const DecisionCandidate &lhs,
                       const DecisionCandidate &rhs) const;

    bool implicatedByNdo(const DecisionCandidate &candidate,
                         const NdoDiagnosis &diagnosis) const;

    Config config_;
};

const char *toString(StabilizerAction action);

} // namespace general_planner::nhbp
