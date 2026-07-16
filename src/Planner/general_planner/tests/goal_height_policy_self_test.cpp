#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include "fsm/goal_height_policy.hpp"

int main() {
    using fsm::GoalHeightMode;
    using fsm::resolveGoalHeight;

    int failures = 0;
    const auto expectNear = [&failures](const std::string &name,
                                        const double actual,
                                        const double expected) {
        if (!std::isfinite(actual) || std::abs(actual - expected) > 1.0e-12) {
            std::cerr << name << ": expected " << expected
                      << ", got " << actual << '\n';
            ++failures;
        }
    };

    expectNear("2D configured height",
               resolveGoalHeight(3.2, 1.5,
                                 GoalHeightMode::CONFIGURED_CLICK_HEIGHT),
               1.5);
    expectNear("3D positive message height",
               resolveGoalHeight(3.2, 1.5, GoalHeightMode::MESSAGE_HEIGHT),
               3.2);
    expectNear("3D zero message height",
               resolveGoalHeight(0.0, 1.5, GoalHeightMode::MESSAGE_HEIGHT),
               0.0);
    expectNear("3D negative message height",
               resolveGoalHeight(-0.6, 1.5, GoalHeightMode::MESSAGE_HEIGHT),
               -0.6);
    expectNear("legacy minus-five sentinel",
               resolveGoalHeight(2.4, -5.0,
                                 GoalHeightMode::CONFIGURED_CLICK_HEIGHT),
               2.4);
    expectNear("legacy NaN click height",
               resolveGoalHeight(2.4,
                                 std::numeric_limits<double>::quiet_NaN(),
                                 GoalHeightMode::CONFIGURED_CLICK_HEIGHT),
               2.4);

    if (failures != 0) {
        std::cerr << "goal_height_policy_self_test: FAIL (" << failures
                  << " cases)\n";
        return 1;
    }

    std::cout << "goal_height_policy_self_test: PASS\n";
    return 0;
}
