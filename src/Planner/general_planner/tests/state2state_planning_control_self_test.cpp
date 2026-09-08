#include <general_core/state2state/state2state_planning_control.hpp>
#include <utils/optimization/cancellation.hpp>
#include <utils/optimization/lbfgs.h>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
void expect(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
}
struct Cost {
    int evaluations{0};
    general_planner::state2state_task::State2StatePlanningControl *control;
};
double evaluate(void *instance, const Eigen::VectorXd &x, Eigen::VectorXd &gradient) {
    auto &cost = *static_cast<Cost *>(instance);
    if (++cost.evaluations == 2) cost.control->requestCancel();
    gradient = 2.0 * x;
    return x.squaredNorm();
}
}
int main() {
    using general_planner::state2state_task::State2StatePlanningControl;
    using math_utils::lbfgs;
    State2StatePlanningControl control;
    control.begin(1.0);
    expect(!control.cancelRequested(), "new operation should be active");
    {
        math_utils::ScopedCancellation scope([&] { return control.cancelRequested(); });
        bool other_thread_cancelled = true;
        std::thread worker([&] { other_thread_cancelled = math_utils::optimizationCancelled(); });
        worker.join();
        expect(!other_thread_cancelled, "cancellation must be worker-local");
        Cost cost{0, &control};
        Eigen::VectorXd x = Eigen::VectorXd::Constant(3, 10.0);
        double f = 0.0;
        expect(lbfgs::lbfgs_optimize(x, f, evaluate, nullptr, nullptr, &cost,
                                   lbfgs::lbfgs_parameter_t{}) == lbfgs::LBFGS_CANCELED,
               "line search must observe cancellation without a progress callback");
        expect(cost.evaluations == 2, "cancelled optimizer performed further evaluations");
    }
    expect(!math_utils::optimizationCancelled(), "scope must remove the worker hook");
    control.finish();
    control.begin(0.001);
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    expect(control.cancelRequested(), "steady deadline must cancel without ROS/watchdog callbacks");
    control.finish();
    control.begin(1.0);
    expect(!control.cancelRequested(), "worker should recover for next operation");
    control.finish();
    std::cout << "state2state_planning_control_self_test: PASS\n";
}
