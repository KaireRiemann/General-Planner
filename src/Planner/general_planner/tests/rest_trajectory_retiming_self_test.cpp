#include <general_core/state2state/rest_trajectory_retiming.hpp>
#include <cstdlib>
#include <iostream>

void require(bool ok, const char *reason) {
  if (!ok) { std::cerr << reason << '\n'; std::exit(1); }
}
int main() {
  using geometry_utils::Trajectory;
  for (double distance : {0.5, -0.5, 2.0}) {
    // Seventh-order rest-to-rest step: zero velocity, acceleration AND jerk
    // at both boundaries, intentionally too fast before repair.
    const double duration = .2;
    Eigen::MatrixXd c = Eigen::MatrixXd::Zero(3, 8);
    c(2, 0) = -20 * distance / std::pow(duration, 7);
    c(2, 1) = 70 * distance / std::pow(duration, 6);
    c(2, 2) = -84 * distance / std::pow(duration, 5);
    c(2, 3) = 35 * distance / std::pow(duration, 4);
    c(0, 7) = 21.5; c(2, 7) = 1.5;
    Trajectory original; original.emplace_back(duration, c);
    Trajectory fixed = original;
    require(general_planner::retimeRestTrajectory(fixed, 2, 3, 70), "retiming failed");
    require(fixed.getTotalDuration() > duration, "duration not increased");
    for (int i = 0; i <= 1000; ++i) {
      const double u = i / 1000.0, t = u * fixed.getTotalDuration();
      require((fixed.getPos(t) - original.getPos(u * duration)).norm() < 1e-7,
              "spatial path changed");
      require(fixed.getVel(t).norm() <= 2.000001, "velocity limit");
      require(fixed.getAcc(t).norm() <= 3.000001, "acceleration limit");
      require(fixed.getJer(t).norm() <= 70.000001, "jerk limit");
    }
    c(0, 6) = .1;
    Trajectory moving; moving.emplace_back(duration, c);
    require(!general_planner::retimeRestTrajectory(moving, 2, 3, 70),
            "nonzero boundary velocity must not be scaled");
    require(moving.getTotalDuration() == duration, "rejected trajectory mutated");
  }
  std::cout << "rest_trajectory_retiming_self_test passed\n";
}
