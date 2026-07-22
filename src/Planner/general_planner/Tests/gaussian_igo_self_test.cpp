#include <traj_opt/zero_order/gaussian_igo.hpp>

#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <iostream>

namespace
{
struct Evaluation
{
  bool feasible{false};
  double violation{0.0};
  double objective{0.0};
};

bool better(const Evaluation &lhs, const Evaluation &rhs)
{
  if (lhs.feasible != rhs.feasible)
  {
    return lhs.feasible;
  }
  if (!lhs.feasible && lhs.violation != rhs.violation)
  {
    return lhs.violation < rhs.violation;
  }
  return lhs.objective < rhs.objective;
}
}  // namespace

int main()
{
  using namespace traj_opt::zero_order;
  GaussianIgoOptions options;
  options.population = 32;
  options.generations = 12;
  options.threads = 4;
  options.seed = 19;
  options.elite_ratio = 0.25;

  const Eigen::VectorXd mean = Eigen::VectorXd::Constant(4, 1.5);
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(4, 4);
  covariance(0, 1) = covariance(1, 0) = 0.35;
  auto evaluate = [](const Eigen::VectorXd &decision) {
    Evaluation evaluation;
    const Eigen::VectorXd target = Eigen::VectorXd::Constant(4, 0.2);
    evaluation.objective = (decision - target).squaredNorm();
    evaluation.violation = std::max(0.0, decision.norm() - 1.2);
    evaluation.feasible = evaluation.violation <= 1.0e-12;
    return evaluation;
  };
  auto run = [&]() {
    GaussianIgoSolver solver;
    return solver.optimize<Evaluation>(
        mean,
        covariance,
        {Eigen::VectorXd::Zero(4)},
        options,
        std::chrono::steady_clock::now() + std::chrono::seconds(2),
        evaluate,
        better,
        [](const Evaluation &evaluation) { return evaluation.feasible; });
  };

  const auto first = run();
  const auto second = run();
  if (!first.has_feasible || !first.best_feasible ||
      first.best_feasible->objective > 0.01)
  {
    std::cerr << "Gaussian IGO failed to retain a feasible archive." << std::endl;
    return 1;
  }
  if (!second.best_feasible ||
      std::abs(first.best_feasible->objective - second.best_feasible->objective) > 1.0e-12 ||
      (first.best_feasible_decision - second.best_feasible_decision).norm() > 1.0e-12)
  {
    std::cerr << "Seeded Gaussian IGO is not deterministic." << std::endl;
    return 2;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> covariance_eigen(first.final_covariance);
  if (covariance_eigen.info() != Eigen::Success ||
      covariance_eigen.eigenvalues().minCoeff() <= 0.0 ||
      covariance_eigen.eigenvalues().maxCoeff() /
              covariance_eigen.eigenvalues().minCoeff() >
          options.max_condition_number * 1.001)
  {
    std::cerr << "Covariance regularization contract is broken." << std::endl;
    return 4;
  }

  GaussianIgoSolver solver;
  const auto impossible = solver.optimize<Evaluation>(
      Eigen::VectorXd::Zero(2),
      Eigen::MatrixXd::Identity(2, 2),
      {},
      options,
      std::chrono::steady_clock::now() + std::chrono::seconds(2),
      [](const Eigen::VectorXd &decision) {
        return Evaluation{false, 1.0 + decision.squaredNorm(), decision.squaredNorm()};
      },
      better,
      [](const Evaluation &evaluation) { return evaluation.feasible; });
  if (impossible.has_feasible || !impossible.best_infeasible)
  {
    std::cerr << "No-feasible status contract is broken." << std::endl;
    return 3;
  }

  std::cout << "gaussian_igo_self_test: PASS, objective="
            << first.best_feasible->objective
            << ", evaluations=" << first.evaluations << std::endl;
  return 0;
}
