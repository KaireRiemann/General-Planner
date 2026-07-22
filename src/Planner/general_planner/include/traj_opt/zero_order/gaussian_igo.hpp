#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <random>
#include <thread>
#include <utility>
#include <vector>

namespace traj_opt::zero_order
{

struct GaussianIgoOptions
{
  int population{24};
  int generations{5};
  int threads{1};
  int no_feasible_expand_generations{3};
  double elite_ratio{0.25};
  double mean_learning_rate{0.8};
  double covariance_learning_rate{0.2};
  double min_eigenvalue{1.0e-6};
  double max_condition_number{1.0e6};
  double covariance_expand_factor{1.6};
  bool antithetic{true};
  std::uint64_t seed{7};
};

template <typename Evaluation>
struct GaussianIgoResult
{
  bool has_feasible{false};
  bool timed_out{false};
  int evaluations{0};
  int generations{0};
  double final_feasible_ratio{0.0};
  Eigen::VectorXd final_mean;
  Eigen::MatrixXd final_covariance;
  std::optional<Evaluation> best_feasible;
  std::optional<Evaluation> best_infeasible;
  Eigen::VectorXd best_feasible_decision;
  Eigen::VectorXd best_infeasible_decision;
};

// The optimizer deliberately knows nothing about penalties or trajectories.  Its
// only ordering is the strict weak ordering supplied by Better, which lets the
// state2state adapter implement feasible-first, lexicographic constraint ranks.
class GaussianIgoSolver
{
public:
  template <typename Evaluation, typename Evaluate, typename Better, typename Feasible>
  GaussianIgoResult<Evaluation> optimize(
      const Eigen::VectorXd &initial_mean,
      const Eigen::MatrixXd &initial_covariance,
      const std::vector<Eigen::VectorXd> &injected_decisions,
      const GaussianIgoOptions &options,
      const std::chrono::steady_clock::time_point &deadline,
      Evaluate &&evaluate,
      Better &&better,
      Feasible &&feasible) const
  {
    GaussianIgoResult<Evaluation> result;
    const int dimension = initial_mean.size();
    if (dimension <= 0 || initial_covariance.rows() != dimension ||
        initial_covariance.cols() != dimension)
    {
      return result;
    }

    const int population = std::max(4, options.population);
    const int elite_count = std::max(
        2, std::min(population, static_cast<int>(std::ceil(
                                   population * std::clamp(options.elite_ratio, 0.05, 1.0)))));
    Eigen::VectorXd mean = initial_mean;
    Eigen::MatrixXd covariance = regularizeCovariance(initial_covariance, options);
    std::mt19937_64 rng(options.seed);
    std::normal_distribution<double> normal(0.0, 1.0);
    int generations_without_feasible = 0;

    struct Entry
    {
      Eigen::VectorXd decision;
      std::optional<Evaluation> evaluation;
    };

    for (int generation = 0; generation < std::max(1, options.generations); ++generation)
    {
      if (std::chrono::steady_clock::now() >= deadline)
      {
        result.timed_out = true;
        break;
      }

      covariance = regularizeCovariance(covariance, options);
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(covariance);
      if (eig.info() != Eigen::Success)
      {
        break;
      }
      const Eigen::MatrixXd transform =
          eig.eigenvectors() * eig.eigenvalues().cwiseSqrt().asDiagonal();

      std::vector<Entry> entries(static_cast<std::size_t>(population));
      int cursor = 0;
      entries[static_cast<std::size_t>(cursor++)].decision = mean;
      for (const auto &decision : injected_decisions)
      {
        if (cursor >= population)
        {
          break;
        }
        if (decision.size() == dimension && decision.allFinite())
        {
          entries[static_cast<std::size_t>(cursor++)].decision = decision;
        }
      }
      while (cursor < population)
      {
        Eigen::VectorXd noise(dimension);
        for (int i = 0; i < dimension; ++i)
        {
          noise(i) = normal(rng);
        }
        const Eigen::VectorXd delta = transform * noise;
        entries[static_cast<std::size_t>(cursor++)].decision = mean + delta;
        if (options.antithetic && cursor < population)
        {
          entries[static_cast<std::size_t>(cursor++)].decision = mean - delta;
        }
      }

      const int worker_count = std::max(1, std::min(options.threads, population));
      std::atomic<int> next_index{0};
      auto worker = [&]() {
        while (true)
        {
          const int index = next_index.fetch_add(1);
          if (index >= population)
          {
            return;
          }
          entries[static_cast<std::size_t>(index)].evaluation =
              evaluate(entries[static_cast<std::size_t>(index)].decision);
        }
      };
      std::vector<std::thread> workers;
      workers.reserve(static_cast<std::size_t>(worker_count - 1));
      for (int i = 1; i < worker_count; ++i)
      {
        workers.emplace_back(worker);
      }
      worker();
      for (auto &thread : workers)
      {
        thread.join();
      }

      result.evaluations += population;
      result.generations = generation + 1;
      std::sort(entries.begin(), entries.end(), [&](const Entry &lhs, const Entry &rhs) {
        return better(*lhs.evaluation, *rhs.evaluation);
      });

      int feasible_count = 0;
      for (const auto &entry : entries)
      {
        if (feasible(*entry.evaluation))
        {
          ++feasible_count;
          if (!result.best_feasible || better(*entry.evaluation, *result.best_feasible))
          {
            result.best_feasible = entry.evaluation;
            result.best_feasible_decision = entry.decision;
          }
        }
        else if (!result.best_infeasible || better(*entry.evaluation, *result.best_infeasible))
        {
          result.best_infeasible = entry.evaluation;
          result.best_infeasible_decision = entry.decision;
        }
      }
      result.final_feasible_ratio = static_cast<double>(feasible_count) / population;
      generations_without_feasible = feasible_count == 0 ? generations_without_feasible + 1 : 0;

      Eigen::VectorXd weights(elite_count);
      for (int i = 0; i < elite_count; ++i)
      {
        weights(i) = std::log(elite_count + 0.5) - std::log(i + 1.0);
      }
      weights /= weights.sum();
      const Eigen::VectorXd old_mean = mean;
      Eigen::VectorXd ranked_step = Eigen::VectorXd::Zero(dimension);
      for (int i = 0; i < elite_count; ++i)
      {
        ranked_step.noalias() +=
            weights(i) * (entries[static_cast<std::size_t>(i)].decision - old_mean);
      }
      mean.noalias() += std::clamp(options.mean_learning_rate, 0.0, 1.0) * ranked_step;

      Eigen::MatrixXd ranked_covariance = Eigen::MatrixXd::Zero(dimension, dimension);
      for (int i = 0; i < elite_count; ++i)
      {
        const Eigen::VectorXd delta = entries[static_cast<std::size_t>(i)].decision - old_mean;
        ranked_covariance.noalias() += weights(i) * delta * delta.transpose();
      }
      const double covariance_rate = std::clamp(options.covariance_learning_rate, 0.0, 1.0);
      covariance = (1.0 - covariance_rate) * covariance + covariance_rate * ranked_covariance;
      if (generations_without_feasible >= std::max(1, options.no_feasible_expand_generations))
      {
        covariance *= std::max(1.0, options.covariance_expand_factor);
        generations_without_feasible = 0;
      }
    }

    result.has_feasible = result.best_feasible.has_value();
    result.final_mean = mean;
    result.final_covariance = regularizeCovariance(covariance, options);
    return result;
  }

private:
  static Eigen::MatrixXd regularizeCovariance(const Eigen::MatrixXd &input,
                                               const GaussianIgoOptions &options)
  {
    Eigen::MatrixXd symmetric = 0.5 * (input + input.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(symmetric);
    if (eig.info() != Eigen::Success)
    {
      return Eigen::MatrixXd::Identity(input.rows(), input.cols());
    }
    Eigen::VectorXd values = eig.eigenvalues();
    const double floor = std::max(1.0e-12, options.min_eigenvalue);
    const double ceiling = floor * std::max(1.0, options.max_condition_number);
    values = values.cwiseMax(floor).cwiseMin(ceiling);
    return eig.eigenvectors() * values.asDiagonal() * eig.eigenvectors().transpose();
  }
};

}  // namespace traj_opt::zero_order
