#pragma once

#include "traj_opt/convex_hull/adaptive_bezier_forest.hpp"
#include "traj_opt/solver_quality_report.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace traj_opt
{
namespace convex_hull
{

struct ConstraintGeneratorOptions
{
  int top_k_per_leaf{1};
  double positive_threshold{0.0};
  std::size_t max_candidates{256};
};

/**
 * Phase-1 constraint extraction from a certified forest.
 * Phase 2 will pack these into ALM/SQP active sets; here we only surface the
 * worst Bernstein / control indices for reporting and later correction.
 */
inline std::vector<ConstraintCandidate> extractViolatedCandidates(
    const AdaptiveBezierForest &forest,
    const ConstraintGeneratorOptions &options = {})
{
  std::vector<ConstraintCandidate> candidates;
  const auto &leaves = forest.leaves();
  for (std::size_t leaf_id = 0; leaf_id < leaves.size(); ++leaf_id)
  {
    const auto &leaf = leaves[leaf_id];
    if (!leaf.is_active)
    {
      continue;
    }
    if (leaf.status != LeafStatus::VIOLATED &&
        leaf.status != LeafStatus::MAX_DEPTH_REACHED &&
        leaf.upper_bound <= options.positive_threshold)
    {
      continue;
    }
    if (leaf.upper_bound <= options.positive_threshold &&
        leaf.status != LeafStatus::VIOLATED)
    {
      continue;
    }

    ConstraintCandidate candidate;
    candidate.kind =
        leaf.derivative_order == 0
            ? ConstraintKind::PolynomialPosition
            : ConstraintKind::PolynomialDerivativeNorm;
    candidate.source_segment = leaf.source_segment;
    candidate.derivative_order = leaf.derivative_order;
    candidate.leaf_id = static_cast<int>(leaf_id);
    candidate.depth = leaf.depth;
    candidate.binary_index = leaf.binary_index;
    candidate.control_or_bernstein_index = leaf.worst_constraint;
    candidate.plane_id = leaf.plane_id;
    candidate.value = leaf.upper_bound;
    candidate.margin = leaf.robust_margin;
    candidate.historical_multiplier = 0.0;
    candidates.push_back(candidate);
  }

  std::sort(candidates.begin(),
            candidates.end(),
            [](const ConstraintCandidate &a, const ConstraintCandidate &b) {
              return a.value > b.value;
            });
  if (candidates.size() > options.max_candidates)
  {
    candidates.resize(options.max_candidates);
  }

  // Keep at most top_k per leaf.
  if (options.top_k_per_leaf > 0)
  {
    std::vector<int> kept_per_leaf(leaves.size(), 0);
    std::vector<ConstraintCandidate> filtered;
    filtered.reserve(candidates.size());
    for (const auto &candidate : candidates)
    {
      if (candidate.leaf_id < 0 ||
          candidate.leaf_id >= static_cast<int>(kept_per_leaf.size()))
      {
        continue;
      }
      int &count = kept_per_leaf[static_cast<std::size_t>(candidate.leaf_id)];
      if (count >= options.top_k_per_leaf)
      {
        continue;
      }
      ++count;
      filtered.push_back(candidate);
    }
    candidates.swap(filtered);
  }
  return candidates;
}

} // namespace convex_hull
} // namespace traj_opt
