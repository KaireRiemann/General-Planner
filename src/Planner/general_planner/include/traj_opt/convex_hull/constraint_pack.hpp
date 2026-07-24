#pragma once

#include "traj_opt/solver_quality_report.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <vector>

namespace traj_opt
{
namespace convex_hull
{

struct PackedConstraint
{
  int source_segment{-1};
  int derivative_order{0};
  int depth{0};
  int binary_index{0};
  int control_or_bernstein_index{0};
  int plane_id{-1};
  double seed_multiplier{0.0};
};

struct PackedConstraintSet
{
  std::vector<PackedConstraint> constraints;
  std::uint64_t topology_signature{0};
  int top_k{0};
};

inline bool samePackedIdentity(const PackedConstraint &a,
                               const PackedConstraint &b)
{
  return a.source_segment == b.source_segment &&
         a.derivative_order == b.derivative_order &&
         a.depth == b.depth &&
         a.binary_index == b.binary_index &&
         a.control_or_bernstein_index == b.control_or_bernstein_index &&
         a.plane_id == b.plane_id;
}

inline std::uint64_t hashCombine(std::uint64_t seed, std::uint64_t value)
{
  seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
  return seed;
}

inline std::uint64_t topologySignature(
    const std::vector<PackedConstraint> &constraints)
{
  std::uint64_t signature = 1469598103934665603ULL;
  signature = hashCombine(signature,
                          static_cast<std::uint64_t>(constraints.size()));
  for (const auto &constraint : constraints)
  {
    signature = hashCombine(
        signature, static_cast<std::uint64_t>(constraint.source_segment + 1));
    signature = hashCombine(
        signature,
        static_cast<std::uint64_t>(constraint.derivative_order + 1));
    signature = hashCombine(
        signature, static_cast<std::uint64_t>(constraint.depth + 1));
    signature = hashCombine(
        signature,
        static_cast<std::uint64_t>(constraint.binary_index + 1));
    signature = hashCombine(
        signature,
        static_cast<std::uint64_t>(constraint.control_or_bernstein_index +
                                   1));
    signature = hashCombine(
        signature, static_cast<std::uint64_t>(constraint.plane_id + 2));
  }
  return signature;
}

/**
 * Pack oracle top-K violations into a frozen ALM/SQP active set.
 * Duplicates (same leaf identity) are collapsed, keeping the worse value.
 */
inline PackedConstraintSet packConstraintCandidates(
    const std::vector<ConstraintCandidate> &candidates,
    int top_k,
    const Eigen::VectorXd *previous_multipliers = nullptr,
    const PackedConstraintSet *previous_set = nullptr)
{
  PackedConstraintSet packed;
  packed.top_k = std::max(0, top_k);

  std::vector<ConstraintCandidate> ordered = candidates;
  std::sort(ordered.begin(),
            ordered.end(),
            [](const ConstraintCandidate &a, const ConstraintCandidate &b) {
              return a.value > b.value;
            });

  for (const auto &candidate : ordered)
  {
    if (candidate.source_segment < 0 || candidate.derivative_order < 0)
    {
      continue;
    }
    PackedConstraint constraint;
    constraint.source_segment = candidate.source_segment;
    constraint.derivative_order = candidate.derivative_order;
    constraint.depth = std::max(0, candidate.depth);
    constraint.binary_index = std::max(0, candidate.binary_index);
    constraint.control_or_bernstein_index =
        std::max(0, candidate.control_or_bernstein_index);
    constraint.plane_id = candidate.plane_id;
    constraint.seed_multiplier =
        std::max(0.0, candidate.historical_multiplier);

    bool duplicate = false;
    for (auto &existing : packed.constraints)
    {
      if (samePackedIdentity(existing, constraint))
      {
        duplicate = true;
        break;
      }
    }
    if (duplicate)
    {
      continue;
    }

    if (previous_set != nullptr &&
        previous_multipliers != nullptr &&
        previous_multipliers->size() ==
            static_cast<Eigen::Index>(previous_set->constraints.size()))
    {
      for (std::size_t i = 0; i < previous_set->constraints.size(); ++i)
      {
        if (samePackedIdentity(previous_set->constraints[i], constraint))
        {
          constraint.seed_multiplier =
              std::max(0.0, (*previous_multipliers)(static_cast<Eigen::Index>(i)));
          break;
        }
      }
    }

    packed.constraints.push_back(constraint);
    if (packed.top_k > 0 &&
        static_cast<int>(packed.constraints.size()) >= packed.top_k)
    {
      break;
    }
  }

  packed.topology_signature = topologySignature(packed.constraints);
  return packed;
}

inline Eigen::VectorXd seedMultipliers(const PackedConstraintSet &packed)
{
  Eigen::VectorXd multipliers(
      static_cast<Eigen::Index>(packed.constraints.size()));
  for (std::size_t i = 0; i < packed.constraints.size(); ++i)
  {
    multipliers(static_cast<Eigen::Index>(i)) =
        std::max(0.0, packed.constraints[i].seed_multiplier);
  }
  return multipliers;
}

/**
 * Append-only pack growth: keep previous rows (and their multipliers) and
 * append new identities from candidates. Used between PHR outers.
 */
inline bool appendPackedCandidates(
    PackedConstraintSet &packed,
    Eigen::VectorXd &multipliers,
    const std::vector<ConstraintCandidate> &candidates,
    int max_total)
{
  if (max_total <= 0)
  {
    max_total = static_cast<int>(packed.constraints.size());
  }
  bool changed = false;
  std::vector<ConstraintCandidate> ordered = candidates;
  std::sort(ordered.begin(),
            ordered.end(),
            [](const ConstraintCandidate &a, const ConstraintCandidate &b) {
              return a.value > b.value;
            });
  for (const auto &candidate : ordered)
  {
    if (static_cast<int>(packed.constraints.size()) >= max_total)
    {
      break;
    }
    PackedConstraint constraint;
    constraint.source_segment = candidate.source_segment;
    constraint.derivative_order = candidate.derivative_order;
    constraint.depth = std::max(0, candidate.depth);
    constraint.binary_index = std::max(0, candidate.binary_index);
    constraint.control_or_bernstein_index =
        std::max(0, candidate.control_or_bernstein_index);
    constraint.plane_id = candidate.plane_id;
    constraint.seed_multiplier = 0.0;
    bool exists = false;
    for (const auto &existing : packed.constraints)
    {
      if (samePackedIdentity(existing, constraint))
      {
        exists = true;
        break;
      }
    }
    if (exists)
    {
      continue;
    }
    packed.constraints.push_back(constraint);
    Eigen::VectorXd grown(multipliers.size() + 1);
    if (multipliers.size() > 0)
    {
      grown.head(multipliers.size()) = multipliers;
    }
    grown(multipliers.size()) = 0.0;
    multipliers.swap(grown);
    changed = true;
  }
  if (changed)
  {
    packed.topology_signature = topologySignature(packed.constraints);
  }
  return changed;
}

} // namespace convex_hull
} // namespace traj_opt
