#pragma once

#include "traj_opt/convex_hull/bezier_product.hpp"
#include "traj_opt/convex_hull/scalar_bernstein.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace traj_opt
{
namespace convex_hull
{

struct BezierLeaf
{
  int source_segment{0};
  int derivative_order{0};
  int depth{0};
  int binary_index{0};

  double u_begin{0.0};
  double u_end{1.0};

  int parent{-1};
  int left_child{-1};
  int right_child{-1};

  LeafStatus status{LeafStatus::UNCERTAIN};

  double lower_bound{0.0};
  double upper_bound{0.0};
  double robust_margin{0.0};

  int worst_constraint{0};
  int plane_id{-1};
  bool is_active{true};

  Eigen::MatrixXd controls;
};

struct ForestOptions
{
  int max_depth{6};
  double safe_margin{0.05};
  double bound_gap_tol{1.0e-6};
  double refine_margin{0.05};
  double coarsen_margin{0.10};
  bool enable_hysteresis{true};
  std::size_t max_leaves{4096};
};

/**
 * Adaptive de Casteljau forest for one (segment, order[, plane]) certificate
 * stream. Topology may only change between certificate calls; leaves keep
 * hysteresis so near-boundary residual does not thrash refine/coarsen.
 */
class AdaptiveBezierForest
{
public:
  void configure(const ForestOptions &options)
  {
    options_ = options;
    options_.max_depth = std::clamp(options_.max_depth, 0, 12);
    options_.safe_margin = std::max(0.0, options_.safe_margin);
    options_.bound_gap_tol = std::max(0.0, options_.bound_gap_tol);
    options_.refine_margin = std::max(0.0, options_.refine_margin);
    options_.coarsen_margin =
        std::max(options_.refine_margin, options_.coarsen_margin);
  }

  void clear()
  {
    leaves_.clear();
    root_ = -1;
  }

  void seed(int source_segment,
            int derivative_order,
            int plane_id,
            const Eigen::Ref<const Eigen::MatrixXd> &controls,
            double u_begin = 0.0,
            double u_end = 1.0)
  {
    clear();
    BezierLeaf leaf;
    leaf.source_segment = source_segment;
    leaf.derivative_order = derivative_order;
    leaf.plane_id = plane_id;
    leaf.depth = 0;
    leaf.binary_index = 0;
    leaf.u_begin = u_begin;
    leaf.u_end = u_end;
    leaf.controls = controls;
    leaf.is_active = true;
    leaves_.push_back(leaf);
    root_ = 0;
  }

  const std::vector<BezierLeaf> &leaves() const { return leaves_; }

  std::vector<int> activeLeafIds() const
  {
    std::vector<int> ids;
    for (std::size_t i = 0; i < leaves_.size(); ++i)
    {
      if (leaves_[i].is_active)
      {
        ids.push_back(static_cast<int>(i));
      }
    }
    return ids;
  }

  int maxDepthUsed() const
  {
    int depth = 0;
    for (const auto &leaf : leaves_)
    {
      if (leaf.is_active)
      {
        depth = std::max(depth, leaf.depth);
      }
    }
    return depth;
  }

  /**
   * Update scalar bounds / status for a half-space leaf.
   */
  LeafStatus evaluateHalfSpace(int leaf_id,
                               const Eigen::Vector3d &normal,
                               double offset)
  {
    auto &leaf = leaves_.at(static_cast<std::size_t>(leaf_id));
    const ScalarBound bound =
        halfSpaceBounds(leaf.controls, normal, offset, 0.0);
    leaf.lower_bound = bound.lower;
    leaf.upper_bound = bound.upper;
    leaf.worst_constraint =
        bound.upper > 0.0 ? bound.argmax : bound.argmin;
    leaf.robust_margin = -bound.upper;
    leaf.status = classifyScalarLeaf(
        bound, options_.safe_margin, options_.bound_gap_tol);
    if (leaf.status == LeafStatus::UNCERTAIN &&
        leaf.depth >= options_.max_depth)
    {
      leaf.status = LeafStatus::MAX_DEPTH_REACHED;
    }
    return leaf.status;
  }

  /**
   * Level A (vector hull) then Level B (squared-norm Bernstein) for a
   * Euclidean ball constraint ||v|| <= bound.
   */
  LeafStatus evaluateNormBound(int leaf_id, double bound)
  {
    auto &leaf = leaves_.at(static_cast<std::size_t>(leaf_id));
    if (!std::isfinite(bound) || bound <= 0.0)
    {
      leaf.lower_bound = 0.0;
      leaf.upper_bound = 0.0;
      leaf.status = LeafStatus::SAFE;
      leaf.robust_margin = std::numeric_limits<double>::infinity();
      return leaf.status;
    }

    // Store nondimensional residuals (||v||^2 / bound^2 - 1), matching
    // packed PHR / ALM so oracle and correction share one scale.
    const double bound_sq = bound * bound;

    // Level A: vector control-point ball (fast, conservative).
    const ScalarBound hull = vectorHullNormBound(leaf.controls, bound);
    const ScalarBound hull_normalized{hull.lower / bound_sq,
                                    hull.upper / bound_sq,
                                    hull.argmin,
                                    hull.argmax};
    if (hull_normalized.upper <= -options_.safe_margin)
    {
      leaf.lower_bound = hull_normalized.lower;
      leaf.upper_bound = hull_normalized.upper;
      leaf.worst_constraint = hull_normalized.argmax;
      leaf.robust_margin = -hull_normalized.upper;
      leaf.status = LeafStatus::SAFE;
      return leaf.status;
    }

    // Level B: squared-norm Bernstein (tighter continuous certificate).
    const ScalarBound bernstein = squaredNormBoundBounds(leaf.controls, bound);
    const ScalarBound bernstein_normalized{bernstein.lower / bound_sq,
                                         bernstein.upper / bound_sq,
                                         bernstein.argmin,
                                         bernstein.argmax};
    leaf.lower_bound = bernstein_normalized.lower;
    leaf.upper_bound = bernstein_normalized.upper;
    leaf.worst_constraint = bernstein_normalized.argmax;
    leaf.robust_margin = -bernstein_normalized.upper;
    leaf.status = classifyScalarLeaf(
        bernstein_normalized, options_.safe_margin, options_.bound_gap_tol);
    if (leaf.status == LeafStatus::UNCERTAIN &&
        leaf.depth >= options_.max_depth)
    {
      leaf.status = LeafStatus::MAX_DEPTH_REACHED;
    }
    return leaf.status;
  }

  /**
   * Split an active uncertain leaf at midpoint. Returns left child id, or
   * -1 if the split was refused (budget / max depth / hysteresis).
   */
  int split(int leaf_id)
  {
    auto &leaf = leaves_.at(static_cast<std::size_t>(leaf_id));
    if (!leaf.is_active || leaf.left_child >= 0)
    {
      return leaf.left_child;
    }
    if (leaf.depth >= options_.max_depth ||
        leaves_.size() + 2 > options_.max_leaves)
    {
      leaf.status = LeafStatus::MAX_DEPTH_REACHED;
      return -1;
    }
    if (options_.enable_hysteresis &&
        leaf.upper_bound <= -options_.refine_margin &&
        leaf.upper_bound > -options_.coarsen_margin)
    {
      // Near-safe band: do not refine further.
      return -1;
    }

    const auto children = deCasteljauSplit(leaf.controls, 0.5);
    const double u_mid = 0.5 * (leaf.u_begin + leaf.u_end);

    BezierLeaf left = leaf;
    left.parent = leaf_id;
    left.left_child = -1;
    left.right_child = -1;
    left.depth = leaf.depth + 1;
    left.binary_index = leaf.binary_index * 2;
    left.u_begin = leaf.u_begin;
    left.u_end = u_mid;
    left.controls = children.first;
    left.is_active = true;
    left.status = LeafStatus::UNCERTAIN;

    BezierLeaf right = left;
    right.binary_index = leaf.binary_index * 2 + 1;
    right.u_begin = u_mid;
    right.u_end = leaf.u_end;
    right.controls = children.second;

    const int left_id = static_cast<int>(leaves_.size());
    const int right_id = left_id + 1;
    leaves_.push_back(left);
    leaves_.push_back(right);
    leaf.left_child = left_id;
    leaf.right_child = right_id;
    leaf.is_active = false;
    return left_id;
  }

private:
  ForestOptions options_{};
  std::vector<BezierLeaf> leaves_;
  int root_{-1};
};

} // namespace convex_hull
} // namespace traj_opt
