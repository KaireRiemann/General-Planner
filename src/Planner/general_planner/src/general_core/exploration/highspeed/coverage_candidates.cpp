#include <general_core/exploration/highspeed/fast_exploration_manager.h>
#include <general_core/exploration/highspeed/expl_data.h>
#include <general_core/exploration/highspeed/coverage_candidate_policy.h>
#include <algorithm>
#include <cmath>

namespace fast_planner {

bool FastExplorationManager::jointCoverageCandidatesEnabled() const {
  return coverage_joint_candidates_ && coverageMotionEnabled() &&
      !coverageRouteEnabled() && coverage_guidance_ &&
      coverage_guidance_->safetyNetEnabled();
}

bool FastExplorationManager::activeCoverageObservation(CoverageObservationGate &gate) const {
  if (!coverageMotionEnabled() || !has_active_coverage_goal_) return false;
  gate.goal = active_coverage_target_.approach_position;
  gate.radius = std::max(0.05, coverage_recovery_reached_radius_);
  gate.identity = active_coverage_target_.stable_id;
  const Eigen::Vector3d direction = active_coverage_target_.position - gate.goal;
  gate.yaw = std::hypot(direction.x(), direction.y()) > 1e-3
      ? std::atan2(direction.y(), direction.x()) : planner_manager_->local_data_.curr_yaw_;
  gate.yaw_tolerance = 0.75;
  return gate.goal.allFinite();
}

FastExplorationManager::CoverageCandidatesResult
FastExplorationManager::appendCoverageCandidates(
    const Eigen::Vector3d &pos, double current_speed, double curr_yaw,
    std::size_t viewpoint_count_before_defer, const CoveragePlan::Ptr &coverage_snapshot,
    vector<TopoNode::Ptr> &viewpoints) {
  CoverageCandidatesResult result;
  if (targetDirectedModeConfigured()) return result;
  const bool target_directed = false;
  const bool structural_coverage = coverageRouteEnabled();
  const ros::Time coverage_now = ros::Time::now();
  const int active_clusters = frontier_manager_ptr_->activeClusterCount();
  const int reachable_clusters =
      frontier_manager_ptr_->reachableClusterCount();
  // The frontier debounce controls legacy fallback and completion only.
  // Joint tours may admit observation approaches while frontiers still exist.
  const bool no_executable_frontier = viewpoints.empty();
  if (!no_executable_frontier) {
    coverage_executable_empty_count_ = 0;
    coverage_executable_empty_since_ = ros::Time(0);
  } else {
    if (coverage_executable_empty_since_.isZero()) {
      coverage_executable_empty_since_ = coverage_now;
      coverage_executable_empty_count_ = 1;
    } else {
      ++coverage_executable_empty_count_;
    }
  }
  const double executable_empty_duration =
      coverage_executable_empty_since_.isZero()
          ? 0.0
          : (coverage_now - coverage_executable_empty_since_).toSec();
  const bool executable_empty_stable =
      no_executable_frontier &&
      ((coverageMotionEnabled() && viewpoint_count_before_defer > 0 &&
        coverage_executable_empty_count_ >= 2 && executable_empty_duration >= 0.20) ||
      (coverage_executable_empty_count_ >=
          coverage_executable_empty_min_count_ &&
      executable_empty_duration >=
          coverage_executable_empty_min_duration_));
  const bool moving_handoff_ready =
      coverage_moving_handoff_enable_ &&
      planner_manager_->hasCommittedTrajectory();
  // End the fallback phase at an action boundary, not after exhausting every
  // regenerated unknown-volume approach. The FSM will force/validate the full
  // frontier audit and independently require repeated empty results and rest.
  // Never interrupt an active observation or mistake cooldown for absence of
  // reachable frontiers; renewed measured gain/frontiers reopen this gate.
  // The pre-cooldown pool is an independent veto: a validated viewpoint must
  // not disappear from completion merely because it is temporarily deferred.
  updateCoverageCompletion(std::max(reachable_clusters, static_cast<int>(viewpoint_count_before_defer)),
      executable_empty_stable,
      coverage_guidance_ && coverage_guidance_->finishGuardEnabled() && coverage_snapshot && coverage_snapshot->valid,
      coverage_snapshot ? coverage_snapshot->observed_voxel_count : -1);
  if (coverageMotionEnabled() && coverage_terminal_audit_pending_ && !has_active_coverage_goal_) {
    last_plan_empty_frontier_=active_clusters==0;
    last_plan_no_reachable_=active_clusters>0;
    ROS_INFO_STREAM_THROTTLE(1.0,"[coverage handoff] frontier audit takes precedence over new CP cleanup goals");
    result.terminal_audit = true;
    return result;
  }
  const bool recovery_enabled = !target_directed && coverage_guidance_ &&
      coverage_guidance_->safetyNetEnabled();
  const auto admission = coverage_candidates::admission(
      recovery_enabled && coverage_executable_candidate_enable_,
      jointCoverageCandidatesEnabled(), no_executable_frontier,
      executable_empty_stable, structural_coverage, has_active_coverage_goal_,
      coverage_terminal_audit_pending_, moving_handoff_ready,
      current_speed, coverage_executable_candidate_max_speed_);
  const bool coverage_handoff_pending = admission.waiting;
  result.handoff_pending = coverage_handoff_pending;
  if (coverage_handoff_pending) {
    ROS_INFO_STREAM_THROTTLE(
        0.5, "[coverage handoff] wait for stable executable-frontier-empty "
                 "count="
                 << coverage_executable_empty_count_ << "/"
                 << coverage_executable_empty_min_count_ << " duration="
                 << executable_empty_duration << "/"
                 << coverage_executable_empty_min_duration_
                 << "s raw_active=" << active_clusters
                 << " raw_reachable=" << reachable_clusters
                 << " speed=" << current_speed << "/"
                 << coverage_executable_candidate_max_speed_
                 << " moving_handoff=" << moving_handoff_ready
                 << " blocker="
                 << (!executable_empty_stable ? "empty_debounce"
                                              : "vehicle_speed"));
  }
  auto &priority_floor_active = result.priority_floor_active;
  auto &ascending_to_priority_floor = result.ascending_to_priority_floor;
  auto &first_priority_floor_rank = result.first_priority_floor_rank;
  auto isPriorityFloorTarget = [&](const CoverageTarget &target) {
    return !jointCoverageCandidatesEnabled() && !structural_coverage && coverage_floor_priority_enable_ &&
           target.position.z() >= coverage_floor_priority_min_z_;
  };
  if (admission.admit_new || admission.retain_active) {
    auto coverage_targets = admission.admit_new
        ? coverage_guidance_->unknownApproachTargets(pos, 160, 0.8)
        : std::vector<CoverageTarget>{};
    if (structural_coverage && admission.admit_new) {
      coverage_targets.clear();
      if (coverage_snapshot) for (const auto &target : coverage_snapshot->ordered_targets)
        if (target.type == CoverageTargetType::REACHABLE_UNKNOWN && target.has_approach &&
            (target.approach_position-pos).norm() >= .8) coverage_targets.push_back(target);
      // The persistent component inventory remains the recovery authority,
      // even when the current sparse CP has no usable approach.
      if (coverage_targets.empty())
        coverage_targets = coverage_guidance_->unknownApproachTargets(pos, 160, 0.8);
    }
    // Canonicalize the executable approach before consulting recovery state.
    // The raw approach_position is only a component hint; the selected entry
    // from approach_candidates is the action that is actually inserted into
    // the topology graph. Checking cooldown/exhaustion before this step let
    // two regenerated ids mapped to one disconnected approach bypass each
    // other's terminal record indefinitely.
    std::vector<CoverageTarget> canonical_coverage_targets;
    canonical_coverage_targets.reserve(coverage_targets.size());
    for (CoverageTarget target : coverage_targets) {
      if (coverageRecoveryExhausted(target)) {
        continue;
      }
      if (!selectSafeCoverageApproach(target, false)) {
        continue;
      }
      if (coverageRecoveryExhausted(target)) {
        rememberCoverageRecoveryAlias(target);
        continue;
      }
      canonical_coverage_targets.emplace_back(std::move(target));
    }
    coverage_targets.swap(canonical_coverage_targets);
    const bool active_goal_is_priority =
        has_active_coverage_goal_ &&
        isPriorityFloorTarget(active_coverage_target_);
    // Once ordinary frontiers are exhausted in a multi-floor scene, finish
    // the executable upper-floor pool before returning to lower-floor
    // perimeter cleanup. Preserve an already active lower-floor goal, then
    // switch floors at the next handoff.
    priority_floor_active =
        coverage_floor_priority_enable_ &&
        (!has_active_coverage_goal_ || active_goal_is_priority) &&
        std::any_of(
            coverage_targets.begin(), coverage_targets.end(),
            [&](const CoverageTarget &target) {
              return isPriorityFloorTarget(target) &&
                     !coverageRecoveryDeferred(target, coverage_now);
            });
    if (priority_floor_active) {
      for (const CoverageTarget &target : coverage_targets) {
        if (isPriorityFloorTarget(target) &&
            !coverageRecoveryDeferred(target, coverage_now)) {
          first_priority_floor_rank =
              std::min(first_priority_floor_rank, target.route_rank);
        }
      }
      ascending_to_priority_floor =
          pos.z() < coverage_floor_priority_min_z_ - 0.4 &&
          first_priority_floor_rank != std::numeric_limits<int>::max();
    }
    // When the robot is still below the priority floor, retain the short CP
    // prefix immediately preceding its first upper-floor observation. Those
    // lower-z nodes describe the staircase/doorway transition in the free-zone
    // graph. A pure z filter discarded them and asked MINCO to connect
    // directly to scattered upper-floor endpoints.
    auto isFloorPhaseTarget = [&](const CoverageTarget &target) {
      if (!priority_floor_active) {
        return true;
      }
      if (!ascending_to_priority_floor) {
        return isPriorityFloorTarget(target);
      }
      const int transition_rank_begin =
          std::max(0, first_priority_floor_rank -
                          coverage_floor_transition_rank_window_);
      return target.route_rank >= transition_rank_begin &&
             target.route_rank <= first_priority_floor_rank;
    };
    // The persistent CP route remains a long-horizon guide, but execution is
    // receding-horizon: expose several nearby/high-gain observations to the
    // real topology cost instead of blindly taking the first two CP nodes.
    auto localExecutionScore = [&](const CoverageTarget &target) {
      const double distance =
          target.has_approach
              ? (target.approach_position - pos).norm()
              : std::numeric_limits<double>::infinity();
      const double bounded_rank =
          std::min(40.0, static_cast<double>(std::max(0, target.route_rank)));
      const double bounded_gain =
          std::min(2.0, 0.35 * std::log1p(std::max(0, target.voxel_count)));
      if (structural_coverage) return static_cast<double>(target.route_rank);
      return distance + coverage_route_rank_weight_ * bounded_rank -
             bounded_gain;
    };
    std::stable_sort(
        coverage_targets.begin(), coverage_targets.end(),
        [&](const CoverageTarget &first, const CoverageTarget &second) {
          return localExecutionScore(first) < localExecutionScore(second);
        });
    if (has_active_coverage_goal_) {
      std::stable_sort(
          coverage_targets.begin(), coverage_targets.end(),
          [&](const CoverageTarget &first, const CoverageTarget &second) {
            const bool first_active =
                first.stable_id != 0 &&
                first.stable_id == active_coverage_target_.stable_id;
            const bool second_active =
                second.stable_id != 0 &&
                second.stable_id == active_coverage_target_.stable_id;
            return first_active && !second_active;
          });
    }
    auto appendCoverageViewpoint = [&](const CoverageTarget &target) {
      if (!target.has_approach || !target.approach_position.allFinite()) {
        return false;
      }
      TopoNode::Ptr viewpoint = std::make_shared<TopoNode>();
      viewpoint->is_viewpoint_ = true;
      viewpoint->is_coverage_target_ = true;
      viewpoint->frontier_cluster_id_ = -1;
      viewpoint->coverage_target_id_ = target.stable_id;
      viewpoint->center_ = target.approach_position.cast<float>();
      viewpoint->coverage_unknown_ = target.position.cast<float>();
      viewpoint->coverage_voxel_count_ = target.voxel_count;
      viewpoint->coverage_route_rank_ = target.route_rank;
      viewpoint->frontier_information_gain_ =
          static_cast<double>(std::max(0, target.voxel_count));
      const Eigen::Vector3d observe_direction =
          target.position - target.approach_position;
      viewpoint->yaw_ =
          std::hypot(observe_direction.x(), observe_direction.y()) > 1.0e-3
              ? std::atan2(observe_direction.y(), observe_direction.x())
              : curr_yaw;
      viewpoints.emplace_back(viewpoint);
      return true;
    };

    // Drain the current bounded action before the full FSM audit. Do not
    // let a newly generated speculative CP node start another cleanup tour.
    if (coverage_terminal_audit_pending_) coverage_targets.clear();
    int promoted = 0;
    std::vector<CoverageTarget> promoted_identities;
    promoted_identities.reserve(coverage_executable_candidate_max_count_);
    auto alreadyPromoted = [&](const CoverageTarget &target) {
      return std::any_of(
          promoted_identities.begin(), promoted_identities.end(),
          [&](const CoverageTarget &accepted) {
            return sameCoverageExecutionTarget(
                accepted, target, coverage_recovery_match_radius_);
          });
    };
    if (admission.retain_active) {
      CoverageTarget held=active_coverage_target_;
      held.approach_candidates={held.approach_position};
      if (!coverageRecoveryExhausted(held) && selectSafeCoverageApproach(held,true) &&
          appendCoverageViewpoint(held)) {
        ++promoted;
        promoted_identities.push_back(held);
      }
    }
    for (CoverageTarget target : coverage_targets) {
      if (promoted >= coverage_executable_candidate_max_count_) {
        break;
      }
      if (!isFloorPhaseTarget(target)) {
        continue;
      }
      if (coverageRecoveryExhausted(target)) {
        rememberCoverageRecoveryAlias(target);
        continue;
      }
      const bool is_active =
          has_active_coverage_goal_ && target.stable_id != 0 &&
          target.stable_id == active_coverage_target_.stable_id;
      if (!is_active && coverageRecoveryDeferred(target, coverage_now)) {
        rememberCoverageRecoveryAlias(target);
        continue;
      }
      if (alreadyPromoted(target)) {
        continue;
      }
      if (appendCoverageViewpoint(target)) {
        ++promoted;
        promoted_identities.emplace_back(target);
      }
    }

    // Preserve the normal 45 s room/floor rotation while another target is
    // executable. Once both the frontend and normal coverage pool are empty,
    // however, waiting for the coverage plateau before shortening cooldown is
    // dead time: the current zero-terminal trajectory can end long before a
    // retry is exposed. The retry remains bounded by the per-target attempt
    // counters and terminal_retry_interval, so it cannot spin forever.
    int terminal_retry_promoted = 0;
    int terminal_eligible_promoted = 0;
    int cooling_pending = 0;
    double next_terminal_retry =
        std::numeric_limits<double>::infinity();
    if (admission.admit_new && promoted == 0 && no_executable_frontier) {
      auto terminal_targets =
          coverage_guidance_->unknownApproachTargets(pos, 160, 0.0);
      std::stable_sort(
          terminal_targets.begin(), terminal_targets.end(),
          [&](const CoverageTarget &first, const CoverageTarget &second) {
            return localExecutionScore(first) < localExecutionScore(second);
          });
      for (CoverageTarget target : terminal_targets) {
        if (promoted >= coverage_executable_candidate_max_count_) {
          break;
        }
        // The terminal set includes low-gain and cooling targets omitted from
        // the preferred pool, so it must repeat the same canonicalization.
        // Terminal failures are recorded here because there is no remaining
        // normal executable action to make progress on them later.
        if (coverageRecoveryExhausted(target)) {
          continue;
        }
        if (!selectSafeCoverageApproach(target, true)) {
          continue;
        }
        if (coverageRecoveryExhausted(target)) {
          rememberCoverageRecoveryAlias(target);
          continue;
        }
        if (!isFloorPhaseTarget(target)) {
          continue;
        }
        if (alreadyPromoted(target)) {
          continue;
        }
        const bool is_active =
            has_active_coverage_goal_ && target.stable_id != 0 &&
            target.stable_id == active_coverage_target_.stable_id;
        if (is_active ||
            !coverageRecoveryDeferred(target, coverage_now)) {
          if (appendCoverageViewpoint(target)) {
            ++promoted;
            ++terminal_eligible_promoted;
            promoted_identities.emplace_back(target);
          }
          continue;
        }
        rememberCoverageRecoveryAlias(target);
        double cooling_remaining = 0.0;
        if (!coverageRecoveryCooling(target, coverage_now,
                                     &cooling_remaining)) {
          continue;
        }
        ++cooling_pending;
        double retry_after = cooling_remaining;
        if (!coverage_terminal_retry_enable_ ||
            !coverageTerminalRetryReady(target, coverage_now, &retry_after)) {
          next_terminal_retry =
              std::min(next_terminal_retry, retry_after);
          continue;
        }
        if (appendCoverageViewpoint(target)) {
          ++promoted;
          ++terminal_retry_promoted;
          promoted_identities.emplace_back(target);
        }
      }
    }
    if (terminal_retry_promoted > 0) {
      ROS_WARN_STREAM_THROTTLE(
          0.5, "[coverage terminal retry] promoted="
                   << terminal_retry_promoted
                   << " after normal eligible set drained; cooling="
                   << cooling_pending
                   << " retry_interval="
                   << coverage_terminal_retry_interval_ << "s");
    } else if (terminal_eligible_promoted > 0) {
      ROS_INFO_STREAM_THROTTLE(
          0.5, "[coverage terminal drain] promoted low-gain actionable="
                   << terminal_eligible_promoted
                   << " after preferred execution pool drained");
    } else if (promoted == 0 && cooling_pending > 0) {
      ROS_INFO_STREAM_THROTTLE(
          0.5, "[coverage terminal retry] wait for bounded retry: cooling="
                   << cooling_pending << " retry_after="
                   << (std::isfinite(next_terminal_retry)
                           ? std::max(0.0, next_terminal_retry)
                           : coverage_terminal_retry_interval_)
                   << "s");
    }
    if (promoted > 0) {
      ROS_INFO_STREAM_THROTTLE(
          0.5, "[coverage candidate] promoted=" << promoted
                                                << " frontend_active="
                                                << active_clusters
                                                << " executable_frontiers="
                                                << viewpoints.size() - promoted
                                                << " speed=" << current_speed
                                                << " joint=" << jointCoverageCandidatesEnabled());
    }
  }

  return result;
}

}  // namespace fast_planner
