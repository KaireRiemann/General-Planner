#pragma once

#include <general_core/planner_runtime/planner_status.hpp>

namespace general_planner::planner_runtime {

// Keep source-selection policy independent from ROS plumbing so the safety
// behavior can be regression-tested without a running ROS master.
enum class GatewayOutputMode {
  NAVIGATION,
  EXPLORATION,
  EXPLICIT_HOLD,
  SOURCE_TIMEOUT_HOLD,
  // Do not turn an external-control handover into a HOLD command.  This is a
  // second, policy-level guard in addition to publishing_enabled=false.
  EXTERNAL_GATE_SUPPRESSED,
};

constexpr GatewayOutputMode selectGatewayOutputMode(
    const CommandOwner authorized_owner, const bool navigation_fresh,
    const bool exploration_fresh) {
  if (authorized_owner == CommandOwner::STATE2STATE) {
    return navigation_fresh ? GatewayOutputMode::NAVIGATION
                            : GatewayOutputMode::SOURCE_TIMEOUT_HOLD;
  }
  if (authorized_owner == CommandOwner::EXPLORATION) {
    return exploration_fresh ? GatewayOutputMode::EXPLORATION
                             : GatewayOutputMode::SOURCE_TIMEOUT_HOLD;
  }
  if (authorized_owner == CommandOwner::GATE) {
    return GatewayOutputMode::EXTERNAL_GATE_SUPPRESSED;
  }
  return GatewayOutputMode::EXPLICIT_HOLD;
}

/**
 * Decide whether the supervisor must retire a task after the command gateway
 * has already entered its safe current-pose fallback. A source has two
 * distinct failure cases: it never emitted a first command after ownership
 * was authorized, or it emitted commands and then went stale.
 *
 * `source_age_seconds` deliberately has the value infinity before the first
 * command. It must therefore never be used to judge the first case: doing so
 * turns the normal authorization-to-first-command gap into an immediate task
 * failure. The first case is bounded instead by authorization age.
 */
constexpr bool shouldAbortCommandSource(
    const bool source_expected,
    const bool source_received_since_authorization,
    const bool source_fresh,
    const double source_age_seconds,
    const double authorization_age_seconds,
    const double first_command_grace_seconds,
    const double stale_source_abort_seconds) {
  if (!source_expected) {
    return false;
  }
  if (!source_received_since_authorization) {
    return authorization_age_seconds >= first_command_grace_seconds;
  }
  return !source_fresh && source_age_seconds >= stale_source_abort_seconds;
}

} // namespace general_planner::planner_runtime
