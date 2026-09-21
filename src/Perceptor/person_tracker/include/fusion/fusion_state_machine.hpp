#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>

#include <Eigen/Core>

namespace person_tracker
{

enum class FusionState : std::uint8_t
{
  WAITING = 0,
  LOCKED = 1,
  SUSPECT = 2,
  LOST = 3
};

enum class SemanticEvidence : std::uint8_t
{
  UNKNOWN = 0,
  OUT_OF_VIEW = 1,
  CONSISTENT = 2,
  CONFLICT = 3
};

struct FusionStateParameters
{
  int conflict_frames{3};
  int confirmation_frames{2};
  double confirmation_radius{0.45};
};

// This state machine deliberately does not own the lidar tracker.  LOCKED and
// SUSPECT both leave the FAPP/EKF path alive; the only extra authority granted
// by SUSPECT is a request to select a repeatedly confirmed YOLO-supported
// lidar component.
class FusionStateMachine
{
public:
  explicit FusionStateMachine(const FusionStateParameters & parameters = {})
  : parameters_(parameters)
  {
    parameters_.conflict_frames = std::max(1, parameters_.conflict_frames);
    parameters_.confirmation_frames = std::max(1, parameters_.confirmation_frames);
    parameters_.confirmation_radius = std::max(0.0, parameters_.confirmation_radius);
  }

  void reset()
  {
    state_ = FusionState::WAITING;
    evidence_ = SemanticEvidence::UNKNOWN;
    resetCounters();
  }

  void lockTarget()
  {
    state_ = FusionState::LOCKED;
    evidence_ = SemanticEvidence::UNKNOWN;
    resetCounters();
  }

  void loseTarget()
  {
    state_ = FusionState::LOST;
    evidence_ = SemanticEvidence::UNKNOWN;
    resetCounters();
  }

  void observe(
    SemanticEvidence evidence,
    const std::optional<Eigen::Vector2d> & semantic_candidate = std::nullopt)
  {
    evidence_ = evidence;
    if (state_ == FusionState::WAITING || state_ == FusionState::LOST) {
      return;
    }

    if (evidence == SemanticEvidence::UNKNOWN ||
      evidence == SemanticEvidence::OUT_OF_VIEW)
    {
      // Both mean "YOLO has no vote".  Consecutive evidence must start over,
      // but a SUSPECT audit is retained until positive evidence resolves it.
      conflict_count_ = 0;
      confirmation_count_ = 0;
      confirmation_candidate_.reset();
      reselection_ready_ = false;
      return;
    }

    if (state_ == FusionState::LOCKED) {
      confirmation_count_ = 0;
      confirmation_candidate_.reset();
      reselection_ready_ = false;
      if (evidence == SemanticEvidence::CONSISTENT) {
        conflict_count_ = 0;
        return;
      }
      ++conflict_count_;
      if (conflict_count_ >= parameters_.conflict_frames) {
        state_ = FusionState::SUSPECT;
        confirmation_count_ = 0;
        confirmation_candidate_.reset();
      }
      return;
    }

    // SUSPECT: repeated agreement with the existing EKF clears suspicion.
    // Repeated conflict at a spatially stable new component arms reselection.
    if (evidence == SemanticEvidence::CONSISTENT) {
      reselection_ready_ = false;
      confirmation_candidate_.reset();
      ++confirmation_count_;
      if (confirmation_count_ >= parameters_.confirmation_frames) {
        state_ = FusionState::LOCKED;
        resetCounters();
      }
      return;
    }

    if (!semantic_candidate) {
      confirmation_count_ = 0;
      confirmation_candidate_.reset();
      reselection_ready_ = false;
      return;
    }
    const bool same_candidate = confirmation_candidate_ &&
      (*semantic_candidate - *confirmation_candidate_).norm() <=
      parameters_.confirmation_radius;
    confirmation_count_ = same_candidate ? confirmation_count_ + 1 : 1;
    confirmation_candidate_ = semantic_candidate;
    reselection_ready_ =
      confirmation_count_ >= parameters_.confirmation_frames;
  }

  void recordEvidence(SemanticEvidence evidence) {evidence_ = evidence;}

  void acceptReselection()
  {
    state_ = FusionState::LOCKED;
    evidence_ = SemanticEvidence::CONSISTENT;
    resetCounters();
  }

  FusionState state() const {return state_;}
  SemanticEvidence evidence() const {return evidence_;}
  int conflictCount() const {return conflict_count_;}
  int confirmationCount() const {return confirmation_count_;}
  bool reselectionReady() const {return reselection_ready_;}

private:
  void resetCounters()
  {
    conflict_count_ = 0;
    confirmation_count_ = 0;
    confirmation_candidate_.reset();
    reselection_ready_ = false;
  }

  FusionStateParameters parameters_;
  FusionState state_{FusionState::WAITING};
  SemanticEvidence evidence_{SemanticEvidence::UNKNOWN};
  int conflict_count_{0};
  int confirmation_count_{0};
  std::optional<Eigen::Vector2d> confirmation_candidate_;
  bool reselection_ready_{false};
};

inline const char * fusionStateName(FusionState state)
{
  switch (state) {
    case FusionState::WAITING: return "WAITING";
    case FusionState::LOCKED: return "LOCKED";
    case FusionState::SUSPECT: return "SUSPECT";
    case FusionState::LOST: return "LOST";
  }
  return "UNKNOWN";
}

inline const char * semanticEvidenceName(SemanticEvidence evidence)
{
  switch (evidence) {
    case SemanticEvidence::UNKNOWN: return "UNKNOWN";
    case SemanticEvidence::OUT_OF_VIEW: return "OUT_OF_VIEW";
    case SemanticEvidence::CONSISTENT: return "CONSISTENT";
    case SemanticEvidence::CONFLICT: return "CONFLICT";
  }
  return "UNKNOWN";
}

}  // namespace person_tracker
