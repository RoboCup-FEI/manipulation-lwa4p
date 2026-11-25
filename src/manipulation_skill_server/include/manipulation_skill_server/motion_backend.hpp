// Copyright 2026 RoboCup-FEI
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "manipulation_skill_server/task.hpp"
#include "manipulation_skill_server/types.hpp"

namespace manipulation_skill_server
{

/// @brief A feasible motion found by a `MotionBackend`; the backend's subclass holds the plan.
class PlannedMotion
{
public:
  virtual ~PlannedMotion() = default;

  /// @return The id of the candidate this motion reaches; empty for a named target.
  const std::string & candidate_id() const {return candidate_id_;}

  /// @return How inconvenient the motion is: finite, lower is better, on the backend's scale.
  double motion_cost() const {return motion_cost_;}

protected:
  /// @param candidate_id The id of the candidate this motion reaches; empty for a named target.
  /// @param motion_cost How inconvenient the motion is, lower is better.
  PlannedMotion(std::string candidate_id, double motion_cost)
  : candidate_id_(std::move(candidate_id)), motion_cost_(motion_cost)
  {
  }

private:
  std::string candidate_id_;
  double motion_cost_;
};

/// @brief The feasible motions a backend found, or why it found none.
struct PlanResult
{
  /// Non-null, at most one per candidate, in any order; empty when nothing is reachable.
  std::vector<std::unique_ptr<PlannedMotion>> plans;
  /// Why there is no plan, for people and logs; may be set when there are plans, too.
  std::string message;
};

/// @brief How the execution of a planned motion ended.
enum class ExecutionStatus
{
  Succeeded,  ///< The arm reached the target and is at rest.
  Canceled,   ///< The caller cancelled; the arm stopped on the way and is at rest.
  Failed,     ///< The arm or its controller failed; the arm stopped on the way and is at rest.
};

/// @brief The end of an execution with a readable cause.
struct ExecutionResult
{
  ExecutionStatus status{ExecutionStatus::Failed};
  std::string message;
};

/// @brief Plans and executes arm motions; implemented by the MoveIt / MTC adapter.
class MotionBackend
{
public:
  virtual ~MotionBackend() = default;

  /// @brief Plans from the current state to every candidate within one shared time budget.
  ///
  /// Returns early with what it has when `cancellation` asks to stop.
  /// @param candidates Validated candidates: unique ids, frames set, unit quaternions.
  /// @param profile The validated motion profile.
  /// @param budget The total planning time for all candidates.
  /// @param cancellation Polled during planning.
  /// @return Every feasible motion found.
  virtual PlanResult plan_to_candidates(
    const std::vector<TcpCandidate> & candidates, MotionProfile profile,
    std::chrono::duration<double> budget, const CancellationToken & cancellation) = 0;

  /// @brief Plans a joint-space motion from the current state to joint positions.
  /// @param target Joint positions by joint name.
  /// @param budget The planning time.
  /// @param cancellation Polled during planning.
  /// @return The motion with an empty candidate id, or no plan.
  virtual PlanResult plan_to_joints(
    const JointTarget & target, std::chrono::duration<double> budget,
    const CancellationToken & cancellation) = 0;

  /// @brief Executes a motion this backend planned and waits until the arm is at rest.
  ///
  /// Stops the arm when `cancellation` asks to.
  /// @param motion A motion returned by this backend.
  /// @param cancellation Polled during execution.
  /// @return How the execution ended.
  /// @throws std::runtime_error if the arm is not known to be at rest.
  virtual ExecutionResult execute(
    const PlannedMotion & motion, const CancellationToken & cancellation) = 0;
};

}  // namespace manipulation_skill_server
