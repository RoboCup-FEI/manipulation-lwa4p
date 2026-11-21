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
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "manipulation_skill_server/motion_backend.hpp"
#include "manipulation_skill_server/named_pose_table.hpp"
#include "manipulation_skill_server/task.hpp"
#include "manipulation_skill_server/types.hpp"

namespace manipulation_skill_server
{

/// @brief The goal of `GoToPosition`, as received; `GoToPositionTask` validates it.
struct GoToPositionRequest
{
  /// Exactly one of `candidates` and `named_target` must be set.
  std::vector<TcpCandidate> candidates;
  std::string named_target;
  MotionProfile profile{MotionProfile::Ptp};
};

/// @brief The result of `GoToPosition`.
struct GoToPositionResult
{
  Outcome outcome;
  /// The candidate the arm moved to; empty for a named target or when no motion started.
  std::string selected_candidate_id;
};

/// @brief Weights of the plan ranking; both finite and not negative.
///
/// A placeholder until the team settles the formula (consolidation D02): the plan with the
/// lowest `motion_cost * motion_cost_of_plan - quality * quality_of_candidate` wins.
/// A candidate without a quality estimate gets no quality term, as if its quality were 0.
/// Only the ratio of the weights changes which plan wins. The defaults weigh both terms
/// equally; the ROS layer will use them as parameter defaults.
struct SelectionWeights
{
  double motion_cost{1.0};
  double quality{1.0};
};

/// @brief Server configuration of `GoToPosition`.
class GoToPositionSettings
{
public:
  /// @param planning_budget The total planning time of one goal.
  /// @param selection_weights The weights of the plan ranking.
  /// @throws std::invalid_argument if the budget is not positive and finite, or a weight is
  ///   negative or not finite.
  GoToPositionSettings(
    std::chrono::duration<double> planning_budget, SelectionWeights selection_weights);

  /// @return The total planning time of one goal.
  std::chrono::duration<double> planning_budget() const;

  /// @return The weights of the plan ranking.
  const SelectionWeights & selection_weights() const;

private:
  std::chrono::duration<double> planning_budget_;
  SelectionWeights selection_weights_;
};

/// @brief Moves the TCP to the best reachable candidate, or the arm to a named pose.
///
/// Order: validate the goal, plan, choose the plan ranked best by `SelectionWeights`,
/// check for cancellation, execute. A goal that breaks the rules of `GoToPosition.action`
/// ends with `INVALID_GOAL` before the backend is called.
class GoToPositionTask : public Task<GoToPositionResult>
{
public:
  /// @param context The link to the caller.
  /// @param request The goal as received.
  /// @param motion The motion backend; must outlive the task.
  /// @param named_poses The configured named poses; must outlive the task.
  /// @param settings The configuration of this action.
  GoToPositionTask(
    std::unique_ptr<TaskContext<GoToPositionResult>> context, GoToPositionRequest request,
    MotionBackend & motion, const NamedPoseTable & named_poses,
    const GoToPositionSettings & settings);

protected:
  /// @throws std::logic_error if the backend returns a plan it was not asked for or with a
  ///   non-finite cost.
  /// @throws Whatever the backend throws.
  GoToPositionResult run() override;

private:
  std::optional<std::string> find_goal_error() const;
  std::optional<std::string> find_candidate_error(std::size_t index) const;
  GoToPositionResult move_along_best(PlanResult planned);
  const PlannedMotion & choose_plan(
    const std::vector<std::unique_ptr<PlannedMotion>> & plans) const;
  std::optional<double> quality_of(const std::string & candidate_id) const;

  GoToPositionRequest request_;
  MotionBackend & motion_;
  const NamedPoseTable & named_poses_;
  GoToPositionSettings settings_;
};

}  // namespace manipulation_skill_server
