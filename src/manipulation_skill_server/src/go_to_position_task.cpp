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

#include "manipulation_skill_server/go_to_position_task.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace manipulation_skill_server
{

namespace
{

// Accepts quaternions computed in single precision; the task normalizes them afterwards.
constexpr double UNIT_QUATERNION_TOLERANCE = 1e-3;

double norm(const Quaternion & q)
{
  return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

bool is_finite(const Vector3 & v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool is_unit(const Quaternion & q)
{
  const double length = norm(q);
  return std::isfinite(length) && std::abs(length - 1.0) <= UNIT_QUATERNION_TOLERANCE;
}

void normalize(Quaternion & q)
{
  const double length = norm(q);
  q.x /= length;
  q.y /= length;
  q.z /= length;
  q.w /= length;
}

bool is_known(MotionProfile profile)
{
  return profile == MotionProfile::Ptp || profile == MotionProfile::Lin;
}

// Lower is better; see SelectionWeights.
double combined_cost(
  double motion_cost, const std::optional<double> & quality, const SelectionWeights & weights)
{
  return weights.motion_cost * motion_cost - weights.quality * quality.value_or(0.0);
}

GoToPositionResult make_result(OutcomeCode code, std::string message)
{
  GoToPositionResult result;
  result.outcome = {code, std::move(message)};
  return result;
}

}  // namespace

GoToPositionSettings::GoToPositionSettings(
  std::chrono::duration<double> planning_budget, SelectionWeights selection_weights)
: planning_budget_(planning_budget), selection_weights_(selection_weights)
{
  if (!std::isfinite(planning_budget_.count()) || planning_budget_.count() <= 0.0) {
    throw std::invalid_argument("the planning budget must be positive and finite");
  }
  const bool motion_weight_valid =
    std::isfinite(selection_weights_.motion_cost) && selection_weights_.motion_cost >= 0.0;
  const bool quality_weight_valid =
    std::isfinite(selection_weights_.quality) && selection_weights_.quality >= 0.0;
  if (!motion_weight_valid || !quality_weight_valid) {
    throw std::invalid_argument("the selection weights must be finite and not negative");
  }
}

std::chrono::duration<double> GoToPositionSettings::planning_budget() const
{
  return planning_budget_;
}

const SelectionWeights & GoToPositionSettings::selection_weights() const
{
  return selection_weights_;
}

GoToPositionTask::GoToPositionTask(
  std::unique_ptr<TaskContext<GoToPositionResult>> context, GoToPositionRequest request,
  MotionBackend & motion, const NamedPoseTable & named_poses, const GoToPositionSettings & settings)
: Task<GoToPositionResult>(std::move(context)),
  request_(std::move(request)),
  motion_(motion),
  named_poses_(named_poses),
  settings_(settings)
{
}

GoToPositionResult GoToPositionTask::run()
{
  if (const auto error = find_goal_error()) {
    return make_result(OutcomeCode::InvalidGoal, *error);
  }

  if (!request_.named_target.empty()) {
    const JointTarget & target = *named_poses_.find(request_.named_target);
    return move_along_best(
      motion_.plan_to_joints(target, settings_.planning_budget(), cancellation()));
  }

  for (TcpCandidate & candidate : request_.candidates) {
    normalize(candidate.tcp_pose.pose.orientation);
  }
  return move_along_best(motion_.plan_to_candidates(
    request_.candidates, request_.profile, settings_.planning_budget(), cancellation()));
}

std::optional<std::string> GoToPositionTask::find_goal_error() const
{
  const bool has_candidates = !request_.candidates.empty();
  const bool has_named_target = !request_.named_target.empty();
  if (has_candidates == has_named_target) {
    return "the goal must hold exactly one of candidates and named_target";
  }
  if (!is_known(request_.profile)) {
    return "unknown profile " + std::to_string(static_cast<int>(request_.profile));
  }

  if (has_named_target) {
    if (request_.profile != MotionProfile::Ptp) {
      return "a named target allows only the PTP profile";
    }
    if (named_poses_.find(request_.named_target) == nullptr) {
      return "unknown named target '" + request_.named_target + "'";
    }
    return std::nullopt;
  }

  std::set<std::string> ids;
  for (std::size_t index = 0; index < request_.candidates.size(); ++index) {
    if (const auto error = find_candidate_error(index)) {
      return error;
    }
    if (!ids.insert(request_.candidates[index].id).second) {
      return "candidate id '" + request_.candidates[index].id + "' is not unique";
    }
  }
  return std::nullopt;
}

std::optional<std::string> GoToPositionTask::find_candidate_error(std::size_t index) const
{
  const TcpCandidate & candidate = request_.candidates[index];
  if (candidate.id.empty()) {
    return "candidate " + std::to_string(index) + " has an empty id";
  }
  const std::string name = "candidate '" + candidate.id + "'";
  if (candidate.tcp_pose.frame_id.empty()) {
    return name + " has an empty frame_id";
  }
  if (!is_finite(candidate.tcp_pose.pose.position_m)) {
    return name + " has a non-finite position";
  }
  if (!is_unit(candidate.tcp_pose.pose.orientation)) {
    return name + " has an orientation that is not a unit quaternion";
  }
  if (candidate.quality && !std::isfinite(*candidate.quality)) {
    return name + " has an infinite quality; use NaN for no estimate";
  }
  return std::nullopt;
}

GoToPositionResult GoToPositionTask::move_along_best(PlanResult planned)
{
  // Checked first: a backend that stopped planning on cancellation may return no plan.
  if (cancellation().cancel_requested()) {
    return make_result(OutcomeCode::Canceled, "canceled before the motion started");
  }
  if (planned.plans.empty()) {
    if (planned.message.empty()) {
      planned.message = "no feasible plan";
    }
    return make_result(OutcomeCode::PlanningFailed, planned.message);
  }

  const PlannedMotion & plan = choose_plan(planned.plans);
  const ExecutionResult executed = motion_.execute(plan, cancellation());

  GoToPositionResult result;
  if (!request_.candidates.empty()) {
    result.selected_candidate_id = plan.candidate_id();
  }
  switch (executed.status) {
    case ExecutionStatus::Succeeded:
      result.outcome = {OutcomeCode::Success, executed.message};
      break;
    case ExecutionStatus::Canceled:
      result.outcome = {OutcomeCode::Canceled, executed.message};
      break;
    case ExecutionStatus::Failed:
      result.outcome = {OutcomeCode::ExecutionFailed, executed.message};
      break;
  }
  return result;
}

const PlannedMotion & GoToPositionTask::choose_plan(
  const std::vector<std::unique_ptr<PlannedMotion>> & plans) const
{
  const PlannedMotion * best = nullptr;
  double best_cost = 0.0;
  for (const auto & plan : plans) {
    if (!std::isfinite(plan->motion_cost())) {
      throw std::logic_error(
        "the motion backend returned a non-finite cost for candidate '" + plan->candidate_id() +
        "'");
    }
    const double cost = combined_cost(
      plan->motion_cost(), quality_of(plan->candidate_id()), settings_.selection_weights());
    // Strict comparison: of equal costs, the plan the backend listed first wins.
    if (best == nullptr || cost < best_cost) {
      best = plan.get();
      best_cost = cost;
    }
  }
  return *best;
}

std::optional<double> GoToPositionTask::quality_of(const std::string & candidate_id) const
{
  if (request_.candidates.empty()) {
    return std::nullopt;
  }
  for (const TcpCandidate & candidate : request_.candidates) {
    if (candidate.id == candidate_id) {
      return candidate.quality;
    }
  }
  throw std::logic_error(
    "the motion backend returned a plan for unknown candidate '" + candidate_id + "'");
}

}  // namespace manipulation_skill_server
