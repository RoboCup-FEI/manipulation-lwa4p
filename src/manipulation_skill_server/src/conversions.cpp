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

#include "manipulation_skill_server/conversions.hpp"

#include <cmath>
#include <cstdint>
#include <optional>

#include "rclcpp/time.hpp"

namespace manipulation_skill_server
{

namespace
{

using GoToPosition = manipulation_interfaces::action::GoToPosition;
using OutcomeMsg = manipulation_interfaces::msg::Outcome;

// to_request() casts the profile number, so the core values must equal the action constants.
static_assert(static_cast<std::uint8_t>(MotionProfile::Ptp) == GoToPosition::Goal::PTP);
static_assert(static_cast<std::uint8_t>(MotionProfile::Lin) == GoToPosition::Goal::LIN);

TcpCandidate to_candidate(const manipulation_interfaces::msg::TcpCandidate & msg)
{
  TcpCandidate candidate;
  candidate.id = msg.id;
  candidate.tcp_pose.frame_id = msg.tcp_pose.header.frame_id;
  candidate.tcp_pose.stamp_ns = rclcpp::Time(msg.tcp_pose.header.stamp).nanoseconds();
  const auto & position = msg.tcp_pose.pose.position;
  candidate.tcp_pose.pose.position_m = {position.x, position.y, position.z};
  const auto & orientation = msg.tcp_pose.pose.orientation;
  candidate.tcp_pose.pose.orientation = {
    orientation.x, orientation.y, orientation.z, orientation.w};
  if (!std::isnan(msg.quality)) {
    candidate.quality = msg.quality;
  }
  return candidate;
}

std::uint8_t to_msg_code(OutcomeCode code)
{
  switch (code) {
    case OutcomeCode::Success:
      return OutcomeMsg::SUCCESS;
    case OutcomeCode::Canceled:
      return OutcomeMsg::CANCELED;
    case OutcomeCode::InvalidGoal:
      return OutcomeMsg::INVALID_GOAL;
    case OutcomeCode::Busy:
      return OutcomeMsg::BUSY;
    case OutcomeCode::PlanningFailed:
      return OutcomeMsg::PLANNING_FAILED;
    case OutcomeCode::GripFailed:
      return OutcomeMsg::GRIP_FAILED;
    case OutcomeCode::ExecutionFailed:
      return OutcomeMsg::EXECUTION_FAILED;
    case OutcomeCode::InternalError:
      return OutcomeMsg::INTERNAL_ERROR;
  }
  return OutcomeMsg::INTERNAL_ERROR;
}

}  // namespace

GoToPositionRequest to_request(const GoToPosition::Goal & goal)
{
  GoToPositionRequest request;
  request.candidates.reserve(goal.candidates.size());
  for (const auto & candidate : goal.candidates) {
    request.candidates.push_back(to_candidate(candidate));
  }
  request.named_target = goal.named_target;
  request.profile = static_cast<MotionProfile>(goal.profile);
  return request;
}

GoToPosition::Result to_ros_result(const GoToPositionResult & result)
{
  GoToPosition::Result msg;
  msg.outcome = to_msg(result.outcome);
  msg.selected_candidate_id = result.selected_candidate_id;
  return msg;
}

OutcomeMsg to_msg(const Outcome & outcome)
{
  OutcomeMsg msg;
  msg.code = to_msg_code(outcome.code);
  msg.message = outcome.message;
  return msg;
}

const char * to_string(OutcomeCode code)
{
  switch (code) {
    case OutcomeCode::Success:
      return "SUCCESS";
    case OutcomeCode::Canceled:
      return "CANCELED";
    case OutcomeCode::InvalidGoal:
      return "INVALID_GOAL";
    case OutcomeCode::Busy:
      return "BUSY";
    case OutcomeCode::PlanningFailed:
      return "PLANNING_FAILED";
    case OutcomeCode::GripFailed:
      return "GRIP_FAILED";
    case OutcomeCode::ExecutionFailed:
      return "EXECUTION_FAILED";
    case OutcomeCode::InternalError:
      return "INTERNAL_ERROR";
  }
  return "INTERNAL_ERROR";
}

}  // namespace manipulation_skill_server
