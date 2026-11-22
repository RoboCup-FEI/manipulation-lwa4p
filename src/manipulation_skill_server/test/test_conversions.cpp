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

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "manipulation_skill_server/conversions.hpp"

namespace manipulation_skill_server::test
{
namespace
{

using GoToPosition = manipulation_interfaces::action::GoToPosition;
using OutcomeMsg = manipulation_interfaces::msg::Outcome;

manipulation_interfaces::msg::TcpCandidate make_candidate_msg(
  const std::string & id, double quality)
{
  manipulation_interfaces::msg::TcpCandidate msg;
  msg.id = id;
  msg.tcp_pose.header.frame_id = "camera_color_optical_frame";
  msg.tcp_pose.header.stamp.sec = 2;
  msg.tcp_pose.header.stamp.nanosec = 5;
  msg.tcp_pose.pose.position.x = 0.1;
  msg.tcp_pose.pose.position.y = 0.2;
  msg.tcp_pose.pose.position.z = 0.3;
  msg.tcp_pose.pose.orientation.x = 0.0;
  msg.tcp_pose.pose.orientation.y = 1.0;
  msg.tcp_pose.pose.orientation.z = 0.0;
  msg.tcp_pose.pose.orientation.w = 0.0;
  msg.quality = quality;
  return msg;
}

TEST(Conversions, CandidateGoalKeepsEveryField)
{
  GoToPosition::Goal goal;
  goal.candidates.push_back(make_candidate_msg("a", 0.75));
  goal.profile = GoToPosition::Goal::LIN;

  const GoToPositionRequest request = to_request(goal);

  ASSERT_EQ(request.candidates.size(), 1u);
  const TcpCandidate & candidate = request.candidates[0];
  EXPECT_EQ(candidate.id, "a");
  EXPECT_EQ(candidate.tcp_pose.frame_id, "camera_color_optical_frame");
  EXPECT_EQ(candidate.tcp_pose.stamp_ns, 2'000'000'005);
  EXPECT_DOUBLE_EQ(candidate.tcp_pose.pose.position_m.z, 0.3);
  EXPECT_DOUBLE_EQ(candidate.tcp_pose.pose.orientation.y, 1.0);
  EXPECT_DOUBLE_EQ(candidate.tcp_pose.pose.orientation.w, 0.0);
  ASSERT_TRUE(candidate.quality.has_value());
  EXPECT_DOUBLE_EQ(*candidate.quality, 0.75);
  EXPECT_TRUE(request.named_target.empty());
  EXPECT_EQ(request.profile, MotionProfile::Lin);
}

TEST(Conversions, NanQualityMeansNoEstimate)
{
  GoToPosition::Goal goal;
  goal.candidates.push_back(make_candidate_msg("a", std::numeric_limits<double>::quiet_NaN()));

  EXPECT_FALSE(to_request(goal).candidates[0].quality.has_value());
}

TEST(Conversions, NegativeStampIsKeptForValidation)
{
  GoToPosition::Goal goal;
  goal.candidates.push_back(make_candidate_msg("a", 0.5));
  goal.candidates[0].tcp_pose.header.stamp.sec = -1;
  goal.candidates[0].tcp_pose.header.stamp.nanosec = 0;

  EXPECT_EQ(to_request(goal).candidates[0].tcp_pose.stamp_ns, -1'000'000'000);
}

TEST(Conversions, NamedTargetAndUnknownProfileAreKeptForValidation)
{
  GoToPosition::Goal goal;
  goal.named_target = "home";
  goal.profile = 7;

  const GoToPositionRequest request = to_request(goal);

  EXPECT_EQ(request.named_target, "home");
  EXPECT_EQ(static_cast<int>(request.profile), 7);
}

TEST(Conversions, EveryOutcomeCodeMapsToItsMessageConstant)
{
  const std::vector<std::pair<OutcomeCode, std::uint8_t>> expected = {
    {OutcomeCode::Success, OutcomeMsg::SUCCESS},
    {OutcomeCode::Canceled, OutcomeMsg::CANCELED},
    {OutcomeCode::InvalidGoal, OutcomeMsg::INVALID_GOAL},
    {OutcomeCode::Busy, OutcomeMsg::BUSY},
    {OutcomeCode::PlanningFailed, OutcomeMsg::PLANNING_FAILED},
    {OutcomeCode::GripFailed, OutcomeMsg::GRIP_FAILED},
    {OutcomeCode::ExecutionFailed, OutcomeMsg::EXECUTION_FAILED},
    {OutcomeCode::InternalError, OutcomeMsg::INTERNAL_ERROR},
  };
  for (const auto & [code, msg_code] : expected) {
    EXPECT_EQ(to_msg({code, "why"}).code, msg_code) << to_string(code);
    EXPECT_EQ(to_msg({code, "why"}).message, "why");
  }
  EXPECT_STREQ(to_string(OutcomeCode::PlanningFailed), "PLANNING_FAILED");
}

TEST(Conversions, ResultKeepsSelectedCandidate)
{
  GoToPositionResult result;
  result.outcome = {OutcomeCode::ExecutionFailed, "controller aborted"};
  result.selected_candidate_id = "b";

  const GoToPosition::Result msg = to_ros_result(result);

  EXPECT_EQ(msg.outcome.code, OutcomeMsg::EXECUTION_FAILED);
  EXPECT_EQ(msg.outcome.message, "controller aborted");
  EXPECT_EQ(msg.selected_candidate_id, "b");
}

}  // namespace
}  // namespace manipulation_skill_server::test
