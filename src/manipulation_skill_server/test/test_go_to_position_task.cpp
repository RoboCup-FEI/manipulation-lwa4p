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

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "fakes.hpp"
#include "manipulation_skill_server/go_to_position_task.hpp"

namespace manipulation_skill_server::test
{
namespace
{

using Record = ResultRecord<GoToPositionResult>;

constexpr double NAN_VALUE = std::numeric_limits<double>::quiet_NaN();
constexpr double INF_VALUE = std::numeric_limits<double>::infinity();

TcpCandidate make_candidate(const std::string & id, std::optional<double> quality = std::nullopt)
{
  TcpCandidate candidate;
  candidate.id = id;
  candidate.tcp_pose.frame_id = "arm_base_link";
  candidate.tcp_pose.pose.position_m = {0.3, 0.0, 0.2};
  candidate.quality = quality;
  return candidate;
}

class GoToPositionTaskTest : public ::testing::Test
{
protected:
  GoToPositionTaskTest()
  : named_poses_({{"home", {{"arm_joint_1", 0.0}, {"arm_joint_2", 0.5}}}}),
    settings_(std::chrono::duration<double>(2.0), SelectionWeights{1.0, 1.0})
  {
  }

  /// Runs a task for the request as the runner does and returns its only result.
  GoToPositionResult run(GoToPositionRequest request)
  {
    GoToPositionTask task(
      std::make_unique<FakeTaskContext<GoToPositionResult>>(record_), std::move(request), backend_,
      named_poses_, settings_);
    task.execute();
    task.finish();
    const auto results = record_->results();
    EXPECT_EQ(results.size(), 1u);
    return results.empty() ? GoToPositionResult{} : results.back();
  }

  GoToPositionRequest candidates_request(std::vector<TcpCandidate> candidates)
  {
    GoToPositionRequest request;
    request.candidates = std::move(candidates);
    return request;
  }

  /// Expects INVALID_GOAL whose message contains `reason`, with no backend call.
  void expect_invalid(GoToPositionRequest request, const std::string & reason)
  {
    const GoToPositionResult result = run(std::move(request));
    EXPECT_EQ(result.outcome.code, OutcomeCode::InvalidGoal);
    EXPECT_NE(result.outcome.message.find(reason), std::string::npos) << result.outcome.message;
    EXPECT_TRUE(result.selected_candidate_id.empty());
    EXPECT_EQ(backend_.plan_calls, 0);
    EXPECT_TRUE(backend_.executed_ids.empty());
  }

  std::shared_ptr<Record> record_ = std::make_shared<Record>();
  FakeMotionBackend backend_;
  NamedPoseTable named_poses_;
  GoToPositionSettings settings_;
};

// Goal validation.

TEST_F(GoToPositionTaskTest, RejectsGoalWithNeitherCandidatesNorNamedTarget)
{
  expect_invalid(GoToPositionRequest{}, "exactly one");
}

TEST_F(GoToPositionTaskTest, RejectsGoalWithBothCandidatesAndNamedTarget)
{
  GoToPositionRequest request = candidates_request({make_candidate("a")});
  request.named_target = "home";
  expect_invalid(request, "exactly one");
}

TEST_F(GoToPositionTaskTest, RejectsUnknownProfile)
{
  GoToPositionRequest request = candidates_request({make_candidate("a")});
  request.profile = static_cast<MotionProfile>(7);
  expect_invalid(request, "unknown profile 7");
}

TEST_F(GoToPositionTaskTest, RejectsNamedTargetWithLin)
{
  GoToPositionRequest request;
  request.named_target = "home";
  request.profile = MotionProfile::Lin;
  expect_invalid(request, "only the PTP profile");
}

TEST_F(GoToPositionTaskTest, RejectsUnknownNamedTarget)
{
  GoToPositionRequest request;
  request.named_target = "nowhere";
  expect_invalid(request, "unknown named target 'nowhere'");
}

TEST_F(GoToPositionTaskTest, RejectsCandidateWithEmptyId)
{
  expect_invalid(candidates_request({make_candidate("a"), make_candidate("")}), "candidate 1");
}

TEST_F(GoToPositionTaskTest, RejectsDuplicateCandidateIds)
{
  expect_invalid(
    candidates_request({make_candidate("a"), make_candidate("b"), make_candidate("a")}),
    "'a' is not unique");
}

TEST_F(GoToPositionTaskTest, RejectsCandidateWithEmptyFrame)
{
  TcpCandidate candidate = make_candidate("a");
  candidate.tcp_pose.frame_id.clear();
  expect_invalid(candidates_request({candidate}), "empty frame_id");
}

TEST_F(GoToPositionTaskTest, RejectsCandidateWithNonFinitePosition)
{
  TcpCandidate candidate = make_candidate("a");
  candidate.tcp_pose.pose.position_m.y = NAN_VALUE;
  expect_invalid(candidates_request({candidate}), "non-finite position");
}

TEST_F(GoToPositionTaskTest, RejectsZeroQuaternionPlaceholder)
{
  TcpCandidate candidate = make_candidate("a");
  candidate.tcp_pose.pose.orientation = {0.0, 0.0, 0.0, 0.0};
  expect_invalid(candidates_request({candidate}), "not a unit quaternion");
}

TEST_F(GoToPositionTaskTest, RejectsNonUnitQuaternion)
{
  TcpCandidate candidate = make_candidate("a");
  candidate.tcp_pose.pose.orientation = {0.0, 0.0, 0.0, 1.01};
  expect_invalid(candidates_request({candidate}), "not a unit quaternion");
}

TEST_F(GoToPositionTaskTest, RejectsInfiniteQuality)
{
  expect_invalid(candidates_request({make_candidate("a", INF_VALUE)}), "infinite quality");
}

TEST_F(GoToPositionTaskTest, AcceptsAlmostUnitQuaternionAndPassesItNormalized)
{
  TcpCandidate candidate = make_candidate("a");
  candidate.tcp_pose.pose.orientation = {0.0, 0.0, 0.0, 1.0005};
  backend_.candidate_costs = {{"a", 1.0}};

  const GoToPositionResult result = run(candidates_request({candidate}));

  EXPECT_EQ(result.outcome.code, OutcomeCode::Success);
  ASSERT_EQ(backend_.planned_candidates.size(), 1u);
  EXPECT_DOUBLE_EQ(backend_.planned_candidates[0].tcp_pose.pose.orientation.w, 1.0);
}

// Named target.

TEST_F(GoToPositionTaskTest, MovesToNamedTarget)
{
  GoToPositionRequest request;
  request.named_target = "home";

  const GoToPositionResult result = run(request);

  EXPECT_EQ(result.outcome.code, OutcomeCode::Success);
  EXPECT_TRUE(result.selected_candidate_id.empty());
  const JointTarget expected{{"arm_joint_1", 0.0}, {"arm_joint_2", 0.5}};
  EXPECT_EQ(backend_.planned_joints, expected);
  EXPECT_DOUBLE_EQ(backend_.planned_budget.count(), 2.0);
  EXPECT_EQ(backend_.executed_ids.size(), 1u);
}

TEST_F(GoToPositionTaskTest, NamedTargetWithoutPlanEndsWithPlanningFailed)
{
  backend_.joint_plan_cost.reset();
  backend_.planning_message = "goal state in collision";
  GoToPositionRequest request;
  request.named_target = "home";

  const GoToPositionResult result = run(request);

  EXPECT_EQ(result.outcome.code, OutcomeCode::PlanningFailed);
  EXPECT_EQ(result.outcome.message, "goal state in collision");
  EXPECT_TRUE(backend_.executed_ids.empty());
}

// Candidates.

TEST_F(GoToPositionTaskTest, PassesCandidatesProfileAndBudgetToBackend)
{
  backend_.candidate_costs = {{"a", 1.0}};
  GoToPositionRequest request = candidates_request({make_candidate("a"), make_candidate("b")});
  request.profile = MotionProfile::Lin;

  run(request);

  ASSERT_EQ(backend_.planned_candidates.size(), 2u);
  EXPECT_EQ(backend_.planned_candidates[1].id, "b");
  EXPECT_EQ(backend_.planned_profile, MotionProfile::Lin);
  EXPECT_DOUBLE_EQ(backend_.planned_budget.count(), 2.0);
}

TEST_F(GoToPositionTaskTest, MovesToCheapestPlanNotFirstFeasible)
{
  backend_.candidate_costs = {{"a", 3.0}, {"b", 1.0}, {"c", 2.0}};

  const GoToPositionResult result =
    run(candidates_request({make_candidate("a"), make_candidate("b"), make_candidate("c")}));

  EXPECT_EQ(result.outcome.code, OutcomeCode::Success);
  EXPECT_EQ(result.selected_candidate_id, "b");
  EXPECT_EQ(backend_.executed_ids, std::vector<std::string>{"b"});
}

TEST_F(GoToPositionTaskTest, QualityCanOutweighMotionCost)
{
  // Combined costs with weights 1 and 1: a = 1 - 0 = 1, b = 1.5 - 1 = 0.5.
  backend_.candidate_costs = {{"a", 1.0}, {"b", 1.5}};

  const GoToPositionResult result =
    run(candidates_request({make_candidate("a", 0.0), make_candidate("b", 1.0)}));

  EXPECT_EQ(result.selected_candidate_id, "b");
}

TEST_F(GoToPositionTaskTest, MissingQualityIsRankedAsZero)
{
  // a has no estimate (0), b has a negative one.
  backend_.candidate_costs = {{"b", 1.0}, {"a", 1.0}};

  const GoToPositionResult result =
    run(candidates_request({make_candidate("a"), make_candidate("b", -0.5)}));

  EXPECT_EQ(result.selected_candidate_id, "a");
}

TEST_F(GoToPositionTaskTest, ZeroQualityWeightRanksByMotionCostOnly)
{
  settings_ = GoToPositionSettings(std::chrono::duration<double>(2.0), SelectionWeights{1.0, 0.0});
  backend_.candidate_costs = {{"a", 1.0}, {"b", 1.5}};

  const GoToPositionResult result =
    run(candidates_request({make_candidate("a", 0.0), make_candidate("b", 100.0)}));

  EXPECT_EQ(result.selected_candidate_id, "a");
}

TEST_F(GoToPositionTaskTest, EqualCostsKeepBackendOrder)
{
  backend_.candidate_costs = {{"c", 1.0}, {"a", 1.0}};

  const GoToPositionResult result =
    run(candidates_request({make_candidate("a"), make_candidate("c")}));

  EXPECT_EQ(result.selected_candidate_id, "c");
}

TEST_F(GoToPositionTaskTest, NoFeasibleCandidateEndsWithPlanningFailed)
{
  backend_.planning_message = "no IK solution for a, b";

  const GoToPositionResult result =
    run(candidates_request({make_candidate("a"), make_candidate("b")}));

  EXPECT_EQ(result.outcome.code, OutcomeCode::PlanningFailed);
  EXPECT_EQ(result.outcome.message, "no IK solution for a, b");
  EXPECT_TRUE(result.selected_candidate_id.empty());
  EXPECT_TRUE(backend_.executed_ids.empty());
}

TEST_F(GoToPositionTaskTest, PlanningFailedHasMessageEvenWithoutBackendReason)
{
  const GoToPositionResult result = run(candidates_request({make_candidate("a")}));

  EXPECT_EQ(result.outcome.code, OutcomeCode::PlanningFailed);
  EXPECT_FALSE(result.outcome.message.empty());
}

// Cancellation and execution.

TEST_F(GoToPositionTaskTest, CancelDuringPlanningEndsWithCanceledWithoutMotion)
{
  backend_.candidate_costs = {{"a", 1.0}};
  backend_.during_planning = [this]() {record_->request_cancel();};

  const GoToPositionResult result = run(candidates_request({make_candidate("a")}));

  EXPECT_EQ(result.outcome.code, OutcomeCode::Canceled);
  EXPECT_TRUE(result.selected_candidate_id.empty());
  EXPECT_TRUE(backend_.executed_ids.empty());
}

TEST_F(GoToPositionTaskTest, CanceledExecutionEndsWithCanceledAndReportsCandidate)
{
  backend_.candidate_costs = {{"a", 1.0}};
  backend_.execution_result = {ExecutionStatus::Canceled, "stopped on request"};

  const GoToPositionResult result = run(candidates_request({make_candidate("a")}));

  EXPECT_EQ(result.outcome.code, OutcomeCode::Canceled);
  EXPECT_EQ(result.outcome.message, "stopped on request");
  EXPECT_EQ(result.selected_candidate_id, "a");
}

TEST_F(GoToPositionTaskTest, FailedExecutionEndsWithExecutionFailedWithoutRetry)
{
  backend_.candidate_costs = {{"a", 1.0}, {"b", 2.0}};
  backend_.execution_result = {ExecutionStatus::Failed, "controller aborted"};

  const GoToPositionResult result =
    run(candidates_request({make_candidate("a"), make_candidate("b")}));

  EXPECT_EQ(result.outcome.code, OutcomeCode::ExecutionFailed);
  EXPECT_EQ(result.outcome.message, "controller aborted");
  EXPECT_EQ(result.selected_candidate_id, "a");
  EXPECT_EQ(backend_.executed_ids, std::vector<std::string>{"a"});
}

// Backend errors propagate, so the runner reports INTERNAL_ERROR and faults.

TEST_F(GoToPositionTaskTest, BackendExceptionPropagates)
{
  backend_.candidate_costs = {{"a", 1.0}};
  backend_.during_execution = []() {throw std::runtime_error("arm state unknown");};
  GoToPositionTask task(
    std::make_unique<FakeTaskContext<GoToPositionResult>>(record_),
    candidates_request({make_candidate("a")}), backend_, named_poses_, settings_);

  EXPECT_THROW(task.execute(), std::runtime_error);
  EXPECT_TRUE(record_->results().empty());
}

TEST_F(GoToPositionTaskTest, PlanForUnknownCandidateIsLogicError)
{
  backend_.candidate_costs = {{"zzz", 1.0}};
  GoToPositionTask task(
    std::make_unique<FakeTaskContext<GoToPositionResult>>(record_),
    candidates_request({make_candidate("a")}), backend_, named_poses_, settings_);

  EXPECT_THROW(task.execute(), std::logic_error);
  EXPECT_TRUE(backend_.executed_ids.empty());
}

TEST_F(GoToPositionTaskTest, NonFiniteMotionCostIsLogicError)
{
  backend_.candidate_costs = {{"a", NAN_VALUE}};
  GoToPositionTask task(
    std::make_unique<FakeTaskContext<GoToPositionResult>>(record_),
    candidates_request({make_candidate("a")}), backend_, named_poses_, settings_);

  EXPECT_THROW(task.execute(), std::logic_error);
  EXPECT_TRUE(backend_.executed_ids.empty());
}

// Settings.

TEST(GoToPositionSettings, RejectsInvalidValues)
{
  using Seconds = std::chrono::duration<double>;
  EXPECT_THROW(
    GoToPositionSettings(Seconds(0.0), SelectionWeights{1.0, 1.0}), std::invalid_argument);
  EXPECT_THROW(
    GoToPositionSettings(Seconds(INF_VALUE), SelectionWeights{1.0, 1.0}), std::invalid_argument);
  EXPECT_THROW(
    GoToPositionSettings(Seconds(1.0), SelectionWeights{-1.0, 1.0}), std::invalid_argument);
  EXPECT_THROW(
    GoToPositionSettings(Seconds(1.0), SelectionWeights{1.0, NAN_VALUE}), std::invalid_argument);
  EXPECT_NO_THROW(GoToPositionSettings(Seconds(1.0), SelectionWeights{0.0, 0.0}));
}

TEST(SelectionWeights, DefaultsWeighBothTermsEqually)
{
  const SelectionWeights weights;
  EXPECT_DOUBLE_EQ(weights.motion_cost, 1.0);
  EXPECT_DOUBLE_EQ(weights.quality, 1.0);
}

}  // namespace
}  // namespace manipulation_skill_server::test
