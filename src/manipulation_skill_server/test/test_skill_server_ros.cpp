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
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <future>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fakes.hpp"
#include "manipulation_skill_server/skill_server_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace manipulation_skill_server::test
{
namespace
{

using GoToPosition = manipulation_interfaces::action::GoToPosition;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<GoToPosition>;
using OutcomeMsg = manipulation_interfaces::msg::Outcome;

constexpr std::chrono::seconds TIMEOUT{10};

struct FinishedGoal
{
  rclcpp_action::ResultCode status{rclcpp_action::ResultCode::UNKNOWN};
  GoToPosition::Result result;
};

std::vector<rclcpp::Parameter> test_parameters()
{
  return {
    rclcpp::Parameter("named_poses.names", std::vector<std::string>{"home"}),
    rclcpp::Parameter("named_poses.joint_names", std::vector<std::string>{"joint_a", "joint_b"}),
    rclcpp::Parameter("named_poses.home", std::vector<double>{0.0, 0.5}),
    rclcpp::Parameter("go_to_position.planning_budget_s", 1.5),
    rclcpp::Parameter("go_to_position.selection_weights.motion_cost", 1.0),
    rclcpp::Parameter("go_to_position.selection_weights.quality", 0.0),
  };
}

GoToPosition::Goal named_goal(const std::string & name)
{
  GoToPosition::Goal goal;
  goal.named_target = name;
  return goal;
}

manipulation_interfaces::msg::TcpCandidate candidate_msg(const std::string & id, double quality)
{
  manipulation_interfaces::msg::TcpCandidate msg;
  msg.id = id;
  msg.tcp_pose.header.frame_id = "arm_base_link";
  msg.tcp_pose.pose.position.x = 0.4;
  msg.tcp_pose.pose.orientation.w = 1.0;
  msg.quality = quality;
  return msg;
}

/// A server node and a client node in one executor, on a namespace of this test only.
class SkillServerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}

  static void TearDownTestSuite() {rclcpp::shutdown();}

  void TearDown() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    server_.reset();
  }

  /// A node options object for the server node.
  static rclcpp::NodeOptions options_with(
    std::vector<rclcpp::Parameter> parameters, bool declare_overrides = false)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(std::move(parameters));
    options.automatically_declare_parameters_from_overrides(declare_overrides);
    return options;
  }

  rclcpp::Node::SharedPtr make_node(
    const std::string & name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  {
    static int counter = 0;
    if (namespace_.empty()) {
      namespace_ =
        "/skill_server_test_" + std::to_string(::getpid()) + "_" + std::to_string(counter++);
    }
    return std::make_shared<rclcpp::Node>(name, namespace_, options);
  }

  /// Starts the server with a fake backend and a client, and waits for the action server.
  void start(const rclcpp::NodeOptions & options = options_with(test_parameters()))
  {
    auto server_node = make_node("skill_server", options);
    auto backend = std::make_unique<FakeMotionBackend>();
    backend_ = backend.get();
    server_ = std::make_unique<SkillServer>(server_node, std::move(backend));

    auto client_node = make_node("client");
    client_ = rclcpp_action::create_client<GoToPosition>(client_node, "go_to_position");

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(server_node);
    executor_->add_node(client_node);
    spin_thread_ = std::thread([this]() {executor_->spin();});
    ASSERT_TRUE(client_->wait_for_action_server(TIMEOUT));
  }

  ClientGoalHandle::SharedPtr send(const GoToPosition::Goal & goal)
  {
    auto future = client_->async_send_goal(goal);
    EXPECT_EQ(future.wait_for(TIMEOUT), std::future_status::ready);
    auto handle = future.get();
    EXPECT_NE(handle, nullptr) << "the goal was rejected";
    return handle;
  }

  FinishedGoal wait_for_result(const ClientGoalHandle::SharedPtr & handle)
  {
    auto future = client_->async_get_result(handle);
    EXPECT_EQ(future.wait_for(TIMEOUT), std::future_status::ready);
    const auto wrapped = future.get();
    return {wrapped.code, *wrapped.result};
  }

  FinishedGoal send_and_wait(const GoToPosition::Goal & goal)
  {
    return wait_for_result(send(goal));
  }

  std::string namespace_;
  FakeMotionBackend * backend_{nullptr};
  std::unique_ptr<SkillServer> server_;
  rclcpp_action::Client<GoToPosition>::SharedPtr client_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TEST_F(SkillServerTest, NamedTargetSucceedsWithJointsAndBudgetFromParameters)
{
  start();

  const FinishedGoal finished = send_and_wait(named_goal("home"));

  EXPECT_EQ(finished.status, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(finished.result.outcome.code, OutcomeMsg::SUCCESS);
  EXPECT_TRUE(finished.result.selected_candidate_id.empty());
  const JointTarget expected{{"joint_a", 0.0}, {"joint_b", 0.5}};
  EXPECT_EQ(backend_->planned_joints, expected);
  EXPECT_DOUBLE_EQ(backend_->planned_budget.count(), 1.5);
}

TEST_F(SkillServerTest, CandidateGoalReportsSelectedCandidate)
{
  start();
  backend_->candidate_costs = {{"b", 1.0}, {"a", 2.0}};
  GoToPosition::Goal goal;
  goal.candidates = {
    candidate_msg("a", std::numeric_limits<double>::quiet_NaN()), candidate_msg("b", 0.5)};

  const FinishedGoal finished = send_and_wait(goal);

  EXPECT_EQ(finished.status, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(finished.result.selected_candidate_id, "b");
  ASSERT_EQ(backend_->planned_candidates.size(), 2u);
  EXPECT_FALSE(backend_->planned_candidates[0].quality.has_value());
  EXPECT_EQ(backend_->planned_candidates[1].tcp_pose.frame_id, "arm_base_link");
}

TEST_F(SkillServerTest, InvalidGoalIsAbortedWithInvalidGoal)
{
  start();

  const FinishedGoal finished = send_and_wait(GoToPosition::Goal{});

  EXPECT_EQ(finished.status, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(finished.result.outcome.code, OutcomeMsg::INVALID_GOAL);
  EXPECT_EQ(backend_->plan_calls, 0);
}

TEST_F(SkillServerTest, UnknownProfileIsAbortedWithInvalidGoal)
{
  start();
  GoToPosition::Goal goal;
  goal.candidates = {candidate_msg("a", 0.0)};
  goal.profile = 7;

  const FinishedGoal finished = send_and_wait(goal);

  EXPECT_EQ(finished.status, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(finished.result.outcome.code, OutcomeMsg::INVALID_GOAL);
}

TEST_F(SkillServerTest, GoalWhileBusyIsAbortedWithBusyAndFirstGoalSucceeds)
{
  start();
  std::promise<void> started;
  std::promise<void> gate;
  auto gate_future = gate.get_future().share();
  backend_->during_execution = [&started, gate_future](const CancellationToken &) {
      started.set_value();
      gate_future.wait();
    };

  const auto first = send(named_goal("home"));
  ASSERT_EQ(started.get_future().wait_for(TIMEOUT), std::future_status::ready);
  const FinishedGoal second = send_and_wait(named_goal("home"));
  gate.set_value();
  const FinishedGoal first_finished = wait_for_result(first);

  EXPECT_EQ(second.status, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(second.result.outcome.code, OutcomeMsg::BUSY);
  EXPECT_EQ(first_finished.status, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(first_finished.result.outcome.code, OutcomeMsg::SUCCESS);
}

TEST_F(SkillServerTest, CanceledGoalEndsWithCanceledStatus)
{
  start();
  std::promise<void> started;
  // The fake stops "the arm" once the server sees the cancel request.
  backend_->during_execution = [this, &started](const CancellationToken & cancellation) {
      started.set_value();
      const auto deadline = std::chrono::steady_clock::now() + TIMEOUT;
      while (!cancellation.cancel_requested() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      backend_->execution_result = {ExecutionStatus::Canceled, "stopped on request"};
    };

  const auto handle = send(named_goal("home"));
  ASSERT_EQ(started.get_future().wait_for(TIMEOUT), std::future_status::ready);
  auto cancel = client_->async_cancel_goal(handle);
  ASSERT_EQ(cancel.wait_for(TIMEOUT), std::future_status::ready);
  const FinishedGoal finished = wait_for_result(handle);

  EXPECT_EQ(finished.status, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(finished.result.outcome.code, OutcomeMsg::CANCELED);
  EXPECT_EQ(finished.result.outcome.message, "stopped on request");
}

TEST_F(SkillServerTest, ServerShutdownCancelsRunningGoal)
{
  start();
  std::promise<void> started;
  auto saw_cancel = std::make_shared<std::atomic<bool>>(false);
  backend_->during_execution = [this, &started,
      saw_cancel](const CancellationToken & cancellation) {
      started.set_value();
      const auto deadline = std::chrono::steady_clock::now() + TIMEOUT;
      while (!cancellation.cancel_requested() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      saw_cancel->store(cancellation.cancel_requested());
      backend_->execution_result = {ExecutionStatus::Canceled, "stopped on shutdown"};
    };
  send(named_goal("home"));
  ASSERT_EQ(started.get_future().wait_for(TIMEOUT), std::future_status::ready);
  executor_->cancel();
  spin_thread_.join();

  const auto before = std::chrono::steady_clock::now();
  server_.reset();

  EXPECT_TRUE(saw_cancel->load());
  EXPECT_LT(std::chrono::steady_clock::now() - before, TIMEOUT / 2);
}

TEST_F(SkillServerTest, CanceledOutcomeWithoutCancelRequestIsAborted)
{
  // ROS allows the CANCELED status only after a cancel request.
  start();
  backend_->execution_result = {ExecutionStatus::Canceled, "stopped by someone else"};

  const FinishedGoal finished = send_and_wait(named_goal("home"));

  EXPECT_EQ(finished.status, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(finished.result.outcome.code, OutcomeMsg::CANCELED);
}

TEST_F(SkillServerTest, BackendExceptionAbortsWithInternalErrorAndLaterGoalsToo)
{
  start();
  backend_->during_execution = [](const CancellationToken &) {
      throw std::runtime_error("arm state unknown");
    };

  const FinishedGoal failed = send_and_wait(named_goal("home"));
  const FinishedGoal later = send_and_wait(named_goal("home"));

  EXPECT_EQ(failed.status, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(failed.result.outcome.code, OutcomeMsg::INTERNAL_ERROR);
  EXPECT_EQ(later.status, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(later.result.outcome.code, OutcomeMsg::INTERNAL_ERROR);
}

TEST_F(SkillServerTest, ReadsParametersDeclaredFromOverrides)
{
  // The MTC backend will need a node with automatically declared parameters.
  start(options_with(test_parameters(), true));

  const FinishedGoal finished = send_and_wait(named_goal("home"));

  EXPECT_EQ(finished.status, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_DOUBLE_EQ(backend_->planned_budget.count(), 1.5);
}

TEST_F(SkillServerTest, DefaultsWithoutParameters)
{
  start(rclcpp::NodeOptions());
  backend_->candidate_costs = {{"a", 1.0}};
  GoToPosition::Goal goal;
  goal.candidates = {candidate_msg("a", 0.0)};

  const FinishedGoal candidate = send_and_wait(goal);
  const FinishedGoal named = send_and_wait(named_goal("home"));

  EXPECT_EQ(candidate.status, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_DOUBLE_EQ(backend_->planned_budget.count(), 5.0);
  EXPECT_EQ(named.result.outcome.code, OutcomeMsg::INVALID_GOAL);
}

class SkillServerParameterTest : public SkillServerTest
{
protected:
  /// Expects the server to refuse to start with the test parameters plus `changed`.
  void expect_refused(const rclcpp::Parameter & changed)
  {
    auto parameters = test_parameters();
    for (auto & parameter : parameters) {
      if (parameter.get_name() == changed.get_name()) {
        parameter = changed;
      }
    }
    auto node = make_node("skill_server", options_with(parameters));
    EXPECT_THROW(SkillServer(node, std::make_unique<FakeMotionBackend>()), std::invalid_argument)
      << changed.get_name();
  }
};

TEST_F(SkillServerParameterTest, RefusesInvalidParameters)
{
  expect_refused(rclcpp::Parameter("named_poses.home", std::vector<double>{0.0}));
  expect_refused(
    rclcpp::Parameter("named_poses.joint_names", std::vector<std::string>{"joint_a", "joint_a"}));
  expect_refused(rclcpp::Parameter("go_to_position.planning_budget_s", 0.0));
  expect_refused(rclcpp::Parameter("go_to_position.selection_weights.quality", -1.0));
}

TEST_F(SkillServerParameterTest, RefusesMissingBackend)
{
  auto node = make_node("skill_server", options_with(test_parameters()));
  EXPECT_THROW(SkillServer(node, nullptr), std::invalid_argument);
}

}  // namespace
}  // namespace manipulation_skill_server::test
