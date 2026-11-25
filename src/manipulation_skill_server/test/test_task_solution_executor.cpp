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
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "manipulation_skill_server/task_solution_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace manipulation_skill_server::test
{
namespace
{

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using ExecuteTaskSolution = moveit_task_constructor_msgs::action::ExecuteTaskSolution;
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTaskSolution>;

class Cancellation : public CancellationToken
{
public:
  bool cancel_requested() const override {return requested.load();}

  std::atomic<bool> requested{false};
};

/// What the fake move_group does with a goal.
struct FakeBehaviour
{
  bool accept{true};
  /// How long the motion takes; empty: it never ends by itself.
  std::optional<std::chrono::milliseconds> motion;
  /// Whether the motion ends at its target or move_group aborts it.
  bool succeeds{true};
  /// How long the arm takes to stop after a cancel request; empty: it never stops.
  std::optional<std::chrono::milliseconds> stop{0ms};
};

/// An execute_task_solution action server in move_group's place.
class FakeMoveGroup
{
public:
  FakeMoveGroup(const rclcpp::Node::SharedPtr & node, FakeBehaviour behaviour)
  : behaviour_(behaviour)
  {
    server_ = rclcpp_action::create_server<ExecuteTaskSolution>(
      node, "execute_task_solution",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteTaskSolution::Goal>) {
        ++goals;
        return behaviour_.accept ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
               rclcpp_action::GoalResponse::REJECT;
      },
      [this](std::shared_ptr<ServerGoalHandle>) {
        ++cancels;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<ServerGoalHandle> goal_handle) {
        motions_.emplace_back([this, goal_handle]() {move(goal_handle);});
      });
  }

  ~FakeMoveGroup()
  {
    stopping_ = true;
    for (std::thread & motion : motions_) {
      motion.join();
    }
  }

  std::atomic<int> goals{0};
  std::atomic<int> cancels{0};

private:
  void move(const std::shared_ptr<ServerGoalHandle> & goal_handle)
  {
    const auto result = std::make_shared<ExecuteTaskSolution::Result>();
    const auto start = Clock::now();
    while (!stopping_) {
      if (goal_handle->is_canceling() && behaviour_.stop) {
        std::this_thread::sleep_for(*behaviour_.stop);
        goal_handle->canceled(result);
        return;
      }
      if (
        !goal_handle->is_canceling() && behaviour_.motion &&
        Clock::now() - start >= *behaviour_.motion)
      {
        if (behaviour_.succeeds) {
          goal_handle->succeed(result);
        } else {
          goal_handle->abort(result);
        }
        return;
      }
      std::this_thread::sleep_for(5ms);
    }
    goal_handle->abort(result);
  }

  FakeBehaviour behaviour_;
  rclcpp_action::Server<ExecuteTaskSolution>::SharedPtr server_;
  std::vector<std::thread> motions_;
  std::atomic<bool> stopping_{false};
};

/// The executor's node spins in its own thread, as in the skill server; the test thread plays
/// the worker.
class TaskSolutionExecutorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}

  static void TearDownTestSuite() {rclcpp::shutdown();}

  TaskSolutionExecutorTest()
  : node_(std::make_shared<rclcpp::Node>("executor_test", unique_namespace()))
  {
    executor_.add_node(node_);
    spin_thread_ = std::thread([this]() {executor_.spin();});
  }

  ~TaskSolutionExecutorTest() override
  {
    executor_.cancel();
    spin_thread_.join();
  }

  /// A namespace of this test only: a real move_group may run on the same ROS domain.
  static std::string unique_namespace()
  {
    static int counter = 0;
    return "/task_solution_executor_test_" + std::to_string(::getpid()) + "_" +
           std::to_string(counter++);
  }

  void start_move_group(FakeBehaviour behaviour)
  {
    move_group_ = std::make_unique<FakeMoveGroup>(node_, behaviour);
  }

  ExecutionResult execute(ExecutionTimeouts timeouts)
  {
    TaskSolutionExecutor executor(node_, timeouts);
    return executor.execute(moveit_task_constructor_msgs::msg::Solution(), cancellation_);
  }

  /// Requests cancellation from another thread after `delay`, as the action server does.
  std::thread cancel_after(std::chrono::milliseconds delay)
  {
    return std::thread([this, delay]() {
               std::this_thread::sleep_for(delay);
               cancellation_.requested = true;
    });
  }

  static double seconds_since(Clock::time_point start)
  {
    return std::chrono::duration<double>(Clock::now() - start).count();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  Cancellation cancellation_;
  std::unique_ptr<FakeMoveGroup> move_group_;
};

TEST_F(TaskSolutionExecutorTest, SucceedsWhenMoveGroupFinishesTheMotion)
{
  FakeBehaviour behaviour;
  behaviour.motion = 100ms;
  start_move_group(behaviour);

  const ExecutionResult result = execute({1s, 5s});

  EXPECT_EQ(result.status, ExecutionStatus::Succeeded) << result.message;
  EXPECT_EQ(move_group_->goals, 1);
  EXPECT_EQ(move_group_->cancels, 0);
}

TEST_F(TaskSolutionExecutorTest, AbortedMotionFails)
{
  FakeBehaviour behaviour;
  behaviour.motion = 100ms;
  behaviour.succeeds = false;
  start_move_group(behaviour);

  const ExecutionResult result = execute({1s, 5s});

  EXPECT_EQ(result.status, ExecutionStatus::Failed);
  EXPECT_NE(result.message.find("aborted"), std::string::npos) << result.message;
}

TEST_F(TaskSolutionExecutorTest, RejectedMotionFails)
{
  FakeBehaviour behaviour;
  behaviour.accept = false;
  start_move_group(behaviour);

  const ExecutionResult result = execute({1s, 5s});

  EXPECT_EQ(result.status, ExecutionStatus::Failed);
  EXPECT_NE(result.message.find("rejected"), std::string::npos) << result.message;
}

TEST_F(TaskSolutionExecutorTest, MissingMoveGroupFailsAfterTheAnswerTimeout)
{
  const auto start = Clock::now();
  const ExecutionResult result = execute({300ms, 5s});

  EXPECT_EQ(result.status, ExecutionStatus::Failed);
  EXPECT_NE(result.message.find("ExecuteTaskSolutionCapability"), std::string::npos);
  EXPECT_LT(seconds_since(start), 2.0);
}

TEST_F(TaskSolutionExecutorTest, CancelWhileWaitingForMoveGroupSendsNoMotion)
{
  std::thread canceler = cancel_after(200ms);
  const auto start = Clock::now();
  const ExecutionResult result = execute({5s, 5s});
  canceler.join();

  EXPECT_EQ(result.status, ExecutionStatus::Canceled);
  EXPECT_NE(result.message.find("before the motion"), std::string::npos) << result.message;
  EXPECT_LT(seconds_since(start), 2.0) << "the wait for move_group ignored the cancel request";
}

TEST_F(TaskSolutionExecutorTest, CancelBeforeSendingSendsNoMotion)
{
  FakeBehaviour behaviour;
  behaviour.motion = 100ms;
  start_move_group(behaviour);
  cancellation_.requested = true;

  const ExecutionResult result = execute({1s, 5s});

  EXPECT_EQ(result.status, ExecutionStatus::Canceled);
  EXPECT_EQ(move_group_->goals, 0);
}

TEST_F(TaskSolutionExecutorTest, CancelDuringTheMotionWaitsUntilTheArmStops)
{
  FakeBehaviour behaviour;
  behaviour.motion = 10s;
  behaviour.stop = 300ms;
  start_move_group(behaviour);

  std::thread canceler = cancel_after(200ms);
  const auto start = Clock::now();
  const ExecutionResult result = execute({2s, 5s});
  canceler.join();

  EXPECT_EQ(result.status, ExecutionStatus::Canceled) << result.message;
  EXPECT_EQ(move_group_->cancels, 1);
  EXPECT_GE(seconds_since(start), 0.45) << "returned before move_group reported the stop";
  EXPECT_LT(seconds_since(start), 3.0);
}

TEST_F(TaskSolutionExecutorTest, MotionOverItsPlannedDurationIsStoppedAndFails)
{
  // The stop takes longer than one poll period: the executor must wait for it.
  FakeBehaviour behaviour;
  behaviour.stop = 300ms;
  start_move_group(behaviour);

  const ExecutionResult result = execute({2s, 200ms});

  EXPECT_EQ(result.status, ExecutionStatus::Failed) << result.message;
  EXPECT_NE(result.message.find("in time"), std::string::npos) << result.message;
  EXPECT_EQ(move_group_->cancels, 1);
}

TEST_F(TaskSolutionExecutorTest, ThrowsWhenMoveGroupDoesNotStopTheArm)
{
  FakeBehaviour behaviour;
  behaviour.motion = 10s;
  behaviour.stop.reset();
  start_move_group(behaviour);

  std::thread canceler = cancel_after(200ms);
  EXPECT_THROW(execute({300ms, 5s}), std::runtime_error);
  canceler.join();
}

}  // namespace
}  // namespace manipulation_skill_server::test
