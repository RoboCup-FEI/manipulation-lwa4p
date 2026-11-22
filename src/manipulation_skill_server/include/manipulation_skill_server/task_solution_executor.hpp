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

#include "manipulation_skill_server/motion_backend.hpp"
#include "manipulation_skill_server/task.hpp"
#include "moveit_task_constructor_msgs/action/execute_task_solution.hpp"
#include "moveit_task_constructor_msgs/msg/solution.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/client_goal_handle.hpp"

namespace manipulation_skill_server
{

/// @brief How long `TaskSolutionExecutor` waits for `move_group`.
struct ExecutionTimeouts
{
  /// For the action server to appear, for the answer to a goal, and for the arm to stop
  /// after a cancel request.
  std::chrono::duration<double> answer{5.0};
  /// Time over the planned duration before the executor stops a motion. move_group aborts a
  /// slow execution earlier itself (trajectory_execution in the MoveIt configuration), so this
  /// only catches a move_group that stopped reporting.
  std::chrono::duration<double> margin{30.0};
};

/// @brief Executes MTC solutions through the `execute_task_solution` action of `move_group`.
///
/// Unlike MTC's `Task::execute()`, it waits after a cancel request until `move_group` reports
/// the end of the motion, so a returned result means the arm is at rest. The node's executor
/// must spin while a solution executes: the action client lives on the node.
class TaskSolutionExecutor
{
public:
  /// @param node The node of the action client.
  /// @param timeouts How long to wait for `move_group`.
  explicit TaskSolutionExecutor(
    rclcpp::Node::SharedPtr node, ExecutionTimeouts timeouts = ExecutionTimeouts());

  /// @brief Sends a solution to `move_group` and waits until the arm is at rest.
  ///
  /// A cancel request before the solution is sent ends with `Canceled` and no motion; a later
  /// one stops the arm. A motion that exceeds its planned duration plus the margin is stopped
  /// and ends with `Failed`.
  /// @param solution The solution of an MTC task.
  /// @param cancellation Polled while waiting.
  /// @return How the execution ended.
  /// @throws std::runtime_error if `move_group` does not answer the goal, does not stop the
  ///   arm after a cancel request, or ROS shuts down during the motion: the arm may still move.
  ExecutionResult execute(
    const moveit_task_constructor_msgs::msg::Solution & solution,
    const CancellationToken & cancellation);

private:
  using ExecuteTaskSolution = moveit_task_constructor_msgs::action::ExecuteTaskSolution;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTaskSolution>;

  ExecutionResult wait_for_result(
    const GoalHandle::SharedPtr & goal_handle, std::chrono::duration<double> planned_duration,
    const CancellationToken & cancellation);

  rclcpp::Node::SharedPtr node_;
  ExecutionTimeouts timeouts_;
  rclcpp_action::Client<ExecuteTaskSolution>::SharedPtr client_;
};

}  // namespace manipulation_skill_server
