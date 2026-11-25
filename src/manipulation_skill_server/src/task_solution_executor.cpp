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

#include "manipulation_skill_server/task_solution_executor.hpp"

#include <chrono>
#include <future>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "manipulation_skill_server/polling.hpp"
#include "moveit/utils/moveit_error_code.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_client.hpp"

namespace manipulation_skill_server
{

namespace
{

using Clock = std::chrono::steady_clock;

Clock::duration to_clock(std::chrono::duration<double> duration)
{
  return std::chrono::duration_cast<Clock::duration>(duration);
}

std::chrono::duration<double> planned_duration_of(
  const moveit_task_constructor_msgs::msg::Solution & solution)
{
  double seconds = 0.0;
  for (const auto & sub_trajectory : solution.sub_trajectory) {
    const auto & points = sub_trajectory.trajectory.joint_trajectory.points;
    if (!points.empty()) {
      seconds += rclcpp::Duration(points.back().time_from_start).seconds();
    }
  }
  return std::chrono::duration<double>(seconds);
}

}  // namespace

TaskSolutionExecutor::TaskSolutionExecutor(rclcpp::Node::SharedPtr node, ExecutionTimeouts timeouts)
: node_(std::move(node)),
  timeouts_(timeouts),
  client_(rclcpp_action::create_client<ExecuteTaskSolution>(node_, "execute_task_solution"))
{
}

ExecutionResult TaskSolutionExecutor::execute(
  const moveit_task_constructor_msgs::msg::Solution & solution,
  const CancellationToken & cancellation)
{
  const ExecutionResult canceled_before_motion{
    ExecutionStatus::Canceled, "canceled before the motion started"};
  const bool server_ready = wait_until(
    [this]() {return client_->action_server_is_ready();},
    Clock::now() + to_clock(timeouts_.answer), cancellation);
  // Checked again right before sending: the last moment a cancel request keeps the arm still.
  if (cancellation.cancel_requested()) {
    return canceled_before_motion;
  }
  if (!server_ready) {
    return {
      ExecutionStatus::Failed,
      "move_group has no execute_task_solution action; it needs "
      "move_group/ExecuteTaskSolutionCapability"};
  }

  ExecuteTaskSolution::Goal goal;
  goal.solution = solution;
  auto goal_future = client_->async_send_goal(goal);
  if (goal_future.wait_for(to_clock(timeouts_.answer)) != std::future_status::ready) {
    throw std::runtime_error("move_group did not answer the execution request");
  }
  const GoalHandle::SharedPtr goal_handle = goal_future.get();
  if (!goal_handle) {
    return {ExecutionStatus::Failed, "move_group rejected the motion"};
  }
  return wait_for_result(goal_handle, planned_duration_of(solution), cancellation);
}

ExecutionResult TaskSolutionExecutor::wait_for_result(
  const GoalHandle::SharedPtr & goal_handle, std::chrono::duration<double> planned_duration,
  const CancellationToken & cancellation)
{
  auto result_future = client_->async_get_result(goal_handle);
  const auto context = node_->get_node_base_interface()->get_context();
  const auto execution_deadline = Clock::now() + to_clock(planned_duration + timeouts_.margin);
  // Set once the executor asks move_group to stop the arm; from then on only the stop counts.
  std::optional<Clock::time_point> stop_deadline;
  bool timed_out = false;
  while (result_future.wait_for(POLL_PERIOD) != std::future_status::ready) {
    if (!rclcpp::ok(context)) {
      throw std::runtime_error("ROS shut down during the motion; the arm may still move");
    }
    if (!stop_deadline) {
      timed_out = Clock::now() > execution_deadline;
      if (timed_out || cancellation.cancel_requested()) {
        client_->async_cancel_goal(goal_handle);
        stop_deadline = Clock::now() + to_clock(timeouts_.answer);
      }
    } else if (Clock::now() > *stop_deadline) {
      throw std::runtime_error("move_group did not stop the motion after a cancel request");
    }
  }

  const auto wrapped = result_future.get();
  // A motion that ended at its target counts, even if a stop was requested meanwhile.
  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
    return {ExecutionStatus::Succeeded, ""};
  }
  if (timed_out) {
    return {ExecutionStatus::Failed, "move_group did not finish the motion in time; stopped it"};
  }
  const std::string code_text =
    wrapped.result ?
    moveit::core::errorCodeToString(moveit::core::MoveItErrorCode(wrapped.result->error_code)) :
    "no result";
  if (stop_deadline) {
    return {ExecutionStatus::Canceled, "stopped on a cancel request (" + code_text + ")"};
  }
  return {ExecutionStatus::Failed, "move_group aborted the motion: " + code_text};
}

}  // namespace manipulation_skill_server
