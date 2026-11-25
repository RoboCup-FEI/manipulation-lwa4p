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

#include <atomic>
#include <exception>
#include <memory>
#include <utility>

#include "manipulation_skill_server/conversions.hpp"
#include "manipulation_skill_server/task.hpp"
#include "manipulation_skill_server/types.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_action/types.hpp"

namespace manipulation_skill_server
{

/// @brief Links one task to its accepted ROS action goal.
///
/// The goal status agrees with the outcome code, as `Outcome.msg` requires: `SUCCEEDED` with
/// `SUCCESS`, `CANCELED` with `CANCELED`, `ABORTED` with every other code.
/// @tparam ActionT The ROS action type.
/// @tparam ResultT The core result type; `to_ros_result()` converts it to `ActionT::Result`.
template<typename ActionT, typename ResultT>
class GoalContext : public TaskContext<ResultT>
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

  /// @param goal_handle The accepted goal.
  /// @param server_stopping Set when the server shuts down; it cancels the goal too.
  /// @param logger The logger for the goal's outcome and delivery errors.
  GoalContext(
    std::shared_ptr<GoalHandle> goal_handle,
    std::shared_ptr<const std::atomic<bool>> server_stopping, rclcpp::Logger logger)
  : goal_handle_(std::move(goal_handle)),
    server_stopping_(std::move(server_stopping)),
    logger_(std::move(logger))
  {
  }

  bool cancel_requested() const override
  {
    return goal_handle_->is_canceling() || server_stopping_->load();
  }

  void finish(const ResultT & result) override
  {
    const Outcome & outcome = result.outcome;
    const auto goal_id = rclcpp_action::to_string(goal_handle_->get_goal_id());
    if (outcome.code == OutcomeCode::Success) {
      RCLCPP_INFO(logger_, "goal %s: SUCCESS %s", goal_id.c_str(), outcome.message.c_str());
    } else {
      RCLCPP_WARN(
        logger_, "goal %s: %s %s", goal_id.c_str(), to_string(outcome.code),
        outcome.message.c_str());
    }

    try {
      auto ros_result = std::make_shared<typename ActionT::Result>(to_ros_result(result));
      if (outcome.code == OutcomeCode::Success) {
        goal_handle_->succeed(ros_result);
      } else if (outcome.code == OutcomeCode::Canceled && goal_handle_->is_canceling()) {
        goal_handle_->canceled(ros_result);
      } else {
        // Also CANCELED without a cancel request: ROS allows the CANCELED status only then.
        goal_handle_->abort(ros_result);
      }
    } catch (const std::exception & exception) {
      // finish() must not throw (TaskContext); the caller does not get this result.
      RCLCPP_ERROR(
        logger_, "goal %s: could not deliver the result: %s", goal_id.c_str(), exception.what());
    }
  }

private:
  std::shared_ptr<GoalHandle> goal_handle_;
  std::shared_ptr<const std::atomic<bool>> server_stopping_;
  rclcpp::Logger logger_;
};

}  // namespace manipulation_skill_server
