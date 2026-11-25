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
#include <memory>

#include "manipulation_interfaces/action/go_to_position.hpp"
#include "manipulation_skill_server/busy_task_runner.hpp"
#include "manipulation_skill_server/go_to_position_task.hpp"
#include "manipulation_skill_server/motion_backend.hpp"
#include "manipulation_skill_server/named_pose_table.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_action/server.hpp"

namespace manipulation_skill_server
{

/// @brief The ROS interface of the skill server: parameters and the `go_to_position` action.
///
/// It holds no logic: it converts goals to core tasks and hands them to one `BusyTaskRunner`.
/// It does not inherit from `rclcpp::Node`, so the MTC backend can share the node, which
/// needs the node's `shared_ptr` while it is constructed.
class SkillServer
{
public:
  using GoToPosition = manipulation_interfaces::action::GoToPosition;

  /// @brief Reads the parameters and starts the action server.
  /// @param node The node for parameters, logging and the action server.
  /// @param motion The motion backend.
  /// @throws std::invalid_argument if a parameter is missing or invalid.
  SkillServer(rclcpp::Node::SharedPtr node, std::unique_ptr<MotionBackend> motion);

  /// @brief Cancels the running goal and waits until its task returns.
  ///
  /// The caller of that goal gets no result: the action server is already gone.
  ~SkillServer();

  SkillServer(const SkillServer &) = delete;
  SkillServer & operator=(const SkillServer &) = delete;

private:
  void on_go_to_position_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoToPosition>> & goal_handle);

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<MotionBackend> motion_;
  GoToPositionSettings go_to_position_settings_;
  NamedPoseTable named_poses_;
  std::shared_ptr<std::atomic<bool>> stopping_ = std::make_shared<std::atomic<bool>>(false);
  // After everything a task uses: the destructor waits for the running task first.
  BusyTaskRunner runner_;
  // Last: destroyed first, so no goal arrives while the members above are destroyed.
  rclcpp_action::Server<GoToPosition>::SharedPtr go_to_position_server_;
};

}  // namespace manipulation_skill_server
