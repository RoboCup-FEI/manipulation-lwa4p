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

#include "manipulation_skill_server/skill_server_ros.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "manipulation_skill_server/conversions.hpp"
#include "manipulation_skill_server/goal_context.hpp"
#include "manipulation_skill_server/parameters.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_action/create_server.hpp"

namespace manipulation_skill_server
{

namespace
{

// MoveIt's default allowed planning time; here it is shared by all candidates of one goal.
constexpr double DEFAULT_PLANNING_BUDGET_S = 5.0;

GoToPositionSettings load_go_to_position_settings(rclcpp::Node & node)
{
  const SelectionWeights defaults;
  SelectionWeights weights;
  weights.motion_cost =
    parameter<double>(node, "go_to_position.selection_weights.motion_cost", defaults.motion_cost);
  weights.quality =
    parameter<double>(node, "go_to_position.selection_weights.quality", defaults.quality);
  const double budget_s =
    parameter<double>(node, "go_to_position.planning_budget_s", DEFAULT_PLANNING_BUDGET_S);
  return GoToPositionSettings(std::chrono::duration<double>(budget_s), weights);
}

NamedPoseTable load_named_poses(rclcpp::Node & node)
{
  const auto names = parameter<std::vector<std::string>>(node, "named_poses.names", {});
  const auto joint_names = parameter<std::vector<std::string>>(node, "named_poses.joint_names", {});

  std::map<std::string, JointTarget> poses;
  for (const std::string & name : names) {
    const std::string key = "named_poses." + name;
    const auto positions = parameter<std::vector<double>>(node, key, {});
    if (positions.size() != joint_names.size()) {
      throw std::invalid_argument(
        key + " has " + std::to_string(positions.size()) + " positions for " +
        std::to_string(joint_names.size()) + " joints in named_poses.joint_names");
    }
    JointTarget target;
    for (std::size_t index = 0; index < joint_names.size(); ++index) {
      target[joint_names[index]] = positions[index];
    }
    if (target.size() != joint_names.size()) {
      throw std::invalid_argument("named_poses.joint_names lists a joint twice");
    }
    if (!poses.emplace(name, std::move(target)).second) {
      throw std::invalid_argument("named_poses.names lists '" + name + "' twice");
    }
  }
  return NamedPoseTable(std::move(poses));
}

}  // namespace

SkillServer::SkillServer(rclcpp::Node::SharedPtr node, std::unique_ptr<MotionBackend> motion)
: node_(std::move(node)),
  motion_(std::move(motion)),
  go_to_position_settings_(load_go_to_position_settings(*node_)),
  named_poses_(load_named_poses(*node_))
{
  if (!motion_) {
    throw std::invalid_argument("the skill server needs a motion backend");
  }

  using GoalHandle = rclcpp_action::ServerGoalHandle<GoToPosition>;
  go_to_position_server_ = rclcpp_action::create_server<GoToPosition>(
    node_, "go_to_position",
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const GoToPosition::Goal>) {
      // Every goal is accepted, so it ends with an Outcome; while busy, the runner ends it
      // with BUSY.
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](std::shared_ptr<GoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
    [this](std::shared_ptr<GoalHandle> goal_handle) {on_go_to_position_accepted(goal_handle);});

  RCLCPP_INFO(
    node_->get_logger(), "go_to_position ready, planning budget %.2f s",
    go_to_position_settings_.planning_budget().count());
}

SkillServer::~SkillServer()
{
  // Before the members are destroyed: the runner's destructor waits for the running task,
  // which stops the arm once it sees the cancellation.
  stopping_->store(true);
}

void SkillServer::on_go_to_position_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoToPosition>> & goal_handle)
{
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(
    node_->get_logger(), "goal %s: go_to_position, %zu candidates, named target '%s'",
    rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(), goal->candidates.size(),
    goal->named_target.c_str());

  auto context = std::make_unique<GoalContext<GoToPosition, GoToPositionResult>>(
    goal_handle, stopping_, node_->get_logger());
  runner_.submit(std::make_unique<GoToPositionTask>(
    std::move(context), to_request(*goal), *motion_, named_poses_, go_to_position_settings_));
}

}  // namespace manipulation_skill_server
