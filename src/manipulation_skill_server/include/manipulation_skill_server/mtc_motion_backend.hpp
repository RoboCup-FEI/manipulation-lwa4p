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
#include <memory>
#include <string>
#include <vector>

#include "manipulation_skill_server/motion_backend.hpp"
#include "moveit/robot_model/robot_model.hpp"
#include "moveit/robot_model_loader/robot_model_loader.hpp"
#include "moveit/task_constructor/solvers/planner_interface.h"
#include "moveit/task_constructor/task.h"
#include "moveit_task_constructor_msgs/action/execute_task_solution.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_action/client.hpp"

namespace manipulation_skill_server
{

/// @brief Plans with the MoveIt Task Constructor (MTC) and executes through `move_group`.
///
/// Every plan is one MTC task: the current state, then a joint-space motion that tries
/// joint interpolation first (the shortest joint path) and the planning pipeline (OMPL by
/// default) if interpolation collides. The motion cost is the joint-space path length.
/// Execution uses the `execute_task_solution` action of `move_group`, which needs
/// `move_group/ExecuteTaskSolutionCapability`. The node's executor must spin while a motion
/// executes: the action client lives on the node.
class MtcMotionBackend : public MotionBackend
{
public:
  /// @brief Loads the robot model and creates the planners.
  ///
  /// The URDF and SRDF come from the parameters `robot_description` and
  /// `robot_description_semantic`; if they are not set, from the same parameters of
  /// `move_group`, so the model is exactly the one `move_group` executes with.
  /// Parameters: `mtc.arm_group` (required), `mtc.planning_pipeline` (default "ompl"),
  /// `mtc.move_group_node` (default "move_group"), and
  /// `robot_description_planning.default_velocity_scaling_factor` and
  /// `..._acceleration_scaling_factor` from the MoveIt configuration (required).
  /// @param node A node that declares its parameter overrides automatically; its executor
  ///   must already spin, because the backend waits for `move_group`'s answer.
  /// @throws std::invalid_argument if a parameter is missing or invalid.
  /// @throws std::runtime_error if the robot model cannot be loaded.
  explicit MtcMotionBackend(rclcpp::Node::SharedPtr node);

  /// @brief Not supported yet: returns no plan and says so.
  PlanResult plan_to_candidates(
    const std::vector<TcpCandidate> & candidates, MotionProfile profile,
    std::chrono::duration<double> budget, const CancellationToken & cancellation) override;

  PlanResult plan_to_joints(
    const JointTarget & target, std::chrono::duration<double> budget,
    const CancellationToken & cancellation) override;

  /// @throws std::runtime_error if `move_group` does not finish the motion or ROS shuts
  ///   down during it: then the arm may still move.
  ExecutionResult execute(
    const PlannedMotion & motion, const CancellationToken & cancellation) override;

private:
  using ExecuteTaskSolution = moveit_task_constructor_msgs::action::ExecuteTaskSolution;

  PlanResult plan(
    std::unique_ptr<moveit::task_constructor::Task> task, const std::string & candidate_id,
    std::chrono::duration<double> budget, const CancellationToken & cancellation) const;

  rclcpp::Node::SharedPtr node_;
  std::string arm_group_;
  // Kept as long as the model: it owns the loaders of the model's kinematics plugins.
  std::unique_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::shared_ptr<moveit::task_constructor::solvers::PlannerInterface> interpolation_planner_;
  std::shared_ptr<moveit::task_constructor::solvers::PlannerInterface> pipeline_planner_;
  rclcpp_action::Client<ExecuteTaskSolution>::SharedPtr execute_client_;
};

}  // namespace manipulation_skill_server
