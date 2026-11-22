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
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "manipulation_skill_server/motion_backend.hpp"
#include "manipulation_skill_server/task_solution_executor.hpp"
#include "moveit/planning_scene/planning_scene.hpp"
#include "moveit/robot_model/robot_model.hpp"
#include "moveit/robot_model_loader/robot_model_loader.hpp"
#include "moveit/task_constructor/solvers/planner_interface.h"
#include "moveit/task_constructor/task.h"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "rclcpp/node.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace manipulation_skill_server
{

/// @brief Plans with the MoveIt Task Constructor (MTC) and executes through `move_group`.
///
/// Every plan is one MTC task: the start state, then one motion of the arm group. The start
/// state is the planning scene of `move_group`, fetched once per goal for all its candidates.
/// A joint-space (`PTP`) motion tries joint interpolation first (the shortest joint path) and
/// the planning pipeline (OMPL by default) where interpolation fails. To a TCP pose, it plans
/// to several inverse kinematics (IK) solutions and keeps the shortest motion, because the
/// first IK solution can be a far elbow or wrist flip. A `LIN` motion moves the TCP along a
/// straight line (`CartesianPath`), the whole line or nothing.
/// The motion cost is the joint-space path length.
/// Execution goes through `TaskSolutionExecutor`. The node's executor must spin while the
/// backend waits for `move_group`: the clients live on the node.
class MtcMotionBackend : public MotionBackend
{
public:
  /// @brief Loads the robot model, creates the planners and starts listening to TF.
  ///
  /// The URDF and SRDF come from the parameters `robot_description` and
  /// `robot_description_semantic`; if they are not set, from the same parameters of
  /// `move_group`, so the model is exactly the one `move_group` executes with.
  /// Parameters: `mtc.arm_group` and `mtc.tcp_frame` (required), `mtc.planning_pipeline`
  /// (default "ompl"), `mtc.move_group_node` (default "move_group"), and
  /// `robot_description_planning.default_velocity_scaling_factor` and
  /// `..._acceleration_scaling_factor` from the MoveIt configuration (required).
  /// @param node A node that declares its parameter overrides automatically; its executor
  ///   must already spin, because the backend waits for `move_group`'s answer.
  /// @throws std::invalid_argument if a parameter is missing or invalid.
  /// @throws std::runtime_error if the robot model cannot be loaded.
  explicit MtcMotionBackend(rclcpp::Node::SharedPtr node);

  /// @brief Plans one motion of the TCP (`mtc.tcp_frame`) per candidate.
  ///
  /// First fetches the planning scene of `move_group`, then transforms every candidate pose
  /// into the model frame through TF at the pose's stamp; all candidates share one short wait
  /// for missing transforms. A candidate whose transform fails gets no plan. Then plans the
  /// candidates in the given order: each gets an equal share of the budget left, so time one
  /// candidate does not use goes to the candidates after it. The message lists every
  /// candidate without a plan and why.
  PlanResult plan_to_candidates(
    const std::vector<TcpCandidate> & candidates, MotionProfile profile,
    std::chrono::duration<double> budget, const CancellationToken & cancellation) override;

  PlanResult plan_to_joints(
    const JointTarget & target, std::chrono::duration<double> budget,
    const CancellationToken & cancellation) override;

  /// @brief Executes the motion with `TaskSolutionExecutor`.
  /// @throws std::runtime_error if `move_group` does not finish the motion or ROS shuts
  ///   down during it: then the arm may still move.
  ExecutionResult execute(
    const PlannedMotion & motion, const CancellationToken & cancellation) override;

private:
  using TaskPtr = std::unique_ptr<moveit::task_constructor::Task>;
  using NamedPlanners =
    std::vector<std::pair<std::string, moveit::task_constructor::solvers::PlannerInterfacePtr>>;

  TaskPtr make_task(const std::string & name, const planning_scene::PlanningScenePtr & scene) const;
  TaskPtr make_ptp_task(
    const std::string & name, const planning_scene::PlanningScenePtr & scene,
    const geometry_msgs::msg::PoseStamped & target, std::chrono::duration<double> timeout) const;
  TaskPtr make_lin_task(
    const std::string & name, const planning_scene::PlanningScenePtr & scene,
    const geometry_msgs::msg::PoseStamped & target) const;

  // Null after the deadline or a cancel request.
  planning_scene::PlanningScenePtr current_scene(
    std::chrono::steady_clock::time_point deadline, const CancellationToken & cancellation) const;
  geometry_msgs::msg::PoseStamped to_model_frame(
    const StampedPose & pose, std::chrono::steady_clock::time_point wait_until_time,
    const CancellationToken & cancellation) const;

  PlanResult plan(
    TaskPtr task, const std::string & candidate_id, std::chrono::duration<double> budget,
    const CancellationToken & cancellation) const;

  rclcpp::Node::SharedPtr node_;
  std::string arm_group_;
  std::string tcp_frame_;
  TaskSolutionExecutor executor_;
  // Kept as long as the model: it owns the loaders of the model's kinematics plugins.
  std::unique_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelConstPtr robot_model_;
  // Joint interpolation, then the planning pipeline.
  NamedPlanners ptp_planners_;
  moveit::task_constructor::solvers::PlannerInterfacePtr cartesian_planner_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
  // The listener fills the buffer from its own thread, so it is destroyed first.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace manipulation_skill_server
