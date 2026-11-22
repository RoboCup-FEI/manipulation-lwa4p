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

#include "manipulation_skill_server/mtc_motion_backend.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "manipulation_skill_server/parameters.hpp"
#include "moveit/task_constructor/container.h"
#include "moveit/task_constructor/cost_terms.h"
#include "moveit/task_constructor/solvers/joint_interpolation.h"
#include "moveit/task_constructor/solvers/pipeline_planner.h"
#include "moveit/task_constructor/stages/current_state.h"
#include "moveit/task_constructor/stages/move_to.h"
#include "moveit/utils/moveit_error_code.hpp"
#include "rclcpp/rclcpp.hpp"

namespace manipulation_skill_server
{

namespace
{

namespace mtc = moveit::task_constructor;
using Clock = std::chrono::steady_clock;

// How often the worker checks the futures and the cancel request while it waits.
constexpr std::chrono::milliseconds POLL_PERIOD{10};
// How long move_group may take to answer an execution or a cancel request.
constexpr std::chrono::seconds ANSWER_TIMEOUT{5};
// How long the server waits for move_group's parameters when it starts.
constexpr std::chrono::seconds MOVE_GROUP_TIMEOUT{60};
// Time over the planned duration before the server stops waiting for move_group. move_group
// itself aborts a slow execution earlier (trajectory_execution in the MoveIt configuration),
// so this only catches a move_group that stopped answering.
constexpr std::chrono::seconds EXECUTION_TIMEOUT_MARGIN{30};

class MtcPlannedMotion : public PlannedMotion
{
public:
  MtcPlannedMotion(
    std::string candidate_id, std::unique_ptr<mtc::Task> task, mtc::SolutionBaseConstPtr solution)
  : PlannedMotion(std::move(candidate_id), solution->cost()),
    task_(std::move(task)),
    solution_(std::move(solution))
  {
  }

  const mtc::SolutionBase & solution() const {return *solution_;}

private:
  // The solution refers to the task's stages: the task must outlive it.
  std::unique_ptr<mtc::Task> task_;
  mtc::SolutionBaseConstPtr solution_;
};

std::chrono::duration<double> planned_duration(
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

template<typename FutureT>
bool ready_within(const FutureT & future, Clock::duration timeout)
{
  return future.wait_for(timeout) == std::future_status::ready;
}

// MTC must plan with the robot model move_group executes with. In Gazebo and Isaac Sim its
// URDF and SRDF differ from the MoveIt package's, and move_group may not publish the SRDF,
// so the server copies both from move_group's parameters unless its own are set.
void use_move_group_descriptions(const rclcpp::Node::SharedPtr & node)
{
  const std::vector<std::string> names = {"robot_description", "robot_description_semantic"};
  if (node->has_parameter(names[0]) && node->has_parameter(names[1])) {
    return;
  }
  const auto move_group = parameter<std::string>(*node, "mtc.move_group_node", "move_group");
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, move_group);
  RCLCPP_INFO(node->get_logger(), "reading the robot description from %s", move_group.c_str());
  if (!client->wait_for_service(MOVE_GROUP_TIMEOUT)) {
    throw std::runtime_error(move_group + " did not answer for its robot description");
  }
  auto parameters = client->get_parameters(names);
  if (!ready_within(parameters, ANSWER_TIMEOUT)) {
    throw std::runtime_error(move_group + " did not send its robot description");
  }
  for (const rclcpp::Parameter & description : parameters.get()) {
    if (description.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      throw std::runtime_error(move_group + " has no parameter " + description.get_name());
    }
    if (!node->has_parameter(description.get_name())) {
      node->declare_parameter(description.get_name(), description.as_string());
    }
  }
}

double scaling_factor(rclcpp::Node & node, const std::string & name)
{
  const double factor = required_parameter<double>(node, name);
  if (!(factor > 0.0 && factor <= 1.0)) {
    throw std::invalid_argument(name + " must be in (0, 1]");
  }
  return factor;
}

}  // namespace

MtcMotionBackend::MtcMotionBackend(rclcpp::Node::SharedPtr node)
: node_(std::move(node)), arm_group_(required_parameter<std::string>(*node_, "mtc.arm_group"))
{
  use_move_group_descriptions(node_);
  robot_model_loader_ =
    std::make_unique<robot_model_loader::RobotModelLoader>(node_, "robot_description");
  robot_model_ = robot_model_loader_->getModel();
  if (!robot_model_) {
    throw std::runtime_error("could not load the robot model from robot_description");
  }
  if (!robot_model_->hasJointModelGroup(arm_group_)) {
    throw std::invalid_argument("mtc.arm_group: the robot model has no group '" + arm_group_ + "'");
  }

  // MTC ignores MoveIt's default scaling factors; without them it would move faster than
  // MoveIt allows (AGENTS.md, section 7).
  const double velocity_scaling =
    scaling_factor(*node_, "robot_description_planning.default_velocity_scaling_factor");
  const double acceleration_scaling =
    scaling_factor(*node_, "robot_description_planning.default_acceleration_scaling_factor");
  interpolation_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  pipeline_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(
    node_, parameter<std::string>(*node_, "mtc.planning_pipeline", "ompl"));
  for (const auto & planner : {interpolation_planner_, pipeline_planner_}) {
    planner->setMaxVelocityScalingFactor(velocity_scaling);
    planner->setMaxAccelerationScalingFactor(acceleration_scaling);
  }

  execute_client_ =
    rclcpp_action::create_client<ExecuteTaskSolution>(node_, "execute_task_solution");
}

PlanResult MtcMotionBackend::plan_to_candidates(
  const std::vector<TcpCandidate> &, MotionProfile, std::chrono::duration<double>,
  const CancellationToken &)
{
  PlanResult result;
  result.message = "the MTC backend cannot plan to TCP candidates yet; use named_target";
  return result;
}

PlanResult MtcMotionBackend::plan_to_joints(
  const JointTarget & target, std::chrono::duration<double> budget,
  const CancellationToken & cancellation)
{
  auto task = std::make_unique<mtc::Task>();
  task->stages()->setName("go_to_position: joint target");
  task->setRobotModel(robot_model_);
  task->add(std::make_unique<mtc::stages::CurrentState>("current state"));

  // The first child that finds a plan wins: interpolation gives the shortest joint path.
  auto motion = std::make_unique<mtc::Fallbacks>("joint-space motion");
  const std::vector<std::pair<std::string, mtc::solvers::PlannerInterfacePtr>> planners = {
    {"joint interpolation", interpolation_planner_},
    {"planning pipeline", pipeline_planner_},
  };
  for (const auto & [name, planner] : planners) {
    auto move = std::make_unique<mtc::stages::MoveTo>(name, planner);
    move->setGroup(arm_group_);
    move->setGoal(target);
    move->setTimeout(budget.count());
    move->setCostTerm(mtc::CostTermConstPtr(std::make_shared<mtc::cost::PathLength>()));
    motion->add(std::move(move));
  }
  task->add(std::move(motion));
  return plan(std::move(task), "", budget, cancellation);
}

PlanResult MtcMotionBackend::plan(
  std::unique_ptr<mtc::Task> task, const std::string & candidate_id,
  std::chrono::duration<double> budget, const CancellationToken & cancellation) const
{
  task->setTimeout(budget.count());
  mtc::Task * const planning_task = task.get();
  // Runs between planning steps; MTC stops at the next one.
  const auto callback = task->addTaskCallback([planning_task, &cancellation](const mtc::Task &) {
        if (cancellation.cancel_requested()) {
          planning_task->preempt();
        }
  });

  PlanResult result;
  moveit::core::MoveItErrorCode code;
  try {
    code = task->plan(1);
  } catch (const mtc::InitStageException & exception) {
    std::ostringstream description;
    description << exception;
    result.message = "invalid MTC task: " + description.str();
    return result;
  }
  task->eraseTaskCallback(callback);

  if (task->solutions().empty()) {
    std::ostringstream failure;
    task->explainFailure(failure);
    result.message = "no plan (" + moveit::core::errorCodeToString(code) + ")";
    if (!failure.str().empty()) {
      result.message += ": " + failure.str();
    }
    return result;
  }
  auto solution = task->solutions().front();
  result.plans.push_back(
    std::make_unique<MtcPlannedMotion>(candidate_id, std::move(task), std::move(solution)));
  return result;
}

ExecutionResult MtcMotionBackend::execute(
  const PlannedMotion & motion, const CancellationToken & cancellation)
{
  ExecuteTaskSolution::Goal goal;
  dynamic_cast<const MtcPlannedMotion &>(motion).solution().toMsg(goal.solution);

  if (!execute_client_->wait_for_action_server(ANSWER_TIMEOUT)) {
    return {
      ExecutionStatus::Failed,
      "move_group has no execute_task_solution action; it needs "
      "move_group/ExecuteTaskSolutionCapability"};
  }
  auto goal_future = execute_client_->async_send_goal(goal);
  if (!ready_within(goal_future, ANSWER_TIMEOUT)) {
    throw std::runtime_error("move_group did not answer the execution request");
  }
  const auto goal_handle = goal_future.get();
  if (!goal_handle) {
    return {ExecutionStatus::Failed, "move_group rejected the motion"};
  }
  auto result_future = execute_client_->async_get_result(goal_handle);

  const auto context = node_->get_node_base_interface()->get_context();
  auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                   planned_duration(goal.solution) + EXECUTION_TIMEOUT_MARGIN);
  bool cancel_sent = false;
  bool timed_out = false;
  while (!ready_within(result_future, POLL_PERIOD)) {
    if (!rclcpp::ok(context)) {
      throw std::runtime_error("ROS shut down during the motion; the arm may still move");
    }
    timed_out = timed_out || Clock::now() > deadline;
    if (!cancel_sent && (cancellation.cancel_requested() || timed_out)) {
      execute_client_->async_cancel_goal(goal_handle);
      cancel_sent = true;
      deadline = std::min(deadline, Clock::now() + ANSWER_TIMEOUT);
    } else if (cancel_sent && Clock::now() > deadline) {
      throw std::runtime_error("move_group did not stop the motion after a cancel request");
    }
  }

  const auto wrapped = result_future.get();
  const std::string code_text =
    wrapped.result ?
    moveit::core::errorCodeToString(moveit::core::MoveItErrorCode(wrapped.result->error_code)) :
    "no result";
  if (timed_out) {
    return {ExecutionStatus::Failed, "move_group did not finish the motion in time; stopped it"};
  }
  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
    return {ExecutionStatus::Succeeded, ""};
  }
  if (cancel_sent) {
    return {ExecutionStatus::Canceled, "stopped on a cancel request (" + code_text + ")"};
  }
  return {ExecutionStatus::Failed, "move_group aborted the motion: " + code_text};
}

}  // namespace manipulation_skill_server
