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
#include <cstdint>
#include <future>
#include <initializer_list>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "manipulation_skill_server/parameters.hpp"
#include "manipulation_skill_server/polling.hpp"
#include "moveit/task_constructor/container.h"
#include "moveit/task_constructor/cost_terms.h"
#include "moveit/task_constructor/solvers/cartesian_path.h"
#include "moveit/task_constructor/solvers/joint_interpolation.h"
#include "moveit/task_constructor/solvers/pipeline_planner.h"
#include "moveit/task_constructor/stages/compute_ik.h"
#include "moveit/task_constructor/stages/connect.h"
#include "moveit/task_constructor/stages/fixed_cartesian_poses.h"
#include "moveit/task_constructor/stages/fixed_state.h"
#include "moveit/task_constructor/stages/move_to.h"
#include "moveit/utils/moveit_error_code.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace manipulation_skill_server
{

namespace
{

namespace mtc = moveit::task_constructor;
using Clock = std::chrono::steady_clock;

// How long the candidates of one goal may wait for missing transforms, all together and
// inside the planning budget. Perception stamps a pose when it takes the image, so its
// transforms are normally there already.
constexpr std::chrono::milliseconds TRANSFORM_WAIT{500};
// IK solutions per TCP pose: a six-joint arm has at most eight for one pose.
constexpr std::uint32_t MAX_IK_SOLUTIONS = 8;
// IK search time per candidate, at most half of the candidate's planning time. One KDL attempt
// takes at most kinematics_solver_timeout (0.05 s on the LWA 4P), so this allows about ten.
constexpr std::chrono::milliseconds IK_SEARCH_TIME{500};
// Joint interpolation checks collisions after at most this step of any joint. MTC's default,
// 0.1 rad, moves the fingers up to 9 cm between checks, past thin obstacles that move_group
// then finds in the denser executed trajectory and refuses to execute.
constexpr double INTERPOLATION_STEP_RAD = 0.01;
constexpr char START_STATE[] = "start state";
// How long move_group may take to answer a request for its parameters.
constexpr std::chrono::seconds ANSWER_TIMEOUT{5};
// How long the server waits for move_group's parameters when it starts.
constexpr std::chrono::seconds MOVE_GROUP_TIMEOUT{60};

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

geometry_msgs::msg::PoseStamped to_msg(const StampedPose & pose)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = pose.frame_id;
  // Not negative: GoToPositionTask rejects such a stamp. 0 asks TF for the latest transform.
  msg.header.stamp = rclcpp::Time(pose.stamp_ns);
  msg.pose.position.x = pose.pose.position_m.x;
  msg.pose.position.y = pose.pose.position_m.y;
  msg.pose.position.z = pose.pose.position_m.z;
  msg.pose.orientation.x = pose.pose.orientation.x;
  msg.pose.orientation.y = pose.pose.orientation.y;
  msg.pose.orientation.z = pose.pose.orientation.z;
  msg.pose.orientation.w = pose.pose.orientation.w;
  return msg;
}

std::string join(const std::vector<std::string> & parts, const std::string & separator)
{
  std::string joined;
  for (const std::string & part : parts) {
    joined += (joined.empty() ? "" : separator) + part;
  }
  return joined;
}

}  // namespace

MtcMotionBackend::MtcMotionBackend(rclcpp::Node::SharedPtr node)
: node_(std::move(node)),
  arm_group_(required_parameter<std::string>(*node_, "mtc.arm_group")),
  tcp_frame_(required_parameter<std::string>(*node_, "mtc.tcp_frame")),
  executor_(node_)
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
  if (!robot_model_->getJointModelGroup(arm_group_)->canSetStateFromIK(tcp_frame_)) {
    throw std::invalid_argument(
      "mtc.tcp_frame: the IK solver of group '" + arm_group_ + "' cannot place link '" +
      tcp_frame_ + "'");
  }

  // MTC ignores MoveIt's default scaling factors; without them it would move faster than
  // MoveIt allows (AGENTS.md, section 7).
  const double velocity_scaling =
    scaling_factor(*node_, "robot_description_planning.default_velocity_scaling_factor");
  const double acceleration_scaling =
    scaling_factor(*node_, "robot_description_planning.default_acceleration_scaling_factor");
  auto interpolation = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation->setProperty("max_step", INTERPOLATION_STEP_RAD);
  auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(
    node_, parameter<std::string>(*node_, "mtc.planning_pipeline", "ompl"));
  auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
  // LIN is the whole line or a failure (AGENTS.md, section 10).
  cartesian->setMinFraction(1.0);
  for (const mtc::solvers::PlannerInterfacePtr & planner :
    std::initializer_list<mtc::solvers::PlannerInterfacePtr>{
      interpolation, pipeline, cartesian})
  {
    planner->setMaxVelocityScalingFactor(velocity_scaling);
    planner->setMaxAccelerationScalingFactor(acceleration_scaling);
  }
  ptp_planners_ = {{"joint interpolation", interpolation}, {"planning pipeline", pipeline}};
  cartesian_planner_ = cartesian;

  scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node_);
}

PlanResult MtcMotionBackend::plan_to_candidates(
  const std::vector<TcpCandidate> & candidates, MotionProfile profile,
  std::chrono::duration<double> budget, const CancellationToken & cancellation)
{
  const auto start = Clock::now();
  const auto deadline = start + std::chrono::duration_cast<Clock::duration>(budget);
  PlanResult result;
  const planning_scene::PlanningScenePtr scene = current_scene(deadline, cancellation);
  if (!scene) {
    result.message = "move_group did not send its planning scene within the planning budget";
    return result;
  }
  std::vector<std::string> failures;

  // One wait for all transforms: candidates from one image share their frame and stamp, so
  // they would otherwise wait one after another for the same missing transform.
  const auto transforms_wait_until = std::min(deadline, start + TRANSFORM_WAIT);
  std::vector<std::pair<const TcpCandidate *, geometry_msgs::msg::PoseStamped>> targets;
  for (const TcpCandidate & candidate : candidates) {
    try {
      targets.emplace_back(
        &candidate, to_model_frame(candidate.tcp_pose, transforms_wait_until, cancellation));
    } catch (const tf2::TransformException & exception) {
      failures.push_back("candidate '" + candidate.id + "': " + exception.what());
    }
  }

  for (std::size_t index = 0; index < targets.size(); ++index) {
    if (cancellation.cancel_requested()) {
      break;
    }
    const auto & [candidate, target] = targets[index];
    const std::string name = "candidate '" + candidate->id + "'";
    const auto candidates_left = static_cast<Clock::rep>(targets.size() - index);
    const std::chrono::duration<double> share = (deadline - Clock::now()) / candidates_left;
    if (share.count() <= 0.0) {
      failures.push_back(name + ": the planning budget ran out before it");
      continue;
    }

    const std::string task_name = "go_to_position: " + name;
    TaskPtr task = profile == MotionProfile::Lin ? make_lin_task(task_name, scene, target) :
      make_ptp_task(task_name, scene, target, share);
    PlanResult planned = plan(std::move(task), candidate->id, share, cancellation);
    if (planned.plans.empty()) {
      RCLCPP_INFO(node_->get_logger(), "%s: %s", name.c_str(), planned.message.c_str());
      failures.push_back(name + ": " + planned.message);
      continue;
    }
    RCLCPP_INFO(
      node_->get_logger(), "%s: motion cost %.3f", name.c_str(),
      planned.plans.front()->motion_cost());
    result.plans.push_back(std::move(planned.plans.front()));
  }
  result.message = join(failures, "; ");
  return result;
}

PlanResult MtcMotionBackend::plan_to_joints(
  const JointTarget & target, std::chrono::duration<double> budget,
  const CancellationToken & cancellation)
{
  const planning_scene::PlanningScenePtr scene =
    current_scene(Clock::now() + std::chrono::duration_cast<Clock::duration>(budget), cancellation);
  if (!scene) {
    PlanResult result;
    result.message = "move_group did not send its planning scene within the planning budget";
    return result;
  }
  TaskPtr task = make_task("go_to_position: joint target", scene);
  // The first planner that finds a plan wins: interpolation gives the shortest joint path.
  auto motion = std::make_unique<mtc::Fallbacks>("motion");
  for (const auto & [planner_name, planner] : ptp_planners_) {
    auto move = std::make_unique<mtc::stages::MoveTo>(planner_name, planner);
    move->setGroup(arm_group_);
    move->setGoal(target);
    move->setTimeout(budget.count());
    move->setCostTerm(std::make_shared<mtc::cost::PathLength>());
    motion->add(std::move(move));
  }
  task->add(std::move(motion));
  return plan(std::move(task), "", budget, cancellation);
}

MtcMotionBackend::TaskPtr MtcMotionBackend::make_task(
  const std::string & name, const planning_scene::PlanningScenePtr & scene) const
{
  auto task = std::make_unique<mtc::Task>();
  task->stages()->setName(name);
  task->setRobotModel(robot_model_);
  task->add(std::make_unique<mtc::stages::FixedState>(START_STATE, scene));
  return task;
}

MtcMotionBackend::TaskPtr MtcMotionBackend::make_ptp_task(
  const std::string & name, const planning_scene::PlanningScenePtr & scene,
  const geometry_msgs::msg::PoseStamped & target, std::chrono::duration<double> timeout) const
{
  TaskPtr task = make_task(name, scene);
  // The IK search takes at most half of the time; the rest plans motions to its solutions.
  const std::chrono::duration<double> ik_time =
    std::min<std::chrono::duration<double>>(IK_SEARCH_TIME, timeout / 2.0);

  // For every IK solution: joint interpolation, and the planning pipeline where it fails.
  auto motion = std::make_unique<mtc::Fallbacks>("motion");
  for (const auto & [planner_name, planner] : ptp_planners_) {
    auto connect = std::make_unique<mtc::stages::Connect>(
      planner_name, mtc::stages::Connect::GroupPlannerVector{{arm_group_, planner}});
    connect->setTimeout((timeout - ik_time).count());
    connect->setCostTerm(std::make_shared<mtc::cost::PathLength>());
    motion->add(std::move(connect));
  }
  task->add(std::move(motion));

  auto pose = std::make_unique<mtc::stages::FixedCartesianPoses>("target pose");
  pose->addPose(target);
  pose->setMonitoredStage(task->stages()->findChild(START_STATE));
  // The first IK attempt starts from the start state, the next ones from random states.
  auto ik = std::make_unique<mtc::stages::ComputeIK>("inverse kinematics", std::move(pose));
  ik->setGroup(arm_group_);
  ik->setIKFrame(tcp_frame_);
  ik->setMaxIKSolutions(MAX_IK_SOLUTIONS);
  ik->setTimeout(ik_time.count());
  ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
  // The motion cost is the path length alone; ComputeIK would add the joint distance again.
  ik->setCostTerm(std::make_shared<mtc::cost::Constant>(0.0));
  task->add(std::move(ik));
  return task;
}

MtcMotionBackend::TaskPtr MtcMotionBackend::make_lin_task(
  const std::string & name, const planning_scene::PlanningScenePtr & scene,
  const geometry_msgs::msg::PoseStamped & target) const
{
  TaskPtr task = make_task(name, scene);
  // CartesianPath has no timeout: it interpolates the line once, step by step.
  auto move = std::make_unique<mtc::stages::MoveTo>("straight line", cartesian_planner_);
  move->setGroup(arm_group_);
  move->setIKFrame(tcp_frame_);
  move->setGoal(target);
  move->setCostTerm(std::make_shared<mtc::cost::PathLength>());
  task->add(std::move(move));
  return task;
}

geometry_msgs::msg::PoseStamped MtcMotionBackend::to_model_frame(
  const StampedPose & pose, Clock::time_point wait_until_time,
  const CancellationToken & cancellation) const
{
  const geometry_msgs::msg::PoseStamped msg = to_msg(pose);
  const std::string & model_frame = robot_model_->getModelFrame();
  const tf2::TimePoint stamp = tf2_ros::fromMsg(msg.header.stamp);
  // Not TF's own timeout: it counts simulation time, which stands still while a simulator is
  // paused.
  wait_until(
    [&]() {return tf_buffer_->canTransform(model_frame, pose.frame_id, stamp);}, wait_until_time,
    cancellation);
  // Throws with the reason if the transform is still missing.
  return tf_buffer_->transform(msg, model_frame);
}

planning_scene::PlanningScenePtr MtcMotionBackend::current_scene(
  Clock::time_point deadline, const CancellationToken & cancellation) const
{
  // Not MTC's CurrentState stage: it waits for move_group's answer without a time limit and
  // creates a node for every task.
  if (!wait_until([this]() {return scene_client_->service_is_ready();}, deadline, cancellation)) {
    return nullptr;
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  // The components CurrentState asks for.
  using Components = moveit_msgs::msg::PlanningSceneComponents;
  request->components.components = Components::SCENE_SETTINGS | Components::ROBOT_STATE |
    Components::ROBOT_STATE_ATTACHED_OBJECTS |
    Components::WORLD_OBJECT_NAMES |
    Components::WORLD_OBJECT_GEOMETRY | Components::OCTOMAP |
    Components::TRANSFORMS | Components::ALLOWED_COLLISION_MATRIX |
    Components::LINK_PADDING_AND_SCALING | Components::OBJECT_COLORS;
  auto response = scene_client_->async_send_request(request);
  const bool answered = wait_until(
    [&response]() {
      return response.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    },
    deadline, cancellation);
  if (!answered) {
    scene_client_->remove_pending_request(response);
    return nullptr;
  }
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  scene->setPlanningSceneMsg(response.get()->scene);
  return scene;
}

PlanResult MtcMotionBackend::plan(
  TaskPtr task, const std::string & candidate_id, std::chrono::duration<double> budget,
  const CancellationToken & cancellation) const
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
    // All solutions within the time: to a TCP pose there is one per IK solution.
    code = task->plan(0);
  } catch (const mtc::InitStageException & exception) {
    std::ostringstream description;
    description << exception;
    result.message = "invalid MTC task: " + description.str();
    return result;
  }
  task->eraseTaskCallback(callback);

  if (task->solutions().empty()) {
    result.message = "no plan (" + moveit::core::errorCodeToString(code) + ")";
    // One line "<stage> (<solutions>/<failures>): <reason>" for the first stage that failed.
    std::ostringstream failure;
    if (task->stages()->explainFailure(failure)) {
      std::string reason = failure.str();
      reason.erase(reason.find_last_not_of('\n') + 1);
      // Some MTC comments start with a space after the colon.
      for (auto at = reason.find("  "); at != std::string::npos; at = reason.find("  ")) {
        reason.erase(at, 1);
      }
      result.message += ": " + reason;
    }
    return result;
  }
  // MTC keeps the solutions sorted by cost: the first is the shortest motion.
  auto solution = task->solutions().front();
  result.plans.push_back(
    std::make_unique<MtcPlannedMotion>(candidate_id, std::move(task), std::move(solution)));
  return result;
}

ExecutionResult MtcMotionBackend::execute(
  const PlannedMotion & motion, const CancellationToken & cancellation)
{
  moveit_task_constructor_msgs::msg::Solution solution;
  dynamic_cast<const MtcPlannedMotion &>(motion).solution().toMsg(solution);
  return executor_.execute(solution, cancellation);
}

}  // namespace manipulation_skill_server
