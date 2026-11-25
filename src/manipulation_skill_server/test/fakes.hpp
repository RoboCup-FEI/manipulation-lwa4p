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
#include <chrono>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "manipulation_skill_server/motion_backend.hpp"
#include "manipulation_skill_server/task.hpp"
#include "manipulation_skill_server/types.hpp"

namespace manipulation_skill_server::test
{

/// @brief What the caller of a task sees: the cancel flag it sets and the results it gets.
template<typename ResultT>
class ResultRecord
{
public:
  void request_cancel() {cancel_.store(true);}

  bool cancel_requested() const {return cancel_.load();}

  void add(const ResultT & result)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      results_.push_back(result);
    }
    changed_.notify_all();
    if (on_result) {
      on_result();
    }
  }

  /// Runs on the delivering thread after each result, as a caller reacting to it would.
  /// Set it before the task runs.
  std::function<void()> on_result;

  /// @return true if at least `count` results arrived within the timeout.
  bool wait_for(std::size_t count, std::chrono::milliseconds timeout = std::chrono::seconds(5))
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return changed_.wait_for(lock, timeout, [this, count]() {return results_.size() >= count;});
  }

  std::vector<ResultT> results() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return results_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable changed_;
  std::vector<ResultT> results_;
  std::atomic<bool> cancel_{false};
};

/// @brief A context that writes into a `ResultRecord` the test keeps.
template<typename ResultT>
class FakeTaskContext : public TaskContext<ResultT>
{
public:
  explicit FakeTaskContext(std::shared_ptr<ResultRecord<ResultT>> record)
  : record_(std::move(record))
  {
  }

  bool cancel_requested() const override {return record_->cancel_requested();}

  void finish(const ResultT & result) override {record_->add(result);}

private:
  std::shared_ptr<ResultRecord<ResultT>> record_;
};

class FakePlannedMotion : public PlannedMotion
{
public:
  FakePlannedMotion(std::string candidate_id, double motion_cost)
  : PlannedMotion(std::move(candidate_id), motion_cost)
  {
  }
};

/// @brief A motion backend with scripted plans and execution, recording its calls.
class FakeMotionBackend : public MotionBackend
{
public:
  // Script.
  /// Feasible candidates by id with their motion cost, returned in this order.
  std::vector<std::pair<std::string, double>> candidate_costs;
  /// The cost of the joint-space plan; empty means no plan.
  std::optional<double> joint_plan_cost{1.0};
  std::string planning_message;
  ExecutionResult execution_result{ExecutionStatus::Succeeded, ""};
  /// Runs inside planning, for example to cancel or to throw.
  std::function<void()> during_planning;
  /// Runs inside execution, for example to wait for cancellation or to throw;
  /// it may change `execution_result`.
  std::function<void(const CancellationToken &)> during_execution;

  // Record.
  int plan_calls{0};
  std::vector<TcpCandidate> planned_candidates;
  MotionProfile planned_profile{MotionProfile::Ptp};
  JointTarget planned_joints;
  std::chrono::duration<double> planned_budget{0.0};
  std::vector<std::string> executed_ids;

  PlanResult plan_to_candidates(
    const std::vector<TcpCandidate> & candidates, MotionProfile profile,
    std::chrono::duration<double> budget, const CancellationToken &) override
  {
    ++plan_calls;
    planned_candidates = candidates;
    planned_profile = profile;
    planned_budget = budget;
    if (during_planning) {
      during_planning();
    }
    PlanResult result;
    for (const auto & [id, cost] : candidate_costs) {
      result.plans.push_back(std::make_unique<FakePlannedMotion>(id, cost));
    }
    result.message = planning_message;
    return result;
  }

  PlanResult plan_to_joints(
    const JointTarget & target, std::chrono::duration<double> budget,
    const CancellationToken &) override
  {
    ++plan_calls;
    planned_joints = target;
    planned_budget = budget;
    if (during_planning) {
      during_planning();
    }
    PlanResult result;
    if (joint_plan_cost) {
      result.plans.push_back(std::make_unique<FakePlannedMotion>("", *joint_plan_cost));
    }
    result.message = planning_message;
    return result;
  }

  ExecutionResult execute(
    const PlannedMotion & motion, const CancellationToken & cancellation) override
  {
    executed_ids.push_back(motion.candidate_id());
    if (during_execution) {
      during_execution(cancellation);
    }
    return execution_result;
  }
};

}  // namespace manipulation_skill_server::test
