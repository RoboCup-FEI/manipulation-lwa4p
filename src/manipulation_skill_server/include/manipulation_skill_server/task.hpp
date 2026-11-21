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

#include <memory>
#include <stdexcept>
#include <utility>

#include "manipulation_skill_server/types.hpp"

namespace manipulation_skill_server
{

/// @brief Tells a running task whether its caller wants it to stop.
class CancellationToken
{
public:
  virtual ~CancellationToken() = default;

  /// @brief Safe to call from any thread.
  /// @return true once the caller has asked to cancel the task.
  virtual bool cancel_requested() const = 0;
};

/// @brief The link of one task to its caller, implemented by the ROS layer.
/// @tparam ResultT The result type of the action; it has a member `Outcome outcome`.
template<typename ResultT>
class TaskContext : public CancellationToken
{
public:
  /// @brief Delivers the result to the caller.
  ///
  /// Must not throw: the implementation handles and logs its own delivery errors.
  /// @param result The final result of the task.
  virtual void finish(const ResultT & result) = 0;
};

/// @brief A command that the runner executes on its worker thread.
class TaskInterface
{
public:
  virtual ~TaskInterface() = default;

  /// @brief Does the work and keeps the result for `finish()`; delivers nothing.
  /// @throws Any exception when the task cannot tell what state the robot is in.
  virtual void execute() = 0;

  /// @brief Delivers the result of `execute()`.
  virtual void finish() = 0;

  /// @brief Delivers a result that carries only an outcome.
  ///
  /// The runner uses it instead of `finish()` for a task it refuses and for a task whose
  /// `execute()` threw.
  /// @param outcome The outcome to report.
  virtual void finish_with(const Outcome & outcome) = 0;
};

/// @brief Base of the concrete tasks: keeps the result of `run()` until `finish()`.
/// @tparam ResultT The result type of the action; default-constructible with `Outcome outcome`.
template<typename ResultT>
class Task : public TaskInterface
{
public:
  /// @param context The link to the caller; the task owns it.
  /// @throws std::invalid_argument if `context` is null.
  explicit Task(std::unique_ptr<TaskContext<ResultT>> context)
  : context_(std::move(context))
  {
    if (!context_) {
      throw std::invalid_argument("a task needs a context");
    }
  }

  void execute() final {result_ = run();}

  void finish() final {context_->finish(result_);}

  void finish_with(const Outcome & outcome) final
  {
    ResultT result;
    result.outcome = outcome;
    context_->finish(result);
  }

protected:
  /// @brief The work of the concrete task.
  /// @return The result to deliver.
  virtual ResultT run() = 0;

  /// @return The cancellation state of this task's caller.
  const CancellationToken & cancellation() const {return *context_;}

private:
  std::unique_ptr<TaskContext<ResultT>> context_;
  ResultT result_;
};

}  // namespace manipulation_skill_server
