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

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "manipulation_skill_server/task.hpp"

namespace manipulation_skill_server
{

/// @brief Runs one task at a time on its own worker thread, without a queue.
///
/// A task submitted while another one runs ends at once with `BUSY` and never runs.
/// When a task throws, the runner is faulted for good: the robot may be in an unknown
/// state, so every later task ends with `INTERNAL_ERROR` until the process restarts.
/// Every task the runner receives gets exactly one result.
/// The worker is free before it delivers a result, so the caller may send its next goal
/// as soon as it has the result.
class BusyTaskRunner
{
public:
  /// @brief Starts the worker thread.
  BusyTaskRunner();

  /// @brief Waits for the running task to return, then stops the worker thread.
  ///
  /// Cancel the running task first: the destructor does not interrupt it.
  ~BusyTaskRunner();

  BusyTaskRunner(const BusyTaskRunner &) = delete;
  BusyTaskRunner & operator=(const BusyTaskRunner &) = delete;

  /// @brief Hands a task to the worker and returns without waiting for it.
  ///
  /// A refused task gets its result on the calling thread before this returns.
  /// @param task The task to run; the runner takes ownership.
  /// @throws std::invalid_argument if `task` is null.
  void submit(std::unique_ptr<TaskInterface> task);

private:
  void worker_loop();
  bool execute_task(TaskInterface & task);

  std::mutex mutex_;
  std::condition_variable task_ready_;
  std::unique_ptr<TaskInterface> pending_task_;
  bool busy_{false};
  bool stopping_{false};
  bool faulted_{false};
  std::string fault_message_;
  // Declared last: the worker starts only after the members it uses exist.
  std::thread worker_;
};

}  // namespace manipulation_skill_server
