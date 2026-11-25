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

#include "manipulation_skill_server/busy_task_runner.hpp"

#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

namespace manipulation_skill_server
{

BusyTaskRunner::BusyTaskRunner()
: worker_([this]() {worker_loop();}) {}

BusyTaskRunner::~BusyTaskRunner()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  task_ready_.notify_one();
  worker_.join();
}

void BusyTaskRunner::submit(std::unique_ptr<TaskInterface> task)
{
  // Without this check a null task would block the runner forever with busy_ set.
  if (!task) {
    throw std::invalid_argument("cannot submit a null task");
  }

  Outcome refusal;
  bool accepted = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (faulted_) {
      refusal = {
        OutcomeCode::InternalError,
        "the server stopped accepting goals after an internal error and must be restarted: " +
        fault_message_};
    } else if (busy_) {
      refusal = {OutcomeCode::Busy, "another action is running"};
    } else {
      // Reserved under the lock, before the worker wakes up, so a second submit sees it.
      busy_ = true;
      pending_task_ = std::move(task);
      accepted = true;
    }
  }

  if (accepted) {
    task_ready_.notify_one();
  } else {
    task->finish_with(refusal);
  }
}

void BusyTaskRunner::worker_loop()
{
  while (true) {
    std::unique_ptr<TaskInterface> task;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      task_ready_.wait(lock, [this]() {return stopping_ || pending_task_ != nullptr;});
      if (!pending_task_) {
        return;
      }
      task = std::move(pending_task_);
    }

    if (!execute_task(*task)) {
      // busy_ stays true: nothing may run after a fault.
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      busy_ = false;
    }
    // Delivered after the worker is free: a caller that sends its next goal as soon as it
    // has this result must not get BUSY.
    task->finish();
  }
}

bool BusyTaskRunner::execute_task(TaskInterface & task)
{
  std::string error;
  try {
    task.execute();
    return true;
  } catch (const std::exception & exception) {
    error = exception.what();
  } catch (...) {
    error = "unknown exception";
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    faulted_ = true;
    fault_message_ = error;
  }
  task.finish_with({OutcomeCode::InternalError, "internal error: " + error});
  return false;
}

}  // namespace manipulation_skill_server
