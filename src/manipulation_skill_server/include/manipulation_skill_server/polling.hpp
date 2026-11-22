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
#include <thread>

#include "manipulation_skill_server/task.hpp"

namespace manipulation_skill_server
{

/// @brief How often the waits of the adapters check their condition and the cancel request.
constexpr std::chrono::milliseconds POLL_PERIOD{10};

/// @brief Waits until a condition holds, a deadline passes, or the caller asks to cancel.
///
/// Counts on the steady clock, not on ROS time, which stands still while a simulator is
/// paused.
/// @param ready The condition; checked every `POLL_PERIOD`, so it must not block.
/// @param deadline When to stop waiting.
/// @param cancellation Checked every `POLL_PERIOD`.
/// @return true if the condition holds; false after the deadline or a cancel request.
template<typename ConditionT>
bool wait_until(
  const ConditionT & ready, std::chrono::steady_clock::time_point deadline,
  const CancellationToken & cancellation)
{
  while (!ready()) {
    if (cancellation.cancel_requested() || std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(POLL_PERIOD);
  }
  return true;
}

}  // namespace manipulation_skill_server
