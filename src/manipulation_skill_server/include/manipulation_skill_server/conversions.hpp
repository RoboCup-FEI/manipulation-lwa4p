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

#include "manipulation_interfaces/action/go_to_position.hpp"
#include "manipulation_interfaces/msg/outcome.hpp"
#include "manipulation_skill_server/go_to_position_task.hpp"
#include "manipulation_skill_server/types.hpp"

namespace manipulation_skill_server
{

/// @brief Converts a received goal without validating it; `GoToPositionTask` validates.
///
/// A NaN quality becomes "no estimate"; the profile number is kept even if it is unknown.
/// @param goal The goal as received.
/// @return The request for the core.
GoToPositionRequest to_request(const manipulation_interfaces::action::GoToPosition::Goal & goal);

/// @param result The result of the core.
/// @return The action result.
manipulation_interfaces::action::GoToPosition::Result to_ros_result(
  const GoToPositionResult & result);

/// @brief The only conversion of outcome codes to `Outcome.msg`.
/// @param outcome The outcome of the core.
/// @return The outcome message.
manipulation_interfaces::msg::Outcome to_msg(const Outcome & outcome);

/// @param code An outcome code.
/// @return Its constant name in `Outcome.msg`, such as "PLANNING_FAILED", for logs.
const char * to_string(OutcomeCode code);

}  // namespace manipulation_skill_server
