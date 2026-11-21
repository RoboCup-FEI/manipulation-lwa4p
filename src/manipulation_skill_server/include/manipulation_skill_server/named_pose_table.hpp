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

#include <map>
#include <string>

#include "manipulation_skill_server/types.hpp"

namespace manipulation_skill_server
{

/// @brief Joint-space arm poses that callers refer to by name, such as "home".
class NamedPoseTable
{
public:
  /// @param poses Joint targets by pose name.
  /// @throws std::invalid_argument if a pose name or joint name is empty, a pose has no
  ///   joints, or a position is not finite.
  explicit NamedPoseTable(std::map<std::string, JointTarget> poses);

  /// @param name The pose name.
  /// @return The pose, or nullptr if the table has no pose with this name.
  const JointTarget * find(const std::string & name) const;

private:
  std::map<std::string, JointTarget> poses_;
};

}  // namespace manipulation_skill_server
