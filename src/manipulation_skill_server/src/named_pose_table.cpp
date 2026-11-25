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

#include "manipulation_skill_server/named_pose_table.hpp"

#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

namespace manipulation_skill_server
{

NamedPoseTable::NamedPoseTable(std::map<std::string, JointTarget> poses)
: poses_(std::move(poses))
{
  for (const auto & [name, joints] : poses_) {
    if (name.empty()) {
      throw std::invalid_argument("a named pose has an empty name");
    }
    if (joints.empty()) {
      throw std::invalid_argument("named pose '" + name + "' has no joints");
    }
    for (const auto & [joint, position] : joints) {
      if (joint.empty()) {
        throw std::invalid_argument("named pose '" + name + "' has a joint with an empty name");
      }
      if (!std::isfinite(position)) {
        throw std::invalid_argument(
          "named pose '" + name + "': joint '" + joint + "' has a non-finite position");
      }
    }
  }
}

const JointTarget * NamedPoseTable::find(const std::string & name) const
{
  const auto pose = poses_.find(name);
  if (pose == poses_.end()) {
    return nullptr;
  }
  return &pose->second;
}

}  // namespace manipulation_skill_server
