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

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "manipulation_skill_server/named_pose_table.hpp"

namespace manipulation_skill_server::test
{
namespace
{

TEST(NamedPoseTable, FindsPoseByName)
{
  const NamedPoseTable table({{"home", {{"arm_joint_1", 0.25}}}});

  const JointTarget * home = table.find("home");

  ASSERT_NE(home, nullptr);
  EXPECT_DOUBLE_EQ(home->at("arm_joint_1"), 0.25);
}

TEST(NamedPoseTable, ReturnsNullForUnknownName)
{
  const NamedPoseTable table({{"home", {{"arm_joint_1", 0.0}}}});

  EXPECT_EQ(table.find("work1"), nullptr);
  EXPECT_EQ(table.find(""), nullptr);
}

TEST(NamedPoseTable, RejectsInvalidPoses)
{
  EXPECT_THROW(NamedPoseTable({{"", {{"arm_joint_1", 0.0}}}}), std::invalid_argument);
  EXPECT_THROW(NamedPoseTable({{"home", {}}}), std::invalid_argument);
  EXPECT_THROW(NamedPoseTable({{"home", {{"", 0.0}}}}), std::invalid_argument);
  EXPECT_THROW(
    NamedPoseTable({{"home", {{"arm_joint_1", std::numeric_limits<double>::quiet_NaN()}}}}),
    std::invalid_argument);
}

}  // namespace
}  // namespace manipulation_skill_server::test
