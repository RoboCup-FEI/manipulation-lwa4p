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

#include <cstdint>
#include <map>
#include <optional>
#include <string>

namespace manipulation_skill_server
{

/// @brief A position in metres.
struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// @brief An orientation; the default is the identity.
struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

/// @brief A pose relative to a frame that the owner of the pose names.
struct Pose
{
  Vector3 position_m;
  Quaternion orientation;
};

/// @brief A pose in a named TF frame at a point in time.
struct StampedPose
{
  std::string frame_id;
  /// Time of the pose in nanoseconds since the ROS epoch; 0 means the latest transform.
  std::int64_t stamp_ns{0};
  Pose pose;
};

/// @brief One target pose of the tool centre point (TCP) with the producer's estimate.
struct TcpCandidate
{
  /// Unique and non-empty within one goal.
  std::string id;
  StampedPose tcp_pose;
  /// Higher is better; empty when the producer gave no estimate.
  std::optional<double> quality;
};

/// @brief How the arm moves to its target.
///
/// The values equal the profile constants of `GoToPosition.action`, so the ROS layer can
/// cast the received number and leave its validation to `GoToPositionTask`.
enum class MotionProfile : std::uint8_t
{
  Ptp = 0,  ///< Joint-space motion.
  Lin = 1,  ///< Straight TCP line; the whole line or a failure.
};

/// @brief Joint positions by joint name: radians for revolute, metres for prismatic joints.
using JointTarget = std::map<std::string, double>;

/// @brief How an action ended; mirrors `Outcome.msg`, converted only in the ROS layer.
enum class OutcomeCode
{
  Success,
  Canceled,
  InvalidGoal,
  Busy,
  PlanningFailed,
  GripFailed,
  ExecutionFailed,
  InternalError,
};

/// @brief An outcome code with a readable cause for people and logs.
struct Outcome
{
  OutcomeCode code{OutcomeCode::InternalError};
  std::string message;
};

}  // namespace manipulation_skill_server
