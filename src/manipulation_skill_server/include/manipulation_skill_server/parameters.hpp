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

#include <stdexcept>
#include <string>

#include "rclcpp/node.hpp"

namespace manipulation_skill_server
{

/// @brief Reads a parameter, declaring it with a default if the node has not declared it.
///
/// The node may declare its parameter overrides automatically (MoveIt and MTC need that),
/// so a parameter from a YAML file may already be declared.
/// @param node The node.
/// @param name The parameter name.
/// @param default_value The value if nobody set the parameter.
/// @return The parameter value.
/// @throws rclcpp::exceptions::InvalidParameterTypeException if the value has another type.
template<typename T>
T parameter(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<T>(name, default_value);
  }
  return node.get_parameter(name).get_value<T>();
}

/// @brief Reads a parameter that has no sensible default.
/// @param node The node; it must declare its parameter overrides automatically.
/// @param name The parameter name.
/// @return The parameter value.
/// @throws std::invalid_argument if the parameter is not set.
/// @throws rclcpp::exceptions::InvalidParameterTypeException if the value has another type.
template<typename T>
T required_parameter(rclcpp::Node & node, const std::string & name)
{
  if (!node.has_parameter(name)) {
    throw std::invalid_argument("the required parameter " + name + " is not set");
  }
  return node.get_parameter(name).get_value<T>();
}

}  // namespace manipulation_skill_server
