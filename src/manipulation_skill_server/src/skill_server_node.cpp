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

#include <pthread.h>
#include <signal.h>

#include <exception>
#include <memory>
#include <thread>

#include "manipulation_skill_server/mtc_motion_backend.hpp"
#include "manipulation_skill_server/skill_server_ros.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // SIGINT and SIGTERM are handled here, not by rclcpp: the server must cancel a running
  // motion while ROS still works, and only then may ROS shut down. Blocked before any thread
  // starts, so every thread inherits the mask and sigwait() below receives them.
  sigset_t stop_signals;
  sigemptyset(&stop_signals);
  sigaddset(&stop_signals, SIGINT);
  sigaddset(&stop_signals, SIGTERM);
  pthread_sigmask(SIG_BLOCK, &stop_signals, nullptr);
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

  rclcpp::NodeOptions options;
  // MoveIt and MTC read their parameters from this node without declaring them.
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("skill_server", options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  int exit_code = 0;
  try {
    auto server = std::make_unique<manipulation_skill_server::SkillServer>(
      node, std::make_unique<manipulation_skill_server::MtcMotionBackend>(node));
    int signal_number = 0;
    sigwait(&stop_signals, &signal_number);
    RCLCPP_INFO(node->get_logger(), "stopping");
    server.reset();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(node->get_logger(), "%s", exception.what());
    exit_code = 1;
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return exit_code;
}
