cmake_minimum_required(VERSION 3.8)
project(manipulation_skill_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(manipulation_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Threads REQUIRED)

# The core without ROS: tasks, runner and backend interfaces. Its tests link only this.
add_library(${PROJECT_NAME}_core STATIC
  src/busy_task_runner.cpp
  src/go_to_position_task.cpp
  src/named_pose_table.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Threads::Threads)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The ROS layer: parameters, the action server and message conversions, without logic.
add_library(${PROJECT_NAME}_ros STATIC
  src/conversions.cpp
  src/skill_server_ros.cpp
)
target_link_libraries(${PROJECT_NAME}_ros PUBLIC
  ${PROJECT_NAME}_core
  ${manipulation_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)
set_target_properties(${PROJECT_NAME}_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name
      test_busy_task_runner
      test_go_to_position_task
      test_named_pose_table)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME}_core)
  endforeach()
  foreach(test_name
      test_conversions
      test_skill_server_ros)
    ament_add_gtest(${test_name} test/${test_name}.cpp TIMEOUT 120)
    target_link_libraries(${test_name} ${PROJECT_NAME}_ros)
  endforeach()
endif()

ament_package()
