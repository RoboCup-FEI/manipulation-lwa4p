cmake_minimum_required(VERSION 3.8)
project(grasp_point_cv)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(grasp_point_cv_interfaces REQUIRED)

add_executable(grasp_point_cv_node src/grasp_point_cv_node.cpp)
ament_target_dependencies(grasp_point_cv_node
  rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge tf2 grasp_point_cv_interfaces)
target_link_libraries(grasp_point_cv_node ${OpenCV_LIBS})

install(TARGETS grasp_point_cv_node DESTINATION lib/${PROJECT_NAME})

ament_package()
