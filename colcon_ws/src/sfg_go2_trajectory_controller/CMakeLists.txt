cmake_minimum_required(VERSION 3.8)
project(sfg_go2_trajectory_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sfg_agent_msgs REQUIRED)
find_package(sfg_utils REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

sfg_utils__add_component("trajectory_controller"
  "${PROJECT_NAME}::TrajectoryController"
  SOURCES
  "src/trajectory_controller.cpp"
  "src/control.cpp"
  "src/trajectory.cpp"
  "src/trajectory_controller_component.cpp"
  DEPENDENCIES
  PRIVATE
  rclcpp_components::component
  PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${geometry_msgs_TARGETS}
  ${sfg_agent_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  EXPORT_DEPENDENCIES
  rclcpp
  rclcpp_action
  geometry_msgs
  sfg_agent_msgs
  tf2
  tf2_ros
)

install(DIRECTORY
  "launch"
  "config"
  DESTINATION "share/${PROJECT_NAME}/"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
