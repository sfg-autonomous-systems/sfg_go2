cmake_minimum_required(VERSION 3.8)
project(sfg_go2_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ########################################################################################################
# Find dependencies.                                                                                     #
# ########################################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sfg_agent_msgs REQUIRED)
find_package(sfg_hardware_interface REQUIRED)
find_package(sfg_utils REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(std_msgs REQUIRED)

# ########################################################################################################
# Add nodes and libraries.                                                                               #
# ########################################################################################################
sfg_utils__add_component("robot_state_bridge" "${PROJECT_NAME}::RobotStateBridge"
  SOURCES
  "src/robot_state_bridge.cpp"
  "src/robot_state_bridge_component.cpp"
  AMENT_DEPENDENCIES
  rclcpp
  rclcpp_components
  sensor_msgs
  sfg_utils
  SYSTEM_DEPENDENCIES
  unitree_sdk2
)

sfg_utils__add_component("locomotion_controller" "${PROJECT_NAME}::LocomotionController"
  SOURCES
  "src/locomotion_controller.cpp"
  "src/locomotion_controller_component.cpp"
  AMENT_DEPENDENCIES
  rclcpp
  rclcpp_components
  sfg_agent_msgs
  sfg_hardware_interface
  sfg_utils
  std_msgs
  SYSTEM_DEPENDENCIES
  unitree_sdk2
)

# ########################################################################################################
# Add other files/directories.                                                                           #
# ########################################################################################################
install(DIRECTORY
  "config"
  "launch"
  DESTINATION "share/${PROJECT_NAME}/"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # The following line skips the linter which checks for copyrights.
  # Comment the line when a copyright and license is added to all source files.
  set(ament_cmake_copyright_FOUND TRUE)

  # The following line skips cpplint (only works in a git repo).
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files.
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()