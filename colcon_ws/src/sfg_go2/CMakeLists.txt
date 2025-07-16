cmake_minimum_required(VERSION 3.8)
project(sfg_go2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ########################################################################################################
# Find dependencies.                                                                                     #
# ########################################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# ########################################################################################################
# Add nodes and libraries.                                                                               #
# ########################################################################################################
set(executable_name "locomotion_controller")
set(target_name "${executable_name}_component")
set(export_name "export_${target_name}")
add_library("${target_name}" SHARED "src/locomotion_controller.cpp" "src/locomotion_controller_component.cpp")
rclcpp_components_register_node(
  "${target_name}"
  PLUGIN "sfg_go2::LocomotionController"
  EXECUTABLE "${executable_name}"
)
target_include_directories("${target_name}" PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features("${target_name}" PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies("${target_name}"
  rclcpp
  rclcpp_components
)
ament_export_targets("${export_name}")
install(
  TARGETS "${target_name}"
  EXPORT "${export_name}"
  LIBRARY DESTINATION "lib"
  ARCHIVE DESTINATION "lib"
  RUNTIME DESTINATION "bin"
  INCLUDES DESTINATION "include"
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