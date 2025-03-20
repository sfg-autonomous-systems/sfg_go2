cmake_minimum_required(VERSION 3.8)
project(go2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(realsense2 REQUIRED)
find_package(unitree_sdk2 REQUIRED)

find_library(LIVOX_LIDAR_SDK_LIBRARY "liblivox_lidar_sdk_shared.so" "/usr/local/lib" REQUIRED)
find_path(LIVOX_LIDAR_SDK_INCLUDE_DIRECTORY NAMES "livox_lidar_api.h" "livox_lidar_def.h" REQUIRED)

# Add launch files
install(DIRECTORY "launch"
  DESTINATION "share/${PROJECT_NAME}"
)

# Add nodes.
set(TARGET rgbd_camera_node)
add_executable(${TARGET}
  "src/rgbd_camera_node.cpp"
  "src/rgbd_camera.cpp"
)
target_include_directories(${TARGET} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${TARGET}
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

# Non-ROS dependencies should not use ament_target_dependencies(...)
# (https://robotics.stackexchange.com/questions/96375/ament-target-dependencies-vs-target-link-libraries).
target_link_libraries(${TARGET}
  realsense2::realsense2
)
target_compile_features(${TARGET} PUBLIC c_std_99 cxx_std_17)
install(TARGETS ${TARGET} DESTINATION "lib/${PROJECT_NAME}")

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
