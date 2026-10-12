cmake_minimum_required(VERSION 3.8)
project(sfg_go2_trajectory_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(trajectory_controller
  "src/trajectory_controller.cpp"
  "src/trajectory_controller_main.cpp"
)
target_include_directories(trajectory_controller PRIVATE "include")
target_compile_features(trajectory_controller PRIVATE cxx_std_17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(trajectory_controller PRIVATE -Wall -Wextra -Wpedantic)
endif()
target_link_libraries(trajectory_controller
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
)
install(TARGETS trajectory_controller DESTINATION lib/${PROJECT_NAME})

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
