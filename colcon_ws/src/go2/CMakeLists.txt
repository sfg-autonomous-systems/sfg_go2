cmake_minimum_required(VERSION 3.8)
project(go2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(ament_cmake_auto REQUIRED)
find_package(ament_lint_auto REQUIRED)
ament_auto_find_build_dependencies()

# Add launch files.
ament_auto_package(INSTALL_TO_SHARE
  "config"
  "launch"
)

# Add nodes.
# ...
#
if(BUILD_TESTING)
  # The following line skips the linter which checks for copyrights.
  # Comment the line when a copyright and license is added to all source files.
  set(ament_cmake_copyright_FOUND TRUE)

  # The following line skips cpplint (only works in a git repo).
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files.
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
