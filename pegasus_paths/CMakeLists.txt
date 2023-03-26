cmake_minimum_required(VERSION 3.10.2)
project(pegasus_paths)

# Default to C++20 and compiler flags to give all warnings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-sign-compare -O3)
endif()
if(MSVC)
  add_compile_options(-WX -W2)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pegasus_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package("paths" REQUIRED)
find_package(pid REQUIRED)
find_package(thrust_curves REQUIRED)
find_package(pegasus_utils REQUIRED)
find_package(casadi_vendor REQUIRED)
find_package(casadi REQUIRED)

# Include directories for the code
include_directories(
  include/pegasus_paths
  ${EIGEN3_INCLUDE_DIR}
)

# Define the executable
add_executable(${PROJECT_NAME} 
  # The main ROS2 node implementation
  src/paths_node.cpp
  src/pid_controller.cpp
  src/onboard_controller.cpp
  src/mpc.cpp
  src/main.cpp
)

# Add casadi libraries for linking the project
target_link_libraries(${PROJECT_NAME} casadi)

# Add the ROS2 packages to link against
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pegasus_msgs
  nav_msgs
  "paths"
  pid
  thrust_curves
  pegasus_utils
  casadi_vendor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Specify where to install the compiled code
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

# Specify where to install the launch files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()