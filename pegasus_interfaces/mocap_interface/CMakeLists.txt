cmake_minimum_required(VERSION 3.5)
project(mocap_interface)

# Default to C++17 and setup the compiler flags for linux and macOS
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vrpn_vendor REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Setup the local ROS2 package
include_directories(
  include
)

# Compile the vrpn_client_node
add_executable(vrpn_client_node 
  src/vrpn_client_node.cpp
)

# Add vrpn libraries for linking the project
target_link_libraries(vrpn_client_node)

# Add the ROS2 packages to link against
ament_target_dependencies(vrpn_client_node 
  rclcpp 
  std_msgs 
  geometry_msgs 
  vrpn_vendor
)

# Install the launch, configuration files and build targets
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(TARGETS vrpn_client_node DESTINATION lib/${PROJECT_NAME})

ament_package()
