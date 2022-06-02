cmake_minimum_required(VERSION 3.10.2)
project(mavlink_interface)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(MAVSDK REQUIRED)

# Include directories for the code
include_directories(include)

# Define the executable
add_executable(${PROJECT_NAME}_node src/mavlink_node.cpp)

# Add the ROS2 packages to link against
ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp 
    std_msgs
)

# External link libraries
target_link_libraries(${PROJECT_NAME}_node
    MAVSDK::mavsdk
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
install(TARGETS
    ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()