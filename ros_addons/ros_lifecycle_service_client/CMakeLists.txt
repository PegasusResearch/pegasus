cmake_minimum_required(VERSION 3.10.2)
project(ros_lifecycle_service_client)

# Default to C++17 and compiler flags to give all warnings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()
if(MSVC)
  add_compile_options(-WX -W2)
endif()

# Find dependencies packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(ros_service_client REQUIRED)

# Set the dependencies
set(dependencies
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
    ros_service_client
)

# Define the include directories
include_directories(include)

# Setup the current library
set(library_name ${PROJECT_NAME})

# Add the sources to the library
add_library(${library_name} SHARED
  src/lifecycle_service_client.cpp
)

ament_target_dependencies(${library_name} ${dependencies})

# Define the instalatio directories for the library
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Define the install directory for the headers
install(DIRECTORY include/
  DESTINATION include/
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

# Inform ament compile system that this ROS2 package library
ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()