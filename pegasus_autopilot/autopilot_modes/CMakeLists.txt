cmake_minimum_required(VERSION 3.10.2)
project(autopilot_modes)

# Default to C++17 and compiler flags to give all warnings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()
if(MSVC)
  add_compile_options(-WX -W2)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)

# Pegasus libraries
find_package(autopilot_common REQUIRED)
find_package(pegasus_utils REQUIRED)
find_package(pegasus_msgs REQUIRED)
find_package("paths" REQUIRED)

# Create the package with source code
add_library(
  ${PROJECT_NAME}
  src/mode_arm.cpp
  src/mode_disarm.cpp
  src/mode_follow_trajectory.cpp
  src/mode_hold.cpp
  src/mode_land.cpp
  src/mode_pass_through.cpp
  src/mode_takeoff.cpp
  src/mode_waypoint.cpp
)

# Add the include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  autopilot_common
  pegasus_utils
  pegasus_msgs
  "paths"
)

# Let the pluginlib know that we are creating multiple mode plugins
pluginlib_export_plugin_description_file(autopilot_modes plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT "${PROJECT_NAME}Targets"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
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

ament_export_dependencies()
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Specify the export targets for the header only libraries
ament_export_targets("${PROJECT_NAME}Targets" HAS_LIBRARY_TARGET)
ament_package()