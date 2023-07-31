cmake_minimum_required(VERSION 3.10.2)
project(pegasus_console)

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
find_package(nav_msgs REQUIRED)
find_package(pegasus_msgs REQUIRED)
find_package(pegasus_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------
# Get the FTXUI library
include(FetchContent)
set(FETCHCONTENT_UPDATES_DISCONNECTED TRUE)
FetchContent_Declare(ftxui
    GIT_REPOSITORY https://github.com/ArthurSonzogni/ftxui
    GIT_TAG v4.1.1
)

FetchContent_GetProperties(ftxui)
if(NOT ftxui_POPULATED)
  FetchContent_Populate(ftxui)
  add_subdirectory(${ftxui_SOURCE_DIR} ${ftxui_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

FetchContent_MakeAvailable(ftxui)
# ---------------------------------------------------------------------------

# Include directories for the code
include_directories(
  include/pegasus_console
  ${EIGEN3_INCLUDE_DIR}
)

# Define the executable
add_executable(${PROJECT_NAME}
  src/console_ui.cpp
  src/console_node.cpp
  src/main.cpp
)

# Add casadi libraries for linking the project
target_link_libraries(${PROJECT_NAME} ftxui::screen ftxui::dom ftxui::component)

# Add the ROS2 packages to link against
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
  pegasus_msgs
  pegasus_utils
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Specify where to install the compiled code
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

# Specify where to install the launch files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()