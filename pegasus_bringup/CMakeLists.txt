cmake_minimum_required(VERSION 3.10.2)
project(pegasus_bringup)

# Add -Wno-dev to CLion CMake options to suppress warning about gazebo_dev
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Specify where to install the launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_auto_package()